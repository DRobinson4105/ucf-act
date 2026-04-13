#include "sdkconfig.h"
#ifdef CONFIG_CONTROL_TEST_MODE

#include "control_test_mode.h"

#include <limits.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"

#include "can_protocol.h"
#include "can_twai.h"
#include "control_actuator_runtime.h"
#include "control_config.h"
#include "control_types.h"
#include "dac_mcp4728.h"
#include "motor_dispatch.h"
#include "motor_setup.h"
#include "motor_protocol.h"
#include "motor_rx.h"
#include "motor_types.h"
#include "node_support.h"
#include "relay_dpdt_my5nj.h"
#include "serial_input.h"
#include "status_led.h"

namespace
{

const char *TEST_TAG = "CONTROL_TEST";
constexpr TickType_t TEST_LOOP_INTERVAL = pdMS_TO_TICKS(20);
constexpr TickType_t TEST_CAN_RX_TIMEOUT = pdMS_TO_TICKS(10);
constexpr TickType_t TEST_STATUS_LOG_INTERVAL = pdMS_TO_TICKS(2000);
constexpr TickType_t TEST_MOTOR_QUERY_INTERVAL = pdMS_TO_TICKS(200);
constexpr TickType_t TEST_MOTOR_RX_IDLE_DELAY = pdMS_TO_TICKS(5);
constexpr UBaseType_t TEST_MOTOR_RX_PRIO = 7;
constexpr uint16_t TEST_MOTOR_RX_STACK = 4096;
constexpr int32_t STEERING_TEST_INPUT_MIN = 0;
constexpr int32_t STEERING_TEST_INPUT_MAX = 720;
constexpr int32_t BRAKING_TEST_INPUT_MIN = 0;
constexpr int32_t BRAKING_TEST_INPUT_MAX = PLANNER_BRAKING_MAX_LEVEL;

// Local motor state for test mode (not the global production state)
motor_uim2852_state_t s_test_motor_state = {};
volatile bool s_motor_fault_latched = false;

enum test_state_t : uint8_t
{
	TEST_ARMED,
	TEST_OVERRIDE,
	TEST_SHUTDOWN,
};

typedef struct
{
	int32_t value;
	int32_t max;
} signed_input_accum_t;

const char *actuator_to_string(control_test_actuator_t actuator)
{
	switch (actuator)
	{
	case CONTROL_TEST_ACTUATOR_THROTTLE:
		return "THROTTLE";
	case CONTROL_TEST_ACTUATOR_STEERING:
		return "STEERING";
	case CONTROL_TEST_ACTUATOR_BRAKING:
		return "BRAKING";
	default:
		return "UNKNOWN";
	}
}

const char *fr_state_to_string(fr_state_t state)
{
	switch (state)
	{
	case FR_STATE_NEUTRAL:
		return "NEUTRAL";
	case FR_STATE_FORWARD:
		return "FORWARD";
	case FR_STATE_REVERSE:
		return "REVERSE";
	case FR_STATE_INVALID:
		return "INVALID";
	default:
		return "UNKNOWN";
	}
}

void idle_forever(bool *wdt_reset_failed)
{
	while (true)
	{
		node_task_wdt_reset_or_log(TEST_TAG, "control_test", wdt_reset_failed);
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void test_input_accum_reset(signed_input_accum_t *accum)
{
	if (!accum)
		return;

	accum->value = -1;
}

bool test_input_accum_feed(signed_input_accum_t *accum, int ch)
{
	if (!accum || ch < 0)
		return false;

	if (ch >= '0' && ch <= '9')
	{
		if (accum->value < 0)
			accum->value = 0;
		accum->value = accum->value * 10 + (ch - '0');
		if (accum->value > accum->max)
			accum->value = accum->max;
		return false;
	}

	if ((ch == ' ' || ch == '\r' || ch == '\n') && accum->value >= 0)
		return true;

	test_input_accum_reset(accum);
	return false;
}

int32_t steering_input_to_pulses(int32_t steering_input)
{
	return ((int32_t)steering_input - 360) * (3200 * 50) / 360;
}

int32_t braking_input_to_pulses(int32_t braking_level)
{
	uint8_t clamped_level =
		(braking_level > PLANNER_BRAKING_MAX_LEVEL) ? PLANNER_BRAKING_MAX_LEVEL : (uint8_t)braking_level;
	return ((int32_t)(PLANNER_BRAKING_MAX_LEVEL - clamped_level) * CONFIG_BRAKE_RELEASE_POSITION) /
	       PLANNER_BRAKING_MAX_LEVEL;
}

int32_t clamp_delta_toward_target(int32_t current, int32_t target, int32_t max_delta)
{
	if (max_delta <= 0)
		return target;

	int32_t delta = target - current;
	if (delta > max_delta)
		return current + max_delta;
	if (delta < -max_delta)
		return current - max_delta;
	return target;
}

int32_t compute_test_braking_target(motor_uim2852_state_t *state, int32_t requested_target)
{
	if (!state)
		return requested_target;

	int32_t reference = 0;
	taskENTER_CRITICAL(&state->lock);
	reference = state->pt_last_position;
	taskEXIT_CRITICAL(&state->lock);

	int32_t step_limit = compute_braking_pt_step_limit(state);
	return clamp_delta_toward_target(reference, requested_target, step_limit);
}

void shutdown_throttle_path(void)
{
	(void)dac_mcp4728_disable();
	(void)dac_mcp4728_emergency_stop();
	(void)relay_dpdt_my5nj_deenergize();
}

void shutdown_motor_path(uint8_t node_id)
{
	motor_cmd_t cmd;

	// Stop PT motion
	motor_cmd_st_stop(node_id, true, &cmd);
	(void)motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);

	// Disable motor driver (MO=0)
	motor_cmd_mo_set(node_id, true, MOTOR_MO_STATE_DISABLE, &cmd);
	(void)motor_dispatch_exec(&cmd, pdMS_TO_TICKS(100), pdMS_TO_TICKS(20), nullptr, nullptr, nullptr);
}

/**
 * @brief Update local motor state from a parsed RX frame (mirrors production update_motor_state).
 */
void update_test_motor_state(motor_uim2852_state_t *state, const motor_rx_t *rx)
{
	if (!state || !rx)
		return;

	taskENTER_CRITICAL(&state->lock);
	state->last_response_tick = xTaskGetTickCount();

	if (motor_rx_is_notification(rx) && motor_rx_notification_is_status(rx))
	{
		uint8_t d0 = 0;
		motor_rx_notification_d0(rx, &d0);
		state->driver_enabled = (d0 & 0x04) != 0;
		state->motion_in_progress = ((d0 & 0x03) != 0) && !((rx->data[1] & 0x01) != 0);

		uint8_t d1 = rx->data[1];
		state->stopped = (d1 & 0x01) != 0;
		state->in_position = (d1 & 0x02) != 0;
		state->stall_detected = (d1 & 0x08) != 0;
		state->error_detected = (d1 & 0x80) != 0;

		int32_t pos = 0;
		if (motor_rx_notification_current_position(rx, &pos))
			state->absolute_position = pos;
	}
	else if (motor_rx_is_ack(rx))
	{
		// PA get response — update position
		int32_t pos = 0;
		if (motor_rx_ack_pa_get(rx, &pos))
			state->absolute_position = pos;
	}
	else if (motor_rx_er_is_thrown_error(rx))
	{
		state->error_detected = true;
	}
	taskEXIT_CRITICAL(&state->lock);
}

void log_motor_snapshot(motor_uim2852_state_t *state, const char *label, int32_t requested_input,
                        int32_t target_position, control_test_actuator_t actuator)
{
	if (!state)
		return;

	int32_t absolute_position = 0;
	int32_t current_speed = 0;
	int32_t commanded_target = 0;
	bool driver_on = false;
	bool stopped = false;
	bool in_position = false;
	bool motion_in_progress = false;
	bool fault = false;
	bool stall = false;

	taskENTER_CRITICAL(&state->lock);
	absolute_position = state->absolute_position;
	current_speed = state->current_speed;
	commanded_target = state->target_position;
	driver_on = state->driver_enabled;
	stopped = state->stopped;
	in_position = state->in_position;
	motion_in_progress = state->motion_in_progress;
	fault = state->error_detected;
	stall = state->stall_detected;
	taskEXIT_CRITICAL(&state->lock);

	const char *unit = (actuator == CONTROL_TEST_ACTUATOR_STEERING) ? "deg" : "level";
	ESP_LOGI(TEST_TAG,
	         "%s: input=%ld %s target=%ld actual=%ld cmd_target=%ld speed=%ld driver=%d motion=%d stopped=%d in_pos=%d fault=%d stall=%d",
	         label ? label : "MOTOR", (long)requested_input, unit, (long)target_position, (long)absolute_position,
	         (long)commanded_target, (long)current_speed, driver_on, motion_in_progress, stopped, in_position,
	         fault, stall);
}

/**
 * @brief CAN RX task for test mode: receives frames, feeds dispatch, updates local motor state.
 */
void motor_can_rx_task(void *param)
{
	control_test_mode_context_t *ctx = static_cast<control_test_mode_context_t *>(param);
	if (!ctx)
	{
		vTaskDelete(nullptr);
		return;
	}

	node_task_wdt_add_self_or_log(TEST_TAG, "control_test_rx");
	bool wdt_reset_failed = false;
	twai_message_t msg = {};

	while (true)
	{
		node_task_wdt_reset_or_log(TEST_TAG, "control_test_rx", &wdt_reset_failed);

		if (can_twai_receive(&msg, TEST_CAN_RX_TIMEOUT) != ESP_OK)
		{
			vTaskDelay(TEST_MOTOR_RX_IDLE_DELAY);
			continue;
		}

		ESP_LOGI(TEST_TAG, "CAN RX: id=0x%08lX extd=%d rtr=%d dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X]",
		         (unsigned long)msg.identifier, msg.extd, msg.rtr, (unsigned)msg.data_length_code,
		         msg.data[0], msg.data[1], msg.data[2], msg.data[3],
		         msg.data[4], msg.data[5], msg.data[6], msg.data[7]);

		if (msg.rtr || !msg.extd)
			continue;

		// Feed dispatch queue so motor_dispatch_exec() can match ACKs
		motor_dispatch_enqueue_rx(&msg);

		// Parse and update local state
		motor_rx_t rx = {};
		if (motor_rx_parse(&msg, &rx) != ESP_OK)
			continue;

		// Only process frames from our test motor
		if (rx.producer_id != ctx->motor_node_id)
			continue;

		update_test_motor_state(&s_test_motor_state, &rx);

		// Check for fault conditions
		if (s_test_motor_state.stall_detected || s_test_motor_state.error_detected)
		{
			s_motor_fault_latched = true;
			ESP_LOGE(TEST_TAG, "%s fault: stall=%d err=%d", ctx->motor_label ? ctx->motor_label : "MOTOR",
			         s_test_motor_state.stall_detected, s_test_motor_state.error_detected);

			if (motor_rx_er_is_thrown_error(&rx))
			{
				uint8_t err_code = 0, related_cw = 0, related_sub = 0;
				motor_rx_error_code(&rx, &err_code);
				motor_rx_error_related_cw(&rx, &related_cw);
				motor_rx_error_related_subindex(&rx, &related_sub);
				ESP_LOGE(TEST_TAG, "%s error: code=0x%02X cw=0x%02X sub=%u",
				         ctx->motor_label ? ctx->motor_label : "MOTOR", err_code, related_cw, related_sub);
			}
		}
	}
}

esp_err_t init_throttle_test_components(control_test_mode_context_t *ctx)
{
	if (!ctx || !ctx->dac_cfg || !ctx->relay_cfg || !ctx->driver_inputs || !ctx->driver_inputs_cfg)
		return ESP_ERR_INVALID_ARG;

	esp_err_t err = dac_mcp4728_init(ctx->dac_cfg);
	if (err != ESP_OK)
		return err;

	err = relay_dpdt_my5nj_init(ctx->relay_cfg);
	if (err != ESP_OK)
		return err;

	if (!ctx->bypass_input_pedal_adc)
	{
		err = control_driver_inputs_init_pedal(ctx->driver_inputs, ctx->driver_inputs_cfg);
		if (err != ESP_OK)
			return err;
	}
	else
	{
		ctx->driver_inputs->pedal_ready = true;
	}

	if (!ctx->bypass_input_fr_sensor)
	{
		err = control_driver_inputs_init_fr(ctx->driver_inputs, ctx->driver_inputs_cfg);
		if (err != ESP_OK)
			return err;
	}
	else
	{
		ctx->driver_inputs->fr_ready = true;
	}

	return ESP_OK;
}

/**
 * @brief Helper to send a motor command and check result.
 */
motor_dispatch_result_t exec_cmd(motor_cmd_t *cmd)
{
	ESP_LOGI(TEST_TAG, "TX: id=0x%08lX dlc=%u [%02X %02X %02X %02X %02X %02X %02X %02X]",
	         (unsigned long)cmd->msg.identifier, (unsigned)cmd->msg.data_length_code,
	         cmd->msg.data[0], cmd->msg.data[1], cmd->msg.data[2], cmd->msg.data[3],
	         cmd->msg.data[4], cmd->msg.data[5], cmd->msg.data[6], cmd->msg.data[7]);
	return motor_dispatch_exec(cmd, pdMS_TO_TICKS(200), pdMS_TO_TICKS(50), nullptr, nullptr, nullptr);
}

esp_err_t run_motor_test_setup_plan(control_test_mode_context_t *ctx)
{
	if (!ctx)
		return ESP_ERR_INVALID_ARG;

	const motor_setup_plan_t *plan = nullptr;
	switch (ctx->actuator)
	{
	case CONTROL_TEST_ACTUATOR_BRAKING:
		plan = motor_setup_brake_pt_pv_demo_plan();
		break;
	case CONTROL_TEST_ACTUATOR_STEERING:
		plan = motor_setup_steering_pt_pv_demo_plan();
		break;
	default:
		return ESP_ERR_NOT_SUPPORTED;
	}

	motor_setup_result_t result = {};
	esp_err_t err = motor_setup_run_plan(plan, nullptr, &result);
	if (err != ESP_OK)
		return err;
	if (result.status != MOTOR_SETUP_STATUS_SUCCESS)
		return ESP_FAIL;
	return ESP_OK;
}

/**
 * @brief Initialize the motor dispatch, CAN bus, RX task, and configure + enable the motor for PT mode.
 *
 * PT motion itself is started later, on the first feed, after the startup rows
 * have been queued.
 */
esp_err_t init_motor_test_components(control_test_mode_context_t *ctx)
{
	if (!ctx || ctx->motor_node_id == 0)
		return ESP_ERR_INVALID_ARG;

	s_motor_fault_latched = false;

	// Initialize local motor state
	memset(&s_test_motor_state, 0, sizeof(s_test_motor_state));
	s_test_motor_state.lock = portMUX_INITIALIZER_UNLOCKED;
	s_test_motor_state.pt_frame_time_ms = CONFIG_MOTOR_UIM2852_PT_FRAME_TIME_MS;

	// Init CAN bus
	esp_err_t err = can_twai_init_default(ctx->twai_tx_gpio, ctx->twai_rx_gpio);
	if (err != ESP_OK)
		return err;

	// Init motor dispatch layer
	err = motor_dispatch_init(can_twai_send, 16);
	if (err != ESP_OK)
		return err;

	// Start CAN RX task
	if (xTaskCreate(motor_can_rx_task, "control_test_rx", TEST_MOTOR_RX_STACK, ctx, TEST_MOTOR_RX_PRIO,
	                nullptr) != pdPASS)
	{
		return ESP_FAIL;
	}

	vTaskDelay(pdMS_TO_TICKS(20));

	uint8_t node = ctx->motor_node_id;
	motor_cmd_t cmd;
	motor_dispatch_result_t res;

	ESP_LOGI(TEST_TAG, "%s init: node_id=%u tx=GPIO%d rx=GPIO%d",
	         ctx->motor_label ? ctx->motor_label : "MOTOR", node,
	         (int)ctx->twai_tx_gpio, (int)ctx->twai_rx_gpio);

	// Disable motor first (clean slate)
	motor_cmd_mo_set(node, true, MOTOR_MO_STATE_DISABLE, &cmd);
	res = exec_cmd(&cmd);
	ESP_LOGI(TEST_TAG, "%s MO=0 (disable): result=%d", ctx->motor_label ? ctx->motor_label : "MOTOR", (int)res);
	vTaskDelay(pdMS_TO_TICKS(50));

	// Clear errors
	motor_cmd_er_clear_all(node, true, &cmd);
	res = exec_cmd(&cmd);
	ESP_LOGI(TEST_TAG, "%s ER clear-all: result=%d", ctx->motor_label ? ctx->motor_label : "MOTOR", (int)res);
	vTaskDelay(pdMS_TO_TICKS(50));

	// Reset fault flags — the ER response itself can trigger the RX task's error detection
	s_motor_fault_latched = false;
	taskENTER_CRITICAL(&s_test_motor_state.lock);
	s_test_motor_state.error_detected = false;
	s_test_motor_state.stall_detected = false;
	taskEXIT_CRITICAL(&s_test_motor_state.lock);

	// Query current position
	motor_cmd_pa_get(node, true, &cmd);
	res = exec_cmd(&cmd);
	ESP_LOGI(TEST_TAG, "%s PA get: result=%d", ctx->motor_label ? ctx->motor_label : "MOTOR", (int)res);
	if (res != MOTOR_DISPATCH_RESULT_OK)
	{
		ESP_LOGE(TEST_TAG, "%s position query failed: result=%d", ctx->motor_label ? ctx->motor_label : "MOTOR", (int)res);
		return ESP_FAIL;
	}
	taskENTER_CRITICAL(&s_test_motor_state.lock);
	s_test_motor_state.pt_last_position = s_test_motor_state.absolute_position;
	taskEXIT_CRITICAL(&s_test_motor_state.lock);
	vTaskDelay(pdMS_TO_TICKS(10));

	if (ctx->actuator != CONTROL_TEST_ACTUATOR_BRAKING)
	{
		err = configure_motor_controller_defaults(ctx->motor_node_id, false);
		if (err != ESP_OK)
		{
			ESP_LOGE(TEST_TAG, "%s controller default config failed: %s",
			         ctx->motor_label ? ctx->motor_label : "MOTOR", esp_err_to_name(err));
			return err;
		}
	}

	err = run_motor_test_setup_plan(ctx);
	if (err != ESP_OK)
	{
		ESP_LOGE(TEST_TAG, "%s setup plan failed: %s", ctx->motor_label ? ctx->motor_label : "MOTOR",
		         esp_err_to_name(err));
		return err;
	}

	// Re-synchronize local state after the setup plan primes PT mode and starts motion.
	motor_cmd_pa_get(node, true, &cmd);
	res = exec_cmd(&cmd);
	if (res != MOTOR_DISPATCH_RESULT_OK)
	{
		ESP_LOGE(TEST_TAG, "%s post-setup PA get failed: result=%d", ctx->motor_label ? ctx->motor_label : "MOTOR",
		         (int)res);
		return ESP_FAIL;
	}

	constexpr uint16_t kSetupPlanQueuedRows = 9;
	uint16_t next_write_row = (uint16_t)(MOTOR_UIM2852_PT_FIRST_VALID_ROW + kSetupPlanQueuedRows);
	if (next_write_row > MOTOR_UIM2852_PT_LAST_VALID_ROW)
		next_write_row = MOTOR_UIM2852_PT_FIRST_VALID_ROW;

	// Reset PT bookkeeping
	taskENTER_CRITICAL(&s_test_motor_state.lock);
	s_test_motor_state.pt_write_index = next_write_row;
	s_test_motor_state.pt_prefill_count = 0;
	s_test_motor_state.pt_motion_started = true;
	s_test_motor_state.pt_last_position = s_test_motor_state.absolute_position;
	taskEXIT_CRITICAL(&s_test_motor_state.lock);

	return ESP_OK;
}

void run_throttle_test(control_test_mode_context_t *ctx, bool *wdt_reset_failed)
{
	ESP_LOGI(TEST_TAG, "========================================");
	ESP_LOGI(TEST_TAG, "  CONTROL TEST MODE: THROTTLE");
	ESP_LOGI(TEST_TAG, "  0-4095: set DAC level (type digits + space/enter)");
	ESP_LOGI(TEST_TAG, "  r:   toggle DPDT relay (MY5NJ)");
	ESP_LOGI(TEST_TAG, "  q:   quit (shutdown all)");
	ESP_LOGI(TEST_TAG, "  pedal: manual override (auto re-arms)");
	ESP_LOGI(TEST_TAG, "========================================");

	esp_err_t err = init_throttle_test_components(ctx);
	if (err != ESP_OK)
	{
		ESP_LOGE(TEST_TAG, "Throttle test init failed: %s", esp_err_to_name(err));
		status_led_set_state(NODE_STATE_NOT_READY);
		idle_forever(wdt_reset_failed);
	}

	err = dac_mcp4728_set_level(0);
	if (err != ESP_OK)
	{
		ESP_LOGE(TEST_TAG, "DAC set level 0 failed: %s — cannot arm", esp_err_to_name(err));
		status_led_set_state(NODE_STATE_NOT_READY);
		idle_forever(wdt_reset_failed);
	}

	err = relay_dpdt_my5nj_energize();
	if (err != ESP_OK)
	{
		ESP_LOGE(TEST_TAG, "DPDT relay failed: %s — cannot arm", esp_err_to_name(err));
		(void)dac_mcp4728_disable();
		status_led_set_state(NODE_STATE_NOT_READY);
		idle_forever(wdt_reset_failed);
	}

	vTaskDelay(pdMS_TO_TICKS(50));

	err = dac_mcp4728_enable_autonomous();
	if (err != ESP_OK)
	{
		ESP_LOGE(TEST_TAG, "DAC enable failed: %s — cannot arm", esp_err_to_name(err));
		(void)relay_dpdt_my5nj_deenergize();
		(void)dac_mcp4728_disable();
		status_led_set_state(NODE_STATE_NOT_READY);
		idle_forever(wdt_reset_failed);
	}

	ESP_LOGI(TEST_TAG, "Throttle system ready — DPDT relay ON, DAC autonomous");
	ESP_LOGI(TEST_TAG, "Type 0-4095 + space/enter to set level. r=toggle relay, q=quit");
	ESP_LOGI(TEST_TAG, "F/R software enforced: REVERSE disables throttle, NEUTRAL zeros level");

	uint16_t current_level = 0;
	bool relay_on = true;
	TickType_t last_status_log_tick = 0;
	serial_input_accum_t level_input = SERIAL_INPUT_ACCUM_INIT(4095);

	vTaskDelay(pdMS_TO_TICKS(ctx->fr_debounce_ms + 5));
	control_driver_inputs_update(ctx->driver_inputs, ctx->driver_inputs_cfg, node_get_time_ms(), nullptr);
	fr_state_t fr_arm = control_driver_inputs_fr_state(ctx->driver_inputs);

	test_state_t state = TEST_ARMED;
	if (fr_arm == FR_STATE_REVERSE || fr_arm == FR_STATE_INVALID)
	{
		ESP_LOGW(TEST_TAG, "F/R is %s at arm — entering override until not in reverse", fr_state_to_string(fr_arm));
		(void)dac_mcp4728_disable();
		(void)relay_dpdt_my5nj_deenergize();
		relay_on = false;
		current_level = 0;
		state = TEST_OVERRIDE;
		status_led_set_state(NODE_STATE_NOT_READY);
	}
	else
	{
		ESP_LOGI(TEST_TAG, "Throttle ARMED at level 0 (F/R=%s)", fr_state_to_string(fr_arm));
		status_led_set_state(NODE_STATE_READY);
	}

	while (true)
	{
		node_task_wdt_reset_or_log(TEST_TAG, "control_test", wdt_reset_failed);
		uint32_t now_ms = node_get_time_ms();
		TickType_t now_tick = xTaskGetTickCount();

		control_driver_inputs_update_result_t input_update = {};
		control_driver_inputs_update(ctx->driver_inputs, ctx->driver_inputs_cfg, now_ms, &input_update);
		if (input_update.pedal_runtime_lost)
			ESP_LOGW(TEST_TAG, "PEDAL_INPUT runtime read failure");

		fr_state_t fr_now = control_driver_inputs_fr_state(ctx->driver_inputs);
		if (ctx->bypass_input_fr_sensor)
			fr_now = FR_STATE_FORWARD;
		bool fr_is_reverse = (fr_now == FR_STATE_REVERSE || fr_now == FR_STATE_INVALID);

		bool pedal_is_pressed = control_driver_inputs_pedal_pressed(ctx->driver_inputs, ctx->driver_inputs_cfg);
		if (ctx->bypass_input_pedal_adc)
			pedal_is_pressed = false;

		bool pedal_is_rearmed = control_driver_inputs_pedal_rearmed(ctx->driver_inputs, ctx->driver_inputs_cfg);
		if (ctx->bypass_input_pedal_adc)
			pedal_is_rearmed = true;

		int ch = serial_input_read_char();
		if (ch == 'q' || ch == 'Q')
		{
			ESP_LOGI(TEST_TAG, "Quit requested");
			shutdown_throttle_path();
			status_led_set_state(NODE_STATE_INIT);
			ESP_LOGI(TEST_TAG, "Shutdown complete — idle");
			serial_input_accum_reset(&level_input);
			state = TEST_SHUTDOWN;
		}

		switch (state)
		{
		case TEST_ARMED:
		{
			(void)dac_mcp4728_set_level(current_level);

			if (last_status_log_tick == 0 || (now_tick - last_status_log_tick) >= TEST_STATUS_LOG_INTERVAL)
			{
				ESP_LOGI(TEST_TAG, "Armed: level=%u pedal=%u mV (threshold=%u) fr=%s", current_level,
				         (unsigned)ctx->driver_inputs->pedal_mv, (unsigned)ctx->pedal_threshold_mv,
				         fr_state_to_string(fr_now));
				last_status_log_tick = now_tick;
			}

			if (pedal_is_pressed)
			{
				ESP_LOGI(TEST_TAG, "Pedal pressed (%u mV) — manual override, DPDT relay off",
				         (unsigned)ctx->driver_inputs->pedal_mv);
				(void)dac_mcp4728_disable();
				(void)relay_dpdt_my5nj_deenergize();
				relay_on = false;
				current_level = 0;
				serial_input_accum_reset(&level_input);
				status_led_set_state(NODE_STATE_NOT_READY);
				state = TEST_OVERRIDE;
				break;
			}

			if (fr_is_reverse)
			{
				ESP_LOGW(TEST_TAG, "F/R is %s — override, DPDT relay off", fr_state_to_string(fr_now));
				(void)dac_mcp4728_disable();
				(void)relay_dpdt_my5nj_deenergize();
				relay_on = false;
				current_level = 0;
				serial_input_accum_reset(&level_input);
				status_led_set_state(NODE_STATE_NOT_READY);
				state = TEST_OVERRIDE;
				break;
			}

			if (fr_now == FR_STATE_NEUTRAL && current_level > 0)
			{
				ESP_LOGW(TEST_TAG, "F/R is NEUTRAL — zeroing throttle (was level %u)", current_level);
				esp_err_t zerr = dac_mcp4728_set_level(0);
				if (zerr == ESP_OK)
				{
					current_level = 0;
					status_led_set_state(NODE_STATE_READY);
				}
			}

			if (serial_input_accum_feed(&level_input, ch))
			{
				uint16_t level = (uint16_t)level_input.value;
				serial_input_accum_reset(&level_input);
				if (level != current_level)
				{
					esp_err_t serr = dac_mcp4728_set_level(level);
					if (serr == ESP_OK)
					{
						current_level = level;
						ESP_LOGI(TEST_TAG, "Throttle level: %u", level);
						status_led_set_state(level > 0 ? NODE_STATE_ACTIVE : NODE_STATE_READY);
					}
					else
					{
						ESP_LOGE(TEST_TAG, "DAC set level %u failed: %s", level, esp_err_to_name(serr));
					}
				}
			}
			else if (ch == 'r' || ch == 'R')
			{
				esp_err_t rerr;
				if (relay_on)
				{
					rerr = relay_dpdt_my5nj_deenergize();
					if (rerr == ESP_OK)
					{
						relay_on = false;
						ESP_LOGI(TEST_TAG, "DPDT relay OFF — manual pedal drives Curtis, pedal switch in circuit");
					}
					else
					{
						ESP_LOGE(TEST_TAG, "DPDT relay deenergize failed: %s", esp_err_to_name(rerr));
					}
				}
				else
				{
					esp_err_t dp_err = dac_mcp4728_set_level(0);
					if (dp_err != ESP_OK)
					{
						ESP_LOGE(TEST_TAG, "DAC reset to 0 failed: %s — aborting relay ON", esp_err_to_name(dp_err));
					}
					else
					{
						rerr = relay_dpdt_my5nj_energize();
						if (rerr == ESP_OK)
						{
							relay_on = true;
							current_level = 0;
							ESP_LOGI(TEST_TAG, "DPDT relay ON — DAC drives Curtis (level reset to 0)");
						}
						else
						{
							ESP_LOGE(TEST_TAG, "DPDT relay energize failed: %s", esp_err_to_name(rerr));
						}
					}
				}
			}
			break;
		}

		case TEST_OVERRIDE:
		{
			if (pedal_is_rearmed && !fr_is_reverse)
			{
				ESP_LOGI(TEST_TAG, "Override cleared (pedal rearmed, fr=%s) — re-arming at level 0",
				         fr_state_to_string(fr_now));

				esp_err_t rerr = dac_mcp4728_set_level(0);
				if (rerr == ESP_OK)
					rerr = relay_dpdt_my5nj_energize();
				if (rerr == ESP_OK)
				{
					vTaskDelay(pdMS_TO_TICKS(50));
					rerr = dac_mcp4728_enable_autonomous();
				}

				if (rerr != ESP_OK)
				{
					ESP_LOGE(TEST_TAG, "Re-arm failed: %s — staying in override", esp_err_to_name(rerr));
					(void)relay_dpdt_my5nj_deenergize();
					(void)dac_mcp4728_disable();
					break;
				}

				current_level = 0;
				relay_on = true;
				status_led_set_state(NODE_STATE_READY);
				ESP_LOGI(TEST_TAG, "Throttle ARMED at level 0 — enter command");
				state = TEST_ARMED;
			}
			else if (last_status_log_tick == 0 || (now_tick - last_status_log_tick) >= TEST_STATUS_LOG_INTERVAL)
			{
				ESP_LOGI(TEST_TAG, "Override: manual control (fr=%s, pedal_rearmed=%d — need both clear)",
				         fr_state_to_string(fr_now), pedal_is_rearmed);
				last_status_log_tick = now_tick;
			}
			break;
		}

		case TEST_SHUTDOWN:
			idle_forever(wdt_reset_failed);
			break;
		}

		vTaskDelay(TEST_LOOP_INTERVAL);
	}
}

void run_motor_test(control_test_mode_context_t *ctx, bool *wdt_reset_failed)
{
	if (!ctx || ctx->motor_node_id == 0)
	{
		ESP_LOGE(TEST_TAG, "Missing motor configuration");
		status_led_set_state(NODE_STATE_NOT_READY);
		idle_forever(wdt_reset_failed);
	}

	ESP_LOGI(TEST_TAG, "========================================");
	ESP_LOGI(TEST_TAG, "  CONTROL TEST MODE: %s", actuator_to_string(ctx->actuator));
	if (ctx->actuator == CONTROL_TEST_ACTUATOR_STEERING)
		ESP_LOGI(TEST_TAG, "  Enter steering position 0-720 and press space/enter");
	else
		ESP_LOGI(TEST_TAG, "  Enter braking level 0-%d and press space/enter", PLANNER_BRAKING_MAX_LEVEL);
	ESP_LOGI(TEST_TAG, "  q:   quit (stop and disable motor)");
	ESP_LOGI(TEST_TAG, "========================================");

	esp_err_t err = init_motor_test_components(ctx);
	if (err != ESP_OK)
	{
		ESP_LOGE(TEST_TAG, "%s test init failed: %s", ctx->motor_label ? ctx->motor_label : "MOTOR",
		         esp_err_to_name(err));
		status_led_set_state(NODE_STATE_NOT_READY);
		idle_forever(wdt_reset_failed);
	}

	int32_t requested_input =
		(ctx->actuator == CONTROL_TEST_ACTUATOR_STEERING) ? 360 : BRAKING_TEST_INPUT_MIN;
	int32_t requested_target_position = (ctx->actuator == CONTROL_TEST_ACTUATOR_STEERING)
		                          ? steering_input_to_pulses(requested_input)
		                          : braking_input_to_pulses(requested_input);
	int32_t target_position = (ctx->actuator == CONTROL_TEST_ACTUATOR_BRAKING)
		                          ? compute_test_braking_target(&s_test_motor_state, requested_target_position)
		                          : requested_target_position;
	TickType_t next_feed_tick = 0;
	TickType_t last_status_log_tick = 0;
	TickType_t last_query_tick = 0;
	signed_input_accum_t target_input = {
		.value = -1,
		.max = (ctx->actuator == CONTROL_TEST_ACTUATOR_STEERING) ? STEERING_TEST_INPUT_MAX : BRAKING_TEST_INPUT_MAX,
	};

	status_led_set_state(NODE_STATE_READY);
	log_motor_snapshot(&s_test_motor_state, ctx->motor_label, requested_input, target_position, ctx->actuator);

	while (true)
	{
		node_task_wdt_reset_or_log(TEST_TAG, "control_test", wdt_reset_failed);
		TickType_t now_tick = xTaskGetTickCount();

		int ch = serial_input_read_char();
		if (ch == 'q' || ch == 'Q')
		{
			ESP_LOGI(TEST_TAG, "Quit requested");
			shutdown_motor_path(ctx->motor_node_id);
			status_led_set_state(NODE_STATE_INIT);
			ESP_LOGI(TEST_TAG, "%s shutdown complete — idle", ctx->motor_label ? ctx->motor_label : "MOTOR");
			idle_forever(wdt_reset_failed);
		}

		if (test_input_accum_feed(&target_input, ch))
		{
			requested_input = target_input.value;
			test_input_accum_reset(&target_input);
			requested_target_position = (ctx->actuator == CONTROL_TEST_ACTUATOR_STEERING)
				                  ? steering_input_to_pulses(requested_input)
				                  : braking_input_to_pulses(requested_input);
			target_position = (ctx->actuator == CONTROL_TEST_ACTUATOR_BRAKING)
				                  ? compute_test_braking_target(&s_test_motor_state, requested_target_position)
				                  : requested_target_position;
			if (ctx->actuator == CONTROL_TEST_ACTUATOR_STEERING)
			{
				ESP_LOGI(TEST_TAG, "%s target: %ld deg -> %ld pulses", ctx->motor_label ? ctx->motor_label : "MOTOR",
				         (long)requested_input, (long)target_position);
			}
			else
			{
				ESP_LOGI(TEST_TAG, "%s target: level %ld -> requested=%ld bounded=%ld pulses",
				         ctx->motor_label ? ctx->motor_label : "MOTOR", (long)requested_input,
				         (long)requested_target_position, (long)target_position);
			}
		}

		if (ctx->actuator == CONTROL_TEST_ACTUATOR_BRAKING)
			target_position = compute_test_braking_target(&s_test_motor_state, requested_target_position);

		// Feed PT stream using the shared production helper
		bool fed_frame = false;
		err = feed_pt_stream_if_due(&s_test_motor_state, ctx->motor_node_id, target_position, now_tick,
		                            &next_feed_tick, &fed_frame);
		if (err != ESP_OK)
		{
			ESP_LOGE(TEST_TAG, "%s PT feed failed: %s", ctx->motor_label ? ctx->motor_label : "MOTOR",
			         esp_err_to_name(err));
			status_led_set_state(NODE_STATE_NOT_READY);
		}
		else if (s_motor_fault_latched)
		{
			status_led_set_state(NODE_STATE_NOT_READY);
		}
		else if (fed_frame || s_test_motor_state.motion_in_progress)
		{
			if (ctx->actuator == CONTROL_TEST_ACTUATOR_BRAKING && fed_frame)
			{
				taskENTER_CRITICAL(&s_test_motor_state.lock);
				s_test_motor_state.pt_last_position = target_position;
				taskEXIT_CRITICAL(&s_test_motor_state.lock);
			}
			status_led_set_state(NODE_STATE_ACTIVE);
		}
		else
		{
			status_led_set_state(NODE_STATE_READY);
		}

		// Periodic position query
		if (last_query_tick == 0 || (now_tick - last_query_tick) >= TEST_MOTOR_QUERY_INTERVAL)
		{
			motor_cmd_t cmd;
			motor_cmd_pa_get(ctx->motor_node_id, true, &cmd);
			motor_dispatch_result_t res = exec_cmd(&cmd);
			if (res != MOTOR_DISPATCH_RESULT_OK)
				ESP_LOGW(TEST_TAG, "%s position query failed: result=%d", ctx->motor_label ? ctx->motor_label : "MOTOR",
				         (int)res);
			last_query_tick = now_tick;
		}

		// Periodic status log
		if (last_status_log_tick == 0 || (now_tick - last_status_log_tick) >= TEST_STATUS_LOG_INTERVAL)
		{
			log_motor_snapshot(&s_test_motor_state, ctx->motor_label, requested_input, target_position,
			                   ctx->actuator);
			last_status_log_tick = now_tick;
		}

		vTaskDelay(TEST_LOOP_INTERVAL);
	}
}

} // namespace

void control_test_mode_task(void *param)
{
	control_test_mode_context_t *ctx = static_cast<control_test_mode_context_t *>(param);
	if (!ctx)
	{
		ESP_LOGE(TEST_TAG, "Missing control test mode context");
		vTaskDelete(nullptr);
		return;
	}

	node_task_wdt_add_self_or_log(TEST_TAG, "control_test");
	bool wdt_reset_failed = false;

	esp_err_t console_err = serial_input_init();
	if (console_err != ESP_OK)
		ESP_LOGW(TEST_TAG, "Serial console init failed: %s (serial input may not work)", esp_err_to_name(console_err));

	switch (ctx->actuator)
	{
	case CONTROL_TEST_ACTUATOR_THROTTLE:
		run_throttle_test(ctx, &wdt_reset_failed);
		break;
	case CONTROL_TEST_ACTUATOR_STEERING:
	case CONTROL_TEST_ACTUATOR_BRAKING:
		run_motor_test(ctx, &wdt_reset_failed);
		break;
	default:
		ESP_LOGE(TEST_TAG, "Unknown control test actuator: %d", (int)ctx->actuator);
		status_led_set_state(NODE_STATE_NOT_READY);
		idle_forever(&wdt_reset_failed);
		break;
	}
}
#endif // CONFIG_CONTROL_TEST_MODE
