/**
 * @file stepper_motor_uim2852.cpp
 * @brief UIM2852CA Stepper Motor Control Implementation
 */

#include "stepper_motor_uim2852.h"

#include <string.h>

#include "esp_log.h"
#include "can_twai.h"

static const char *TAG = "STEPPER";

// Default timeout for CAN transmit
static const TickType_t TX_TIMEOUT = pdMS_TO_TICKS(20);

// PT FIFO defaults aligned with the planner's 100 ms command cadence.
static const uint16_t PT_FRAME_TIME_MS_DEFAULT = 100;
static const uint8_t PT_FIFO_LOW_WATER_DEFAULT = 1;
static const uint8_t PT_FIFO_PREFILL_FRAMES = 2;
static const uint16_t PT_START_ROW = 0;
static const uint16_t PT_TABLE_ROWS = 512;

// (Notification callback is now per-motor, stored in stepper_motor_uim2852_t)

// ============================================================================
// Internal Helpers
// ============================================================================

/**
 * @brief Transmit a SimpleCAN3.0 instruction frame to a stepper motor.
 *
 * Builds the 29-bit extended CAN ID from the motor's node ID and
 * command word, optionally sets the ACK request bit, and sends
 * the frame via the TWAI peripheral.  On success, updates the
 * motor's last-command timestamp and ACK-pending flag.
 *
 * @param motor  Motor instance (must be initialized)
 * @param cw     Command word (e.g. STEPPER_UIM2852_CW_MO)
 * @param data   Frame payload bytes
 * @param dl     Data length (0-8)
 * @return ESP_OK on success, or an error code from the TWAI driver
 */
static esp_err_t send_instruction_internal(stepper_motor_uim2852_t *motor, uint8_t cw, const uint8_t *data, uint8_t dl,
                                           bool request_ack)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	// Add ACK bit when explicitly requested for this instruction.
	uint8_t cw_to_send = request_ack ? (cw | STEPPER_UIM2852_CW_ACK_BIT) : cw;

	// Calculate 29-bit CAN ID
	uint32_t can_id = stepper_uim2852_make_can_id(motor->config.node_id, cw_to_send);

	// Send extended frame
	esp_err_t err = can_twai_send_extended(can_id, data, dl, TX_TIMEOUT);

	if (err == ESP_OK)
	{
		motor->last_command_tick = xTaskGetTickCount();
		motor->last_cw_sent = cw;
		if (request_ack)
			motor->ack_pending = true;
	}
#ifdef CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX
	else
		ESP_LOGW(TAG, "Node %u: TX failed for CW=0x%02X: %s", motor->config.node_id, cw, esp_err_to_name(err));
#endif

	return err;
}

static esp_err_t send_instruction(stepper_motor_uim2852_t *motor, uint8_t cw, const uint8_t *data, uint8_t dl)
{
	return send_instruction_internal(motor, cw, data, dl, motor->config.request_ack);
}

static esp_err_t send_ms_query(stepper_motor_uim2852_t *motor, uint8_t index)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_ms(data, index);

	taskENTER_CRITICAL(&motor->lock);
	motor->query_pending_cw = STEPPER_UIM2852_CW_MS;
	motor->query_pending_idx = index;
	taskEXIT_CRITICAL(&motor->lock);

	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_MS, data, dl);
	if (err != ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		if (motor->query_pending_cw == STEPPER_UIM2852_CW_MS && motor->query_pending_idx == index)
		{
			motor->query_pending_cw = 0;
			motor->query_pending_idx = 0;
		}
		taskEXIT_CRITICAL(&motor->lock);
	}

	return err;
}

// ============================================================================
// Initialization
// ============================================================================

void stepper_motor_uim2852_deinit(stepper_motor_uim2852_t *motor)
{
	if (!motor)
		return;
	if (motor->query_sem)
	{
		vSemaphoreDelete(motor->query_sem);
		motor->query_sem = NULL;
	}
	motor->initialized = false;
}

esp_err_t stepper_motor_uim2852_init(stepper_motor_uim2852_t *motor, const stepper_motor_uim2852_config_t *config)
{
	if (!motor)
		return ESP_ERR_INVALID_ARG;

	// Free any previously allocated resources before re-initializing.
	// This prevents semaphore leaks when init is called on an already-
	// initialized motor (e.g. during fault retry).
	stepper_motor_uim2852_deinit(motor);

	memset(motor, 0, sizeof(stepper_motor_uim2852_t));

	if (config)
		motor->config = *config;
	else
	{
		stepper_motor_uim2852_config_t defaults = STEPPER_MOTOR_UIM2852_CONFIG_DEFAULT();
		motor->config = defaults;
	}

	motor->microstep_resolution = 16; // Assume 16 until queried
	motor->pulses_per_rev = 3200;     // 16 * 200
	motor->pt_frame_time_ms = PT_FRAME_TIME_MS_DEFAULT;
	motor->pt_low_water_mark = PT_FIFO_LOW_WATER_DEFAULT;

	// Create binary semaphore for synchronous query_param
	motor->query_sem = xSemaphoreCreateBinary();
	if (!motor->query_sem)
	{
		ESP_LOGE(TAG, "Failed to create query semaphore");
		return ESP_ERR_NO_MEM;
	}
	motor->query_pending_cw = 0;
	portMUX_INITIALIZE(&motor->lock);
	motor->initialized = true;

	return ESP_OK;
}

esp_err_t stepper_motor_uim2852_configure(stepper_motor_uim2852_t *motor)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	uint8_t data[8];
	uint8_t dl;
	esp_err_t err;

	// Clear any stale error/status flags from previous session.
	dl = stepper_uim2852_build_ms_clear(data);
	err = send_instruction(motor, STEPPER_UIM2852_CW_MS, data, dl);
	if (err != ESP_OK)
		return err;

	vTaskDelay(pdMS_TO_TICKS(100));

	taskENTER_CRITICAL(&motor->lock);
	motor->status.error_detected = false;
	taskEXIT_CRITICAL(&motor->lock);

	// Set default motion parameters (accel/decel used as safety limits for PT mode)
	err = stepper_motor_uim2852_set_accel(motor, motor->config.default_accel);
	if (err != ESP_OK)
		return err;

	err = stepper_motor_uim2852_set_decel(motor, motor->config.default_decel);
	if (err != ESP_OK)
		return err;

	// Set emergency stop deceleration
	dl = stepper_uim2852_build_sd(data, motor->config.stop_decel);
	err = send_instruction(motor, STEPPER_UIM2852_CW_SD, data, dl);
	if (err != ESP_OK)
		return err;

	// Enable closed-loop control (IC[6]=1)
	dl = stepper_uim2852_build_ic_set(data, STEPPER_UIM2852_IC_CLOSED_LOOP, 1);
	err = send_instruction(motor, STEPPER_UIM2852_CW_IC, data, dl);
	if (err != ESP_OK)
		return err;

	// Set positive motor direction to clockwise (IC[1]=0)
	dl = stepper_uim2852_build_ic_set(data, STEPPER_UIM2852_IC_DIR_POLARITY, 0);
	err = send_instruction(motor, STEPPER_UIM2852_CW_IC, data, dl);
	if (err != ESP_OK)
		return err;

	// Set working current (MT[1], units of 0.1A)
	dl = stepper_uim2852_build_mt_set(data, STEPPER_UIM2852_MT_WORKING_CURRENT, motor->config.working_current);
	err = send_instruction(motor, STEPPER_UIM2852_CW_MT, data, dl);

	return err;
}

esp_err_t stepper_motor_uim2852_set_limits(stepper_motor_uim2852_t *motor, int32_t lower_limit, int32_t upper_limit)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	esp_err_t err;

	// Set lower working limit (LM[1])
	err = stepper_motor_uim2852_set_param(motor, STEPPER_UIM2852_CW_LM, STEPPER_UIM2852_LM_LOWER_WORK, lower_limit);
	if (err != ESP_OK)
		return err;

	// Set upper working limit (LM[2])
	err = stepper_motor_uim2852_set_param(motor, STEPPER_UIM2852_CW_LM, STEPPER_UIM2852_LM_UPPER_WORK, upper_limit);
	if (err != ESP_OK)
		return err;

	// Enable software limits via IC[7]=1
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_ic_set(data, STEPPER_UIM2852_IC_SOFTWARE_LIMITS, 1);
	err = send_instruction(motor, STEPPER_UIM2852_CW_IC, data, dl);

	return err;
}

// ============================================================================
// Basic Control
// ============================================================================

esp_err_t stepper_motor_uim2852_enable(stepper_motor_uim2852_t *motor)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_mo(data, true);

	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_MO, data, dl);
	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->driver_enabled = true;
		taskEXIT_CRITICAL(&motor->lock);
	}
	return err;
}

esp_err_t stepper_motor_uim2852_disable(stepper_motor_uim2852_t *motor)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_mo(data, false);

	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_MO, data, dl);
	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->driver_enabled = false;
		motor->motion_in_progress = false;
		taskEXIT_CRITICAL(&motor->lock);
	}
	return err;
}

esp_err_t stepper_motor_uim2852_stop(stepper_motor_uim2852_t *motor)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_st(data);

	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_ST, data, dl);
	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->motion_in_progress = false;
		taskEXIT_CRITICAL(&motor->lock);
	}
	return err;
}

esp_err_t stepper_motor_uim2852_emergency_stop(stepper_motor_uim2852_t *motor)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	// First set very high SD rate for emergency (10x configured stop rate)
	uint8_t data[8];
	uint8_t dl;

	uint32_t emergency_decel = motor->config.stop_decel * 10;
	if (emergency_decel < motor->config.stop_decel)
		emergency_decel = UINT32_MAX; // overflow guard
	dl = stepper_uim2852_build_sd(data, emergency_decel);
	esp_err_t sd_err = send_instruction(motor, STEPPER_UIM2852_CW_SD, data, dl);
	(void)sd_err;

	// Issue stop command
	dl = stepper_uim2852_build_st(data);
	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_ST, data, dl);

	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->motion_in_progress = false;
		taskEXIT_CRITICAL(&motor->lock);
	}
	return err;
}

// ============================================================================
// Motion Parameters
// ============================================================================

esp_err_t stepper_motor_uim2852_set_accel(stepper_motor_uim2852_t *motor, uint32_t accel_rate)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_ac(data, accel_rate);
	return send_instruction(motor, STEPPER_UIM2852_CW_AC, data, dl);
}

esp_err_t stepper_motor_uim2852_set_decel(stepper_motor_uim2852_t *motor, uint32_t decel_rate)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_dc(data, decel_rate);
	return send_instruction(motor, STEPPER_UIM2852_CW_DC, data, dl);
}

esp_err_t stepper_motor_uim2852_set_origin(stepper_motor_uim2852_t *motor)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	taskENTER_CRITICAL(&motor->lock);
	bool motion_in_progress = motor->motion_in_progress;
	taskEXIT_CRITICAL(&motor->lock);
	if (motion_in_progress)
		return ESP_ERR_INVALID_STATE;

	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_og(data);

	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_OG, data, dl);
	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->absolute_position = 0;
		motor->target_position = 0;
		motor->status.relative_position = 0;
		taskEXIT_CRITICAL(&motor->lock);
	}
	return err;
}

// ============================================================================
// Position-Time (PT) Interpolation Mode
// ============================================================================

esp_err_t stepper_motor_uim2852_pt_configure(stepper_motor_uim2852_t *motor)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	if (motor->pt_frame_time_ms < 5 || motor->pt_frame_time_ms > 30000)
		return ESP_ERR_INVALID_ARG;

	uint8_t data[8];
	uint8_t dl;
	esp_err_t err;

	// Reset the PT/PVT queue and select PT FIFO mode per the manual:
	// MP[0]=0, MP[1]=0, MP[2]=0, MP[3]=0, MP[4]=frame_time, MP[5]=queue-low.
	const uint8_t cw22_mp0_data[3] = {0x00, 0x00, 0x00};
	err = send_instruction(motor, STEPPER_UIM2852_CW_MP, cw22_mp0_data, 3);
	if (err != ESP_OK)
		return err;

	const uint8_t cw22_first_row_data[3] = {0x01, 0x00, 0x00};
	err = send_instruction(motor, STEPPER_UIM2852_CW_MP, cw22_first_row_data, 3);
	if (err != ESP_OK)
		return err;

	const uint8_t cw22_last_row_data[3] = {0x02, 0x00, 0x00};
	err = send_instruction(motor, STEPPER_UIM2852_CW_MP, cw22_last_row_data, 3);
	if (err != ESP_OK)
		return err;

	const uint8_t cw22_data[3] = {0x03, 0x00, 0x00};
	err = send_instruction(motor, STEPPER_UIM2852_CW_MP, cw22_data, 3);
	if (err != ESP_OK)
		return err;

	data[0] = 0x04;
	data[1] = (uint8_t)(motor->pt_frame_time_ms & 0xFF);
	data[2] = (uint8_t)(motor->pt_frame_time_ms >> 8);
	err = send_instruction(motor, STEPPER_UIM2852_CW_MP, data, 3);
	if (err != ESP_OK)
		return err;

	data[0] = 0x05;
	data[1] = motor->pt_low_water_mark;
	data[2] = 0x00;
	err = send_instruction(motor, STEPPER_UIM2852_CW_MP, data, 3);
	if (err != ESP_OK)
		return err;

	// Enable PVT FIFO empty notification (IE[10]=1)
	dl = stepper_uim2852_build_ie_set(data, STEPPER_UIM2852_IE_PVT_FIFO_EMPTY, 1);
	err = send_instruction(motor, STEPPER_UIM2852_CW_IE, data, dl);
	if (err != ESP_OK)
		return err;

	// Enable PVT FIFO low warning notification (IE[11]=1)
	dl = stepper_uim2852_build_ie_set(data, STEPPER_UIM2852_IE_PVT_FIFO_LOW, 1);
	err = send_instruction(motor, STEPPER_UIM2852_CW_IE, data, dl);
	if (err != ESP_OK)
		return err;

	taskENTER_CRITICAL(&motor->lock);
	motor->pt_mode_active = false;
	motor->pt_motion_started = false;
	motor->pt_fifo_empty = false;
	motor->pt_fifo_low = false;
	motor->pt_write_index = PT_START_ROW;
	motor->pt_prefill_count = 0;
	taskEXIT_CRITICAL(&motor->lock);

#ifdef CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX
	ESP_LOGI(TAG, "Node %u: PT configured (FIFO mode, %u ms frames, low-water=%u)", motor->config.node_id,
	         (unsigned)motor->pt_frame_time_ms, (unsigned)motor->pt_low_water_mark);
#endif
	return ESP_OK;
}

esp_err_t stepper_motor_uim2852_pt_start(stepper_motor_uim2852_t *motor)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_pv(data, PT_START_ROW);
	esp_err_t err = send_instruction(motor, STEPPER_UIM2852_CW_PV, data, dl);

	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->pt_mode_active = true;
		motor->pt_motion_started = false;
		motor->pt_fifo_empty = false;
		motor->pt_fifo_low = false;
		motor->pt_write_index = PT_START_ROW;
		motor->pt_prefill_count = 0;
		taskEXIT_CRITICAL(&motor->lock);
#ifdef CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX
		ESP_LOGI(TAG, "Node %u: PT FIFO armed at row %u", motor->config.node_id, (unsigned)PT_START_ROW);
#endif
	}
	return err;
}

esp_err_t stepper_motor_uim2852_pt_stop(stepper_motor_uim2852_t *motor)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_st(data);
	esp_err_t st_err = send_instruction(motor, STEPPER_UIM2852_CW_ST, data, dl);

	taskENTER_CRITICAL(&motor->lock);
	motor->pt_mode_active = false;
	motor->pt_motion_started = false;
	motor->motion_in_progress = false;
	motor->pt_fifo_empty = false;
	motor->pt_fifo_low = false;
	motor->pt_write_index = PT_START_ROW;
	motor->pt_prefill_count = 0;
	taskEXIT_CRITICAL(&motor->lock);

#ifdef CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX
	ESP_LOGI(TAG, "Node %u: PT interpolation stopped", motor->config.node_id);
#endif

	return st_err;
}

esp_err_t stepper_motor_uim2852_pt_feed(stepper_motor_uim2852_t *motor, int32_t position, uint32_t time_ms)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	// Reject waypoint feeds when PT mode hasn't been started
	if (!motor->pt_mode_active)
		return ESP_ERR_INVALID_STATE;

	if (time_ms != motor->pt_frame_time_ms)
		return ESP_ERR_INVALID_ARG;

	uint8_t data[8];
	uint16_t row = 0;
	bool start_now = false;

	taskENTER_CRITICAL(&motor->lock);
	row = motor->pt_write_index;
	taskEXIT_CRITICAL(&motor->lock);

	uint8_t dl = stepper_uim2852_build_pt(data, row, position);
	// PT row writes are high-rate FIFO traffic; do not request ACK for each row.
	esp_err_t err = send_instruction_internal(motor, STEPPER_UIM2852_CW_PT, data, dl, false);

	if (err == ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->pt_fifo_empty = false;
		motor->pt_fifo_low = false;
		motor->target_position = position;
		motor->pt_write_index = (uint16_t)((motor->pt_write_index + 1) % PT_TABLE_ROWS);
		if (!motor->pt_motion_started && motor->pt_prefill_count < PT_FIFO_PREFILL_FRAMES)
			motor->pt_prefill_count++;
		start_now = (!motor->pt_motion_started && motor->pt_prefill_count >= PT_FIFO_PREFILL_FRAMES);
		taskEXIT_CRITICAL(&motor->lock);

		if (start_now)
		{
			uint8_t bg_data[8];
			uint8_t bg_dl = stepper_uim2852_build_bg(bg_data);
			esp_err_t bg_err = send_instruction_internal(motor, STEPPER_UIM2852_CW_BG, bg_data, bg_dl, false);
			if (bg_err != ESP_OK)
				return bg_err;

			taskENTER_CRITICAL(&motor->lock);
			motor->pt_motion_started = true;
			motor->motion_in_progress = true;
			motor->status.in_position = false;
			motor->status.stopped = false;
			taskEXIT_CRITICAL(&motor->lock);
#ifdef CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX
			ESP_LOGI(TAG, "Node %u: PT FIFO prefilled (%u rows), BG sent", motor->config.node_id,
			         (unsigned)PT_FIFO_PREFILL_FRAMES);
#endif
		}
#ifdef CONFIG_LOG_ACTUATOR_STEPPER_MOTION_TX
		ESP_LOGI(TAG, "Node %u: PT row=%u pos=%ld t=%lu ms", motor->config.node_id, (unsigned)row, (long)position,
		         (unsigned long)time_ms);
#endif
	}
	return err;
}

// ============================================================================
// Status Query
// ============================================================================

esp_err_t stepper_motor_uim2852_query_status(stepper_motor_uim2852_t *motor)
{
	return send_ms_query(motor, STEPPER_UIM2852_MS_FLAGS_RELPOS);
}

esp_err_t stepper_motor_uim2852_query_position(stepper_motor_uim2852_t *motor)
{
	return send_ms_query(motor, STEPPER_UIM2852_MS_SPEED_ABSPOS);
}

esp_err_t stepper_motor_uim2852_clear_status(stepper_motor_uim2852_t *motor)
{
	uint8_t data[8];
	uint8_t dl = stepper_uim2852_build_ms_clear(data);
	return send_instruction(motor, STEPPER_UIM2852_CW_MS, data, dl);
}

// ============================================================================
// CAN Frame Processing
// ============================================================================

bool stepper_motor_uim2852_process_frame(stepper_motor_uim2852_t *motor, const twai_message_t *msg)
{
	if (!motor || !motor->initialized || !msg)
		return false;

	// Must be extended frame
	if (!msg->extd)
		return false;

	// Parse CAN ID
	uint8_t producer_id, cw;
	if (!stepper_uim2852_parse_can_id(msg->identifier, &producer_id, &cw))
		return false;

	// UIM2852 responses publish the motor node ID as the producer ID in the
	// extended CAN identifier, so reject frames from other stepper nodes
	// before any status/notify/error state is updated.
	if (producer_id != motor->config.node_id)
		return false;

	uint8_t cw_base_peek = stepper_uim2852_cw_base(cw);

	bool is_unsolicited = (cw_base_peek == STEPPER_UIM2852_CW_MS || cw_base_peek == STEPPER_UIM2852_CW_NOTIFY ||
	                       cw_base_peek == STEPPER_UIM2852_CW_ER);

	if (!is_unsolicited)
	{
		// This is a query/command response — only accept if we're waiting
		taskENTER_CRITICAL(&motor->lock);
		bool expecting = motor->ack_pending || (motor->query_pending_cw != 0);
		taskEXIT_CRITICAL(&motor->lock);
		if (!expecting)
			return false;
	}

	// Update response timestamp
	taskENTER_CRITICAL(&motor->lock);
	motor->last_response_tick = xTaskGetTickCount();
	motor->ack_pending = false;
	taskEXIT_CRITICAL(&motor->lock);

	uint8_t cw_base = stepper_uim2852_cw_base(cw);
	uint8_t dl = msg->data_length_code;
	const uint8_t *data = msg->data;

	// Handle different response types
	switch (cw_base)
	{
	case STEPPER_UIM2852_CW_MS:
		// Motion status response
		if (dl >= 1)
		{
			uint8_t index = data[0];
			if (index == STEPPER_UIM2852_MS_FLAGS_RELPOS)
			{
				taskENTER_CRITICAL(&motor->lock);
				stepper_uim2852_parse_ms0(data, dl, &motor->status);

				// Update motion state from flags
				motor->driver_enabled = motor->status.driver_on;
				if (motor->status.in_position || motor->status.stopped)
					motor->motion_in_progress = false;
				taskEXIT_CRITICAL(&motor->lock);
			}
			else if (index == STEPPER_UIM2852_MS_SPEED_ABSPOS)
			{
				taskENTER_CRITICAL(&motor->lock);
				stepper_uim2852_parse_ms1(data, dl, &motor->current_speed, &motor->absolute_position);
				taskEXIT_CRITICAL(&motor->lock);
			}
		}
		break;

	case STEPPER_UIM2852_CW_NOTIFY:
		// Real-time notification
		{
			stepper_uim2852_notification_t notif = {};

			if (stepper_uim2852_parse_notification(data, dl, &notif))
			{
				// Handle specific notifications
				if (notif.type == STEPPER_UIM2852_STATUS_PTP_COMPLETE)
				{
					taskENTER_CRITICAL(&motor->lock);
					motor->motion_in_progress = false;
					motor->status.in_position = true;
					motor->absolute_position = notif.position;
					taskEXIT_CRITICAL(&motor->lock);
				}
				else if (notif.type == STEPPER_UIM2852_ALARM_STALL)
				{
					taskENTER_CRITICAL(&motor->lock);
					motor->status.stall_detected = true;
					motor->motion_in_progress = false;
					taskEXIT_CRITICAL(&motor->lock);
				}
				else if (notif.type == STEPPER_UIM2852_STATUS_PVT_FIFO_EMPTY)
				{
					taskENTER_CRITICAL(&motor->lock);
					motor->pt_fifo_empty = true;
					motor->pt_motion_started = false;
					motor->motion_in_progress = false;
					motor->pt_prefill_count = 0;
					taskEXIT_CRITICAL(&motor->lock);
#ifdef CONFIG_LOG_ACTUATOR_STEPPER_RX
					ESP_LOGI(TAG, "Node %u: PVT FIFO empty", motor->config.node_id);
#endif
				}
				else if (notif.type == STEPPER_UIM2852_STATUS_PVT_FIFO_LOW)
				{
					taskENTER_CRITICAL(&motor->lock);
					motor->pt_fifo_low = true;
					taskEXIT_CRITICAL(&motor->lock);
#ifdef CONFIG_LOG_ACTUATOR_STEPPER_RX
					ESP_LOGI(TAG, "Node %u: PVT FIFO low", motor->config.node_id);
#endif
				}

				// Call per-motor notification callback if set
				if (motor->notify_callback)
					motor->notify_callback(motor, &notif);
			}
		}
		break;

	case STEPPER_UIM2852_CW_ER:
		// Error report — suppress spurious 0x33/0x34 errors caused by
		// motor misinterpreting Safety heartbeat frames as commands.
		{
			stepper_uim2852_error_t error = {};
			if (stepper_uim2852_parse_error(data, dl, &error))
			{
				if (error.error_code != STEPPER_UIM2852_ERR_SUBINDEX && error.error_code != STEPPER_UIM2852_ERR_DATA)
				{
					taskENTER_CRITICAL(&motor->lock);
					motor->status.error_detected = true;
					motor->last_error = error;
					taskEXIT_CRITICAL(&motor->lock);
					ESP_LOGE(TAG, "Node %u: Error 0x%02X on CW=0x%02X[%u]", motor->config.node_id, error.error_code,
					         error.related_cw, error.subindex);
				}
			}
		}
		break;

	case STEPPER_UIM2852_CW_MO:
		// ACK for motor enable/disable
		if (dl >= 1)
		{
			taskENTER_CRITICAL(&motor->lock);
			motor->driver_enabled = (data[0] != 0);
			taskEXIT_CRITICAL(&motor->lock);
		}
		break;
	case STEPPER_UIM2852_CW_PP:
	case STEPPER_UIM2852_CW_IC:
	case STEPPER_UIM2852_CW_IE:
	case STEPPER_UIM2852_CW_MT:
	case STEPPER_UIM2852_CW_LM:
	case STEPPER_UIM2852_CW_QE:
		// Parameter response — unblock query_param if this matches the pending query
		{
			taskENTER_CRITICAL(&motor->lock);
			bool match = (motor->query_pending_cw == cw_base && dl >= 2 && data[0] == motor->query_pending_idx);
			taskEXIT_CRITICAL(&motor->lock);
			if (match)
			{
				// Delegate to protocol layer for correct sign-extended parsing
				uint8_t parsed_idx;
				int32_t val = 0;
				stepper_uim2852_parse_param_response(data, dl, &parsed_idx, &val);
				motor->query_result = val;
				taskENTER_CRITICAL(&motor->lock);
				motor->query_pending_cw = 0;
				taskEXIT_CRITICAL(&motor->lock);
				xSemaphoreGive(motor->query_sem);
			}
		}
		break;

	default:
		// ACK for other commands
		break;
	}

	return true;
}

// ============================================================================
// Liveness Watchdog
// ============================================================================

bool stepper_motor_uim2852_check_liveness(const stepper_motor_uim2852_t *motor, TickType_t now_tick,
                                          TickType_t timeout_ticks)
{
	if (!motor || !motor->initialized)
		return false;

	stepper_motor_uim2852_t *m = (stepper_motor_uim2852_t *)motor;
	taskENTER_CRITICAL(&m->lock);
	bool driver_enabled = m->driver_enabled;
	TickType_t last_response_tick = m->last_response_tick;
	taskEXIT_CRITICAL(&m->lock);

	// Only flag timeout if the driver has been enabled (we expect responses)
	// and at least one response has been received (last_response_tick != 0).
	if (!driver_enabled)
		return false;
	if (last_response_tick == 0)
		return false;

	TickType_t elapsed = now_tick - last_response_tick;
	return (elapsed > timeout_ticks);
}

// ============================================================================
// Notification Callback
// ============================================================================

void stepper_motor_uim2852_set_notify_callback(stepper_motor_uim2852_t *motor,
                                               stepper_motor_uim2852_notify_cb_t callback)
{
	if (!motor)
		return;
	taskENTER_CRITICAL(&motor->lock);
	motor->notify_callback = callback;
	taskEXIT_CRITICAL(&motor->lock);
}

// ============================================================================
// Low-Level Access
// ============================================================================

esp_err_t stepper_motor_uim2852_send_instruction(stepper_motor_uim2852_t *motor, uint8_t cw, const uint8_t *data,
                                                 uint8_t dl)
{
	return send_instruction(motor, cw, data, dl);
}

esp_err_t stepper_motor_uim2852_query_param(stepper_motor_uim2852_t *motor, uint8_t cw, uint8_t index, int32_t *value)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	// Drain any stale signal from a previous query
	xSemaphoreTake(motor->query_sem, 0);

	// Record what we are waiting for so process_frame can match the response
	motor->query_result = 0;
	taskENTER_CRITICAL(&motor->lock);
	motor->query_pending_idx = index;
	motor->query_pending_cw = stepper_uim2852_cw_base(cw); // store base CW for matching
	taskEXIT_CRITICAL(&motor->lock);

	uint8_t data[8] = {};
	data[0] = index;

	esp_err_t err = send_instruction(motor, cw, data, 1);
	if (err != ESP_OK)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->query_pending_cw = 0;
		taskEXIT_CRITICAL(&motor->lock);
		return err;
	}

	// Block until process_frame signals the semaphore or we time out (500 ms)
	if (xSemaphoreTake(motor->query_sem, pdMS_TO_TICKS(500)) != pdTRUE)
	{
		taskENTER_CRITICAL(&motor->lock);
		motor->query_pending_cw = 0;
		taskEXIT_CRITICAL(&motor->lock);
		return ESP_ERR_TIMEOUT;
	}

	if (value)
		*value = motor->query_result;
	return ESP_OK;
}

esp_err_t stepper_motor_uim2852_set_param(stepper_motor_uim2852_t *motor, uint8_t cw, uint8_t index, int32_t value)
{
	if (!motor || !motor->initialized)
		return ESP_ERR_INVALID_STATE;

	uint8_t data[8] = {};
	uint8_t cw_base = stepper_uim2852_cw_base(cw);
	uint8_t dl;

	if (cw_base == STEPPER_UIM2852_CW_PP)
		dl = stepper_uim2852_build_pp_set(data, index, (uint8_t)(value & 0xFF));
	else if (cw_base == STEPPER_UIM2852_CW_LM)
		dl = stepper_uim2852_build_lm_set(data, index, value);
	else
		dl = stepper_uim2852_build_ic_set(data, index, (uint16_t)(value & 0xFFFF));

	return send_instruction(motor, cw, data, dl);
}
