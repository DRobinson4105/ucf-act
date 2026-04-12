/**
 * @file control_init.cpp
 * @brief Control ESP32 initialization: peripheral bring-up, bypass logging, and runtime task creation.
 */

#include "control_init.h"

#include "adc_12bitsar.h"
#include "can_twai.h"
#include "control_can_rx.h"
#include "control_config.h"
#include "control_driver_inputs.h"
#include "control_globals.h"
#include "control_health.h"
#include "control_loop.h"
#include "control_orin_link.h"
#include "control_startup_log.h"
#include "control_test_mode.h"
#include "dac_mcp4728.h"
#include "esp_log.h"
#include "esp_system.h"
#include "heartbeat_monitor.h"
#include "motor_dispatch.h"
#include "motor_protocol.h"
#include "node_support.h"
#include "relay_dpdt_my5nj.h"
#include "status_led.h"
#include "usb_serial_link.h"

/**
 * @brief Initialization task that configures all peripherals and spawns runtime tasks.
 *
 * Initializes TWAI, LED, heartbeat monitor, and all actuator/sensor
 * components with one startup attempt each.  Starts the CAN RX task
 * first so motor init queries can receive motor responses.  Logs
 * startup status and active test bypasses, then creates the control
 * and heartbeat tasks before deleting itself.
 *
 * @param param  Unused (NULL)
 */
void main_task(void *param)
{
	(void)param;

	g_control_state = NODE_STATE_INIT;
	g_boot_start_ms = node_get_time_ms();

#ifdef CONFIG_CONTROL_TEST_MODE
	// ----------------------------------------------------------------
	// Control Test Mode: create the standalone selected-actuator test
	// task. Hardware bring-up is owned by control_test_mode.cpp so the
	// test runtime stays self-contained.
	// ----------------------------------------------------------------

	esp_err_t err = status_led_init();
	bool heartbeat_ready = (err == ESP_OK);
	if (heartbeat_ready)
		status_led_set_state(NODE_STATE_INIT);

	ESP_LOGI(TAG_INIT, "--- CONTROL TEST MODE ---");
#if defined(CONFIG_CONTROL_TEST_ACTUATOR_THROTTLE)
	ESP_LOGI(TAG_INIT, "Selected actuator: THROTTLE");
#elif defined(CONFIG_CONTROL_TEST_ACTUATOR_STEERING)
	ESP_LOGI(TAG_INIT, "Selected actuator: STEERING");
#else
	ESP_LOGI(TAG_INIT, "Selected actuator: BRAKING");
#endif
	ESP_LOGI(TAG_INIT, "HEARTBEAT_LED: %s", heartbeat_ready ? "OK" : "UNAVAILABLE");
	ESP_LOGI(TAG_INIT, "Skipped: Safety/Orin state machine and normal control runtime");

	if (xTaskCreate(control_test_mode_task, "control_test", CONTROL_TASK_STACK, &g_control_test_mode,
	                CONTROL_TASK_PRIO, nullptr) != pdPASS)
	{
		ESP_LOGE(TAG_INIT, "Failed to create control test task, restarting");
		esp_restart();
	}

	vTaskDelete(nullptr);

#else // !CONFIG_CONTROL_TEST_MODE

	// ----------------------------------------------------------------
	// Normal Mode: full system initialization
	// ----------------------------------------------------------------

	// Initialize TWAI once during startup. If it fails, mark FAULT and let
	// background per-component retries handle recovery.
	esp_err_t err = ESP_FAIL;
	err = can_twai_init_default(TWAI_TX_GPIO, TWAI_RX_GPIO);
	g_twai_ready = (err == ESP_OK);

	// Initialize motor dispatch layer (uses TWAI for CAN TX)
	if (g_twai_ready)
	{
		err = motor_dispatch_init(can_twai_send, 16);
		if (err != ESP_OK)
			ESP_LOGE(TAG_INIT, "motor_dispatch_init failed: %s", esp_err_to_name(err));
	}

	err = status_led_init();
	bool heartbeat_ready = (err == ESP_OK);
	if (heartbeat_ready)
	{
		status_led_set_state(NODE_STATE_INIT);
	}

	// Initialize heartbeat monitor to detect Safety and Planner heartbeat timeouts.
	heartbeat_monitor_config_t hb_cfg = {.name = "CONTROL"};
	heartbeat_monitor_init(&g_hb_monitor, &hb_cfg);
	g_node_safety = heartbeat_monitor_register(&g_hb_monitor, "Safety", HEARTBEAT_TIMEOUT_MS);
	g_node_planner = heartbeat_monitor_register(&g_hb_monitor, "Planner", HEARTBEAT_TIMEOUT_MS);

	err = usb_serial_link_init();
	g_orin_link_ready = (err == ESP_OK);
	if (!g_orin_link_ready)
		ESP_LOGE(TAG_INIT, "Orin UART link init failed: %s", esp_err_to_name(err));

	g_orin_rx_ctx = {
		.tag = TAG_RX,
		.cmd_lock = &g_cmd_lock,
		.snapshot = &g_cmd,
		.monitor = &g_hb_monitor,
		.planner_node_handle = g_node_planner,
	};
	if (g_orin_link_ready &&
	    xTaskCreate(control_orin_link_rx_task, "orin_rx", CAN_RX_TASK_STACK, &g_orin_rx_ctx, CAN_RX_TASK_PRIO,
	                nullptr) != pdPASS)
	{
		ESP_LOGE(TAG_INIT, "Failed to create Orin RX task, restarting");
		esp_restart();
	}

	// Start CAN RX first so motor init checks can receive responses.
	if (xTaskCreate(can_rx_task, "can_rx", CAN_RX_TASK_STACK, nullptr, CAN_RX_TASK_PRIO, &g_can_rx_task_handle) !=
	    pdPASS)
	{
		ESP_LOGE(TAG_INIT, "Failed to create CAN RX task, restarting");
		esp_restart();
	}

	// One startup attempt per component; retries happen later while faulted.
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	g_dac_ready = (dac_mcp4728_init(&g_dac_cfg) == ESP_OK);
	g_dpdt_relay_ready = (relay_dpdt_my5nj_init(&g_dpdt_relay_cfg) == ESP_OK);
#else
	g_dac_ready = true;
	g_dpdt_relay_ready = true;
#endif

#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
	g_driver_inputs.pedal_ready = (control_driver_inputs_init_pedal(&g_driver_inputs, &g_driver_inputs_cfg) == ESP_OK);
#else
	g_driver_inputs.pedal_ready = true;
#endif

#ifndef CONFIG_BYPASS_INPUT_FR_SENSOR
	g_driver_inputs.fr_ready = (control_driver_inputs_init_fr(&g_driver_inputs, &g_driver_inputs_cfg) == ESP_OK);
#else
	g_driver_inputs.fr_ready = true;
#endif

	// Motor Initialization Phase — simplified init: send MO=0 (disable) to each motor.
	// Full parameter setup (AC/DC/SD/LM) is deferred to the enable sequence.
	motor_dispatch_result_t motor_res;
	bool braking_init_ok = false;
	bool steering_init_ok = false;
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
	if (g_twai_ready)
	{
		motor_cmd_t cmd;
		// MO=0 hot-start safety disable
		motor_cmd_mo_set(MOTOR_NODE_BRAKING, true, MOTOR_MO_STATE_DISABLE, &cmd);
		motor_res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(200), pdMS_TO_TICKS(50), nullptr, nullptr, nullptr);
		braking_init_ok = (motor_res == MOTOR_DISPATCH_RESULT_OK);
		// Initialize local PT state
		g_braking_state.lock = portMUX_INITIALIZER_UNLOCKED;
		g_braking_state.pt_frame_time_ms = CONFIG_MOTOR_UIM2852_PT_FRAME_TIME_MS;
	}
#else
	braking_init_ok = true;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
	if (g_twai_ready)
	{
		motor_cmd_t cmd;
		motor_cmd_mo_set(MOTOR_NODE_STEERING, true, MOTOR_MO_STATE_DISABLE, &cmd);
		motor_res = motor_dispatch_exec(&cmd, pdMS_TO_TICKS(200), pdMS_TO_TICKS(50), nullptr, nullptr, nullptr);
		steering_init_ok = (motor_res == MOTOR_DISPATCH_RESULT_OK);
		// Initialize local PT state
		g_steering_state.lock = portMUX_INITIALIZER_UNLOCKED;
		g_steering_state.pt_frame_time_ms = CONFIG_MOTOR_UIM2852_PT_FRAME_TIME_MS;
	}
#else
	steering_init_ok = true;
#endif
	g_motor_uim2852_braking_ready = braking_init_ok;
	g_motor_uim2852_steering_ready = steering_init_ok;

	control_log_startup_device_status(TAG_INIT, g_twai_ready, TWAI_TX_GPIO, TWAI_RX_GPIO, &g_dac_cfg, g_dac_ready,
	                                  &g_dpdt_relay_cfg, g_dpdt_relay_ready, &g_pedal_adc_cfg,
	                                  (unsigned)PEDAL_ADC_THRESHOLD_MV, g_driver_inputs.pedal_mv,
	                                  g_driver_inputs.pedal_ready,
	                                  adc_12bitsar_is_calibrated(), g_fr_pc817_cfg.forward_gpio,
	                                  g_fr_pc817_cfg.reverse_gpio, g_driver_inputs.fr_ready,
	                                  (unsigned)MOTOR_NODE_STEERING,
	                                  g_motor_uim2852_steering_ready, (unsigned)MOTOR_NODE_BRAKING,
	                                  g_motor_uim2852_braking_ready, heartbeat_ready, g_orin_link_ready);

	// Log all active test bypasses at boot so they are visible on the serial console.
	// Any active bypass is a WARNING — this firmware should not be deployed to the vehicle.
	{
		bool any_bypass = false;
#ifdef CONFIG_BYPASS_SAFETY_TARGET_MIRROR
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: SAFETY_TARGET_MIRROR (forcing ACTIVE)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_SAFETY_ESTOP_MIRROR
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: SAFETY_ESTOP_MIRROR (forcing stop/fault clear)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_SAFETY_LIVENESS_CHECKS
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: SAFETY_LIVENESS_CHECKS (ignoring Safety timeout)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_PLANNER_COMMAND_INPUTS
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: PLANNER_COMMAND_INPUTS (forcing zero commands)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_PLANNER_COMMAND_STALE_CHECKS
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: PLANNER_COMMAND_STALE_CHECKS (no stale timeout)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_INPUT_PEDAL_ADC
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: INPUT_PEDAL_ADC (pedal forced released)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_INPUT_FR_SENSOR
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: INPUT_FR_SENSOR (forcing FORWARD)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_THROTTLE
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: ACTUATOR_THROTTLE (DAC/DPDT relay skipped)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_STEERING
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: ACTUATOR_MOTOR_STEERING (steering skipped)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_MOTOR_UIM2852_BRAKING
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: ACTUATOR_MOTOR_BRAKING (braking skipped)");
		any_bypass = true;
#endif
		if (any_bypass)
		{
			ESP_LOGW(TAG_INIT, "*** TEST BYPASSES ACTIVE — DO NOT DEPLOY TO VEHICLE ***");
		}
	}

	update_control_state_from_component_health();

	if (g_fault_flags != NODE_FAULT_NONE)
	{
		const char *detail = fault_detail_string(g_fault_flags, control_driver_inputs_fr_state(&g_driver_inputs), true);
		if (detail)
		{
			ESP_LOGE(TAG_INIT, "State: INIT -> %s (fault=%s detail=%s)", node_state_to_string(g_control_state),
			         node_fault_to_string(g_fault_flags), detail);
		}
		else
		{
			ESP_LOGE(TAG_INIT, "State: INIT -> %s (fault=%s)", node_state_to_string(g_control_state),
			         node_fault_to_string(g_fault_flags));
		}
	}
	else
	{
		ESP_LOGI(TAG_INIT, "State: INIT (dwell=%lums)", (unsigned long)INIT_DWELL_MS);
	}

	// Sync LED with post-health-check state so it shows red immediately if a
	// component failed init, rather than staying yellow until heartbeat_task starts.
	if (heartbeat_ready)
	{
		status_led_set_state(g_control_state);
	}

	if (xTaskCreate(control_task, "control", CONTROL_TASK_STACK, nullptr, CONTROL_TASK_PRIO, nullptr) != pdPASS)
	{
		ESP_LOGE(TAG_INIT, "Failed to create control task, restarting");
		esp_restart();
	}
	if (xTaskCreate(heartbeat_task, "heartbeat", HEARTBEAT_TASK_STACK, nullptr, HEARTBEAT_TASK_PRIO, nullptr) != pdPASS)
	{
		ESP_LOGE(TAG_INIT, "Failed to create heartbeat task, restarting");
		esp_restart();
	}

	vTaskDelete(nullptr);
#endif // CONFIG_CONTROL_TEST_MODE
}
