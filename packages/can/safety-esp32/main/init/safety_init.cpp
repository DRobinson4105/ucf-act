/**
 * @file safety_init.cpp
 * @brief Safety ESP32 initialization sequence extracted from main.cpp.
 *
 * Configures all peripherals, registers heartbeat nodes, optionally
 * waits for initial heartbeats, logs startup status and active test
 * bypasses, then creates the CAN RX, Safety, and Heartbeat tasks
 * before deleting itself.
 */

#include "safety_init.h"

#include "nvs_flash.h"
#include "battery_monitor.h"
#include "can_twai.h"
#include "esp_log.h"
#include "esp_system.h"
#include "heartbeat_monitor.h"
#include "node_support.h"
#include "relay_srd05vdc.h"
#include "safety_can_rx.h"
#include "safety_config.h"
#include "safety_globals.h"
#include "safety_health.h"
#include "safety_local_inputs.h"
#include "safety_loop.h"
#include "safety_orin_link.h"
#include "safety_startup_log.h"
#include "status_led.h"
#include "ultrasonic_a02yyuw.h"
#include "usb_serial_link.h"

// ============================================================================
// Init Helpers
// ============================================================================

#if !defined(CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS) || !defined(CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS)
/**
 * @brief Block until a node's heartbeat is detected or a timeout expires.
 *
 * Polls the heartbeat monitor at the given interval until the
 * specified node reports alive, or the timeout elapses.
 *
 * @param node_id        Node handle from heartbeat_monitor_register()
 * @param timeout        Maximum wait time in ticks
 * @param poll_interval  Polling period in ticks
 * @return true if the node is alive before timeout, false on timeout
 */
static bool wait_for_heartbeat_alive(int node_id, TickType_t timeout, TickType_t poll_interval)
{
	TickType_t start_tick = xTaskGetTickCount();
	while (!heartbeat_monitor_is_alive(&g_hb_monitor, node_id) &&
	       (TickType_t)(xTaskGetTickCount() - start_tick) < timeout)
	{
		vTaskDelay(poll_interval);
	}
	return heartbeat_monitor_is_alive(&g_hb_monitor, node_id);
}
#endif

// ============================================================================
// Main Task
// ============================================================================

/**
 * @brief Initialization task that configures all peripherals and spawns runtime tasks.
 *
 * Initializes the heartbeat monitor, push button, RF remote,
 * ultrasonic sensor, power relay, TWAI, and WS2812 LED.  Registers
 * Planner and Control nodes in the heartbeat monitor, optionally
 * waits for their initial heartbeats, logs startup status and active
 * test bypasses, then creates the CAN RX, Safety, and Heartbeat
 * tasks before deleting itself.  Component failures at init are
 * non-fatal — retries happen in the safety loop.
 *
 * @param param  Unused (NULL)
 */
void main_task(void *param)
{
	(void)param;
	g_boot_start_ms = node_get_time_ms();

	// Initialize NVS (required for battery SOC persistence)
	esp_err_t nvs_err = nvs_flash_init();
	if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		nvs_flash_erase();
		nvs_err = nvs_flash_init();
	}
	if (nvs_err != ESP_OK)
		ESP_LOGE(TAG_INIT, "NVS init failed: %s", esp_err_to_name(nvs_err));

	// Initialize heartbeat monitor
	heartbeat_monitor_config_t hb_cfg = {.name = "SAFETY"};
	heartbeat_monitor_init(&g_hb_monitor, &hb_cfg);

	// Register nodes to monitor (Safety only cares about Planner and Control)
	g_node_planner = heartbeat_monitor_register(&g_hb_monitor, "Planner", HEARTBEAT_TIMEOUT_MS);
	g_node_control = heartbeat_monitor_register(&g_hb_monitor, "Control", HEARTBEAT_TIMEOUT_MS);

	esp_err_t err = usb_serial_link_init();
	g_orin_link_ready = (err == ESP_OK);
	if (!g_orin_link_ready)
		ESP_LOGE(TAG_INIT, "Orin UART link init failed: %s", esp_err_to_name(err));

	g_orin_rx_ctx = {
		.tag = TAG_RX,
		.monitor = &g_hb_monitor,
		.planner_node_handle = g_node_planner,
		.control_node_handle = g_node_control,
		.mirror_lock = &g_hb_mirror_lock,
		.mirror = &g_hb_mirror,
	};
	if (g_orin_link_ready &&
	    xTaskCreate(safety_orin_link_rx_task, "orin_rx", CAN_RX_TASK_STACK, &g_orin_rx_ctx, CAN_RX_TASK_PRIO,
	                nullptr) != pdPASS)
	{
		ESP_LOGE(TAG_INIT, "Failed to create Orin RX task, restarting");
		esp_restart();
	}

	// Push button e-stop (mxuteek HB2-ES544)
	//   - 22mm NC (normally-closed) red emergency stop push button
	//   - NC design is fail-safe: broken wire = e-stop triggered
	//   - When pressed/activated: switch opens, breaks circuit
	//   - Wired to GPIO with internal pull-up
	g_push_button_cfg = {
		.gpio = PUSH_BUTTON_HB2ES544_GPIO,
		.active_level = PUSH_BUTTON_HB2ES544_ACTIVE_LEVEL,
		.enable_pullup = true,
		.enable_pulldown = false,
		.name = "Push button",
	};

	err = estop_input_init(&g_push_button_cfg);
#ifdef CONFIG_BYPASS_INPUT_PUSH_BUTTON
	g_push_button_init_ok = true;
#else
	g_push_button_init_ok = (err == ESP_OK);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG_INIT, "Push button init failed: %s (will retry in background)", esp_err_to_name(err));
	}
#endif

	// RF remote e-stop (DieseRC EV1527 433MHz)
	//   - DieseRC 433MHz universal wireless remote control switch
	//   - Receiver: DC 12V 1CH RF relay module (EV1527 learning code)
	//   - Includes 2 transmitter remotes
	//   - Carried by safety operator during autonomous testing
	//   - Provides wireless "dead man's switch" capability
	//   - Range-limited to keep operator in visual contact with vehicle
	g_rf_remote_cfg = {
		.gpio = RF_REMOTE_EV1527_GPIO,
		.active_level = RF_REMOTE_EV1527_ACTIVE_LEVEL,
		.enable_pullup = true,
		.enable_pulldown = false,
		.name = "RF remote",
	};

	err = estop_input_init(&g_rf_remote_cfg);
#ifdef CONFIG_BYPASS_INPUT_RF_REMOTE
	g_rf_remote_init_ok = true;
#else
	g_rf_remote_init_ok = (err == ESP_OK);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG_INIT, "RF remote init failed: %s (will retry in background)", esp_err_to_name(err));
	}
#endif

	// Configure power relay (starts disabled = safe)
	g_relay_cfg = {
		.gpio = POWER_RELAY_GPIO,
	};

	err = relay_srd05vdc_init(&g_relay_cfg);
	g_relay_init_ok = (err == ESP_OK);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG_INIT, "Power relay init failed: %s (will retry in background)", esp_err_to_name(err));
		g_target_state = NODE_STATE_NOT_READY;
	}

	// Configure ultrasonic sensor (A02YYUW)
	g_ultrasonic_cfg = {
		.uart_num = ULTRASONIC_A02YYUW_UART,
		.tx_gpio = ULTRASONIC_A02YYUW_TX_GPIO,
		.rx_gpio = ULTRASONIC_A02YYUW_RX_GPIO,
		.baud_rate = ULTRASONIC_A02YYUW_BAUD_RATE,
	};
	safety_local_inputs_init_state(&g_local_inputs, &g_local_inputs_cfg);

	err = ultrasonic_a02yyuw_init(&g_ultrasonic_cfg);
#ifdef CONFIG_BYPASS_INPUT_ULTRASONIC
	g_ultrasonic_init_ok = true;
#else
	g_ultrasonic_init_ok = (err == ESP_OK);
	safety_local_inputs_reset_ultrasonic(&g_local_inputs, &g_local_inputs_cfg);
	g_ultrasonic_init_tick = xTaskGetTickCount();
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG_INIT, "Ultrasonic init failed: %s (will retry in background)", esp_err_to_name(err));
	}
#endif

	// Initialize battery monitor (voltage + current sensing)
	//   - Pack voltage: 220kOhm/10kOhm resistor divider -> GPIO 0 (ADC1_CH0)
	//   - Pack current: LEM HTFS-200-P (5V supply, 6.25mV/A) with
	//     6.8kOhm/10kOhm output divider -> GPIO 1 (ADC1_CH1)
	//   - Non-safety-critical: failure does NOT trigger e-stop
	g_battery_cfg = {
		.voltage_gpio = BATTERY_VOLTAGE_GPIO,
		.current_gpio = BATTERY_CURRENT_GPIO,
		.divider_ratio = 22.33f,          // 220kOhm / 10kOhm nominal 23, calibrated from multimeter
		.current_zero_mv = 2500,          // HTFS at 5V: VCC/2 = 2.5V at 0A
		.current_sens_uv = 6.25f,         // HTFS-200-P: 6.25mV/A = 6.25uV/mA
		.current_output_scale = 0.5952f,  // 10k/(6.8k+10k) output divider
		.capacity_mah = BATTERY_CAPACITY_MAH,
	};

#ifdef CONFIG_BYPASS_INPUT_BATTERY_MONITOR
	g_battery_monitor_init_ok = true;
#else
	err = battery_monitor_init(&g_battery_cfg);
	g_battery_monitor_init_ok = (err == ESP_OK);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG_INIT, "Battery monitor init failed: %s (will retry in background)", esp_err_to_name(err));
	}
#endif

	// Initialize TWAI for CAN communication unless explicitly bypassed.
#ifdef CONFIG_BYPASS_CAN_TWAI
	g_twai_ready = false;
#else
	err = can_twai_init_default(TWAI_TX_GPIO, TWAI_RX_GPIO);
	g_twai_ready = (err == ESP_OK);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG_INIT, "TWAI init failed: %s (will retry in background)", esp_err_to_name(err));
	}
#endif

	// Initialize heartbeat LED (WS2812 on GPIO8)
	err = status_led_init();
	bool heartbeat_ready = (err == ESP_OK);
	if (!heartbeat_ready)
	{
		ESP_LOGE(TAG_INIT, "HEARTBEAT_LED init failed: %s (non-critical, no retry)", esp_err_to_name(err));
	}
	else
	{
		status_led_set_state(g_target_state);
	}

	safety_log_startup_device_status(TAG_INIT, g_twai_ready, TWAI_TX_GPIO, TWAI_RX_GPIO,
	                                 g_push_button_cfg.gpio, g_push_button_cfg.active_level, g_push_button_init_ok,
	                                 g_rf_remote_cfg.gpio, g_rf_remote_cfg.active_level, g_rf_remote_init_ok,
	                                 &g_ultrasonic_cfg, g_ultrasonic_init_ok, &g_relay_cfg, g_relay_init_ok,
	                                 BATTERY_VOLTAGE_GPIO, BATTERY_CURRENT_GPIO, g_battery_monitor_init_ok,
	                                 heartbeat_ready, g_orin_link_ready);

	// Log all active test bypasses at boot so they are visible on the serial console.
	// Any active bypass is a WARNING — this firmware should not be deployed to the vehicle.
	{
		bool any_bypass = false;
#ifdef CONFIG_BYPASS_PLANNER_AUTONOMY_GATE
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: PLANNER_AUTONOMY_GATE (forcing request=true)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: PLANNER_LIVENESS_CHECKS (ignoring Planner timeout)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_PLANNER_STATE_MIRROR
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: PLANNER_STATE_MIRROR (simulating Planner)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: CONTROL_LIVENESS_CHECKS (ignoring Control timeout)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_CONTROL_STATE_MIRROR
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: CONTROL_STATE_MIRROR (simulating Control)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_INPUT_PUSH_BUTTON
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: INPUT_PUSH_BUTTON (e-stop button forced inactive)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_INPUT_RF_REMOTE
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: INPUT_RF_REMOTE (RF remote forced inactive)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_INPUT_ULTRASONIC
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: INPUT_ULTRASONIC (forced clear and healthy)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_INPUT_BATTERY_MONITOR
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: INPUT_BATTERY_MONITOR (forced 50%% SOC)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_CAN_TWAI
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: CAN_TWAI (CAN/TWAI disabled)");
		any_bypass = true;
#endif
		if (any_bypass)
		{
			ESP_LOGW(TAG_INIT, "*** TEST BYPASSES ACTIVE — DO NOT DEPLOY TO VEHICLE ***");
		}
	}

	// CAN RX task is no longer needed — Control heartbeats arrive via Orin UART.
	// Safety has no CAN devices (all sensors are GPIO/UART/ADC).

	// Wait for Planner/Control heartbeats (if not bypassed).
#if !defined(CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS) || !defined(CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS)
	static constexpr TickType_t HB_INIT_WAIT = pdMS_TO_TICKS(1000);
	static constexpr TickType_t HB_INIT_POLL = pdMS_TO_TICKS(50);
#endif

#if !defined(CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS)
	{
		if (wait_for_heartbeat_alive(g_node_planner, HB_INIT_WAIT, HB_INIT_POLL))
			ESP_LOGI(TAG_INIT, "PLANNER: OK: heartbeat detected");
		else
			ESP_LOGE(TAG_INIT, "PLANNER: FAILED: no heartbeat within 1000ms");
	}
#endif

#if !defined(CONFIG_BYPASS_CAN_TWAI) && !defined(CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS)
	{
		if (wait_for_heartbeat_alive(g_node_control, HB_INIT_WAIT, HB_INIT_POLL))
			ESP_LOGI(TAG_INIT, "CONTROL: OK: heartbeat detected");
		else
			ESP_LOGE(TAG_INIT, "CONTROL: FAILED: no heartbeat within 1000ms");
	}
#endif

	ESP_LOGI(TAG_INIT, "Target: INIT (dwell=%lums)", (unsigned long)INIT_DWELL_MS);

	// Start remaining tasks
	if (xTaskCreate(safety_task, "safety", SAFETY_TASK_STACK, nullptr, SAFETY_TASK_PRIO, nullptr) != pdPASS)
	{
		ESP_LOGE(TAG_INIT, "Failed to create safety task, restarting");
		esp_restart();
	}
	if (xTaskCreate(heartbeat_task, "heartbeat", HEARTBEAT_TASK_STACK, nullptr, HEARTBEAT_TASK_PRIO, nullptr) != pdPASS)
	{
		ESP_LOGE(TAG_INIT, "Failed to create heartbeat task, restarting");
		esp_restart();
	}

	// Main task complete - delete self
	vTaskDelete(nullptr);
}
