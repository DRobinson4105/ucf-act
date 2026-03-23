/**
 * @file main.cpp
 * @brief Safety ESP32 main application — e-stop monitoring and system state authority.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "can_protocol.h"
#include "can_twai.h"
#include "led_ws2812.h"
#include "heartbeat_monitor.h"
#include "relay_srd05vdc.h"
#include "ultrasonic_a02yyuw.h"
#include "battery_monitor.h"
#include "safety_logic.h"
#include "system_state.h"
#include "can_tx_track.h"
#include "node_utils.h"

// ============================================================================
// Production Build Guard
// ============================================================================
// If CONFIG_PRODUCTION_BUILD is set, all BYPASS_* flags must be disabled.
// This prevents accidental deployment of bench-test firmware onto the vehicle.
#ifdef CONFIG_PRODUCTION_BUILD
#ifdef CONFIG_BYPASS_PLANNER_AUTONOMY_GATE
#error "PRODUCTION_BUILD: CONFIG_BYPASS_PLANNER_AUTONOMY_GATE must be disabled"
#endif
#ifdef CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS
#error "PRODUCTION_BUILD: CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS must be disabled"
#endif
#ifdef CONFIG_BYPASS_PLANNER_STATE_MIRROR
#error "PRODUCTION_BUILD: CONFIG_BYPASS_PLANNER_STATE_MIRROR must be disabled"
#endif
#ifdef CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS
#error "PRODUCTION_BUILD: CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS must be disabled"
#endif
#ifdef CONFIG_BYPASS_CONTROL_STATE_MIRROR
#error "PRODUCTION_BUILD: CONFIG_BYPASS_CONTROL_STATE_MIRROR must be disabled"
#endif
#ifdef CONFIG_BYPASS_INPUT_PUSH_BUTTON
#error "PRODUCTION_BUILD: CONFIG_BYPASS_INPUT_PUSH_BUTTON must be disabled"
#endif
#ifdef CONFIG_BYPASS_INPUT_RF_REMOTE
#error "PRODUCTION_BUILD: CONFIG_BYPASS_INPUT_RF_REMOTE must be disabled"
#endif
#ifdef CONFIG_BYPASS_INPUT_ULTRASONIC
#error "PRODUCTION_BUILD: CONFIG_BYPASS_INPUT_ULTRASONIC must be disabled"
#endif
#ifdef CONFIG_BYPASS_INPUT_BATTERY_MONITOR
#error "PRODUCTION_BUILD: CONFIG_BYPASS_INPUT_BATTERY_MONITOR must be disabled"
#endif
#ifdef CONFIG_BYPASS_CAN_TWAI
#error "PRODUCTION_BUILD: CONFIG_BYPASS_CAN_TWAI must be disabled"
#endif
#endif // CONFIG_PRODUCTION_BUILD

// Safety ESP32 - System state authority and e-stop monitoring
//
// Monitors physical e-stop inputs (push button HB2-ES544, RF remote EV1527, ultrasonic A02YYUW)
// and node heartbeats (Planner, Control). When any e-stop condition is active, disables power
// relay and retreats the system target state to NOT_READY.
//
// Safety is the ONLY node that can advance state forward (NOT_READY -> READY -> ENABLE -> ACTIVE).
// It broadcasts a heartbeat (0x100) with target_state in the state field and the estop
// fault_code. All three nodes use the same heartbeat format (node_heartbeat_t).
//
// State advancement:
//   NOT_READY -> READY: both Planner and Control report READY, no e-stop,
//   READY -> ENABLE:    Planner/Orin autonomy request edge latched
//   ENABLE -> ACTIVE:   both report ENABLE + enable_complete flag set
//   ENABLE/ACTIVE -> READY: Planner/Orin autonomy request dropped (halt)
//   ANY -> NOT_READY:   e-stop, fault, override, timeout
//
// E-stop faults are OR'd into a bitmask — all active faults are reported simultaneously.
// Fault bits: button | remote | ultrasonic | planner | planner_timeout | control | control_timeout

namespace
{

typedef struct
{
	gpio_num_t gpio;
	int active_level;
	bool enable_pullup;
	bool enable_pulldown;
	const char *name;
} estop_input_config_t;

esp_err_t estop_input_init(const estop_input_config_t *config)
{
	if (!config)
		return ESP_ERR_INVALID_ARG;

	gpio_config_t io_conf = {
		.pin_bit_mask = 1ULL << config->gpio,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = config->enable_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
		.pull_down_en = config->enable_pulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	return gpio_config(&io_conf);
}

bool estop_input_is_active(const estop_input_config_t *config)
{
	if (!config)
		return true;
	return gpio_get_level(config->gpio) == config->active_level;
}

const char *TAG = "SAFETY";
const char *TAG_INIT = "SAFETY_INIT";
const char *TAG_TX __attribute__((unused)) = "SAFETY_TX";
const char *TAG_RX __attribute__((unused)) = "SAFETY_RX";

// ============================================================================
// Task Configuration
// ============================================================================

constexpr int CAN_RX_TASK_STACK = 8192;
constexpr int SAFETY_TASK_STACK = 8192;
constexpr int HEARTBEAT_TASK_STACK = 8192;
constexpr UBaseType_t CAN_RX_TASK_PRIO = 7;
constexpr UBaseType_t SAFETY_TASK_PRIO = 6;
constexpr UBaseType_t HEARTBEAT_TASK_PRIO = 4;

// ============================================================================
// Timing Constants
// ============================================================================

constexpr TickType_t CAN_RX_TIMEOUT = pdMS_TO_TICKS(10);
constexpr TickType_t SAFETY_LOOP_INTERVAL = pdMS_TO_TICKS(50);
constexpr TickType_t HEARTBEAT_SEND_INTERVAL = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS);
constexpr uint32_t INIT_DWELL_MS = 500;         // minimum dwell in INIT before READY
constexpr uint32_t ACTIVE_ENTRY_GRACE_MS = 250; // allow one-way handoff from ENABLE to ACTIVE

// ============================================================================
// Recovery Constants
// ============================================================================

constexpr uint8_t CAN_TX_FAIL_THRESHOLD = 5;
constexpr uint32_t RETRY_INTERVAL_MS = 500; // component retry cadence (infinite, no cap)

// Debounce: require N consecutive "clear" reads before declaring push-button
// or RF remote disengaged. Engage is always immediate (safety-critical).
// At 50ms loop cadence, 3 samples = 150ms disengage hold.
constexpr uint8_t ESTOP_DISENGAGE_COUNT = 3;

// Ultrasonic obstacle debounce: require N consecutive close/clear reads before
// toggling the latched obstacle state.
// At 50ms loop cadence, 2 samples = 100ms engage hold, 3 samples = 150ms clear hold.
constexpr uint8_t ULTRASONIC_ENGAGE_COUNT = 2;
constexpr uint8_t ULTRASONIC_DISENGAGE_COUNT = 3;

// Ultrasonic health hysteresis: require N consecutive agreeing health samples
// before changing health state. Prevents flap around timeout boundary.
// At 50ms loop cadence, 3 samples = 150ms hysteresis window.
constexpr uint8_t ULTRASONIC_HEALTH_HYSTERESIS = 3;

// Startup grace period for ultrasonic edge logging. Suppresses LOST/REGAINED
// transitions while the sensor's UART RX task establishes its first valid
// measurement. Matches the driver's SENSOR_HEALTH_TIMEOUT (500ms).
constexpr TickType_t ULTRASONIC_LOG_GRACE_TICKS = pdMS_TO_TICKS(500);

// ============================================================================
// GPIO Pin Assignments
// ============================================================================

// E-stop inputs
constexpr gpio_num_t PUSH_BUTTON_HB2ES544_GPIO = GPIO_NUM_6;
constexpr int PUSH_BUTTON_HB2ES544_ACTIVE_LEVEL = 1;

constexpr gpio_num_t RF_REMOTE_EV1527_GPIO = GPIO_NUM_7;
constexpr int RF_REMOTE_EV1527_ACTIVE_LEVEL = 1;

// Power relay output (cuts power to actuators when e-stop active)
constexpr gpio_num_t POWER_RELAY_GPIO = GPIO_NUM_2;

// CAN bus (WAVESHARE SN65HVD230 transceiver)
constexpr gpio_num_t TWAI_TX_GPIO = GPIO_NUM_4;
constexpr gpio_num_t TWAI_RX_GPIO = GPIO_NUM_5;

// Battery monitor ADC inputs
constexpr int BATTERY_VOLTAGE_GPIO = 0; // ADC1_CH0: pack voltage via 180k/10k divider
constexpr int BATTERY_CURRENT_GPIO = 1; // ADC1_CH1: HTFS-200-P via 10k/15k divider

// Ultrasonic sensor (A02YYUW)
constexpr uart_port_t ULTRASONIC_A02YYUW_UART = UART_NUM_1;
constexpr int ULTRASONIC_A02YYUW_TX_GPIO = GPIO_NUM_10; // Sensor RX (mode select)
constexpr int ULTRASONIC_A02YYUW_RX_GPIO = GPIO_NUM_11; // Sensor TX (data output)
constexpr int ULTRASONIC_A02YYUW_BAUD_RATE = 9600;
constexpr uint16_t ULTRASONIC_STOP_DISTANCE_MM = 1000;
constexpr uint16_t ULTRASONIC_CLEAR_DISTANCE_MM = 1200;

// ============================================================================
// Global State
// ============================================================================

// Heartbeat monitor (tracks Planner and Control timeouts)
heartbeat_monitor_t g_hb_monitor;
int g_node_planner = -1;
int g_node_control = -1;

// Node state tracking (set by CAN RX task, read by safety_task)
// Mirror fields are updated/read as grouped snapshots under g_hb_mirror_lock.
volatile node_state_t g_planner_state = NODE_STATE_INIT;
volatile node_fault_t g_planner_fault_code = NODE_FAULT_NONE;
volatile node_status_flags_t g_planner_status_flags = 0;
volatile node_stop_t g_planner_stop_flags = NODE_STOP_NONE;

volatile node_state_t g_control_state = NODE_STATE_INIT;
volatile node_fault_t g_control_fault_code = NODE_FAULT_NONE;
volatile node_status_flags_t g_control_status_flags = 0;
volatile node_stop_t g_control_stop_flags = NODE_STOP_NONE;

// Spinlock for grouped planner/control mirror snapshots.
portMUX_TYPE g_hb_mirror_lock = portMUX_INITIALIZER_UNLOCKED;

// System target state (Safety is the authority)
volatile node_state_t g_target_state = NODE_STATE_INIT;

// Safety heartbeat sequence counter
volatile node_seq_t g_safety_hb_seq = 0;
[[maybe_unused]] volatile bool g_hb_tx_failing = false; // edge-trigger for heartbeat TX failure logging

// Spinlock for heartbeat sequence (shared by safety_task immediate send + heartbeat_task)
portMUX_TYPE g_safety_hb_seq_lock = portMUX_INITIALIZER_UNLOCKED;

// Safety heartbeat cause channels.
volatile node_stop_t g_stop_flags = NODE_STOP_NONE;
volatile node_fault_t g_fault_code = NODE_FAULT_NONE;

// Component health tracking.
volatile bool g_push_button_init_ok = false;
volatile bool g_rf_remote_init_ok = false;
volatile bool g_ultrasonic_init_ok = false;
bool g_ultrasonic_health_prev = false;
bool g_ultrasonic_health_seen = false;
uint8_t g_ultrasonic_health_counter = 0; // hysteresis counter
#ifndef CONFIG_BYPASS_INPUT_ULTRASONIC
TickType_t g_ultrasonic_init_tick = 0; // for startup grace period
#endif
volatile bool g_relay_init_ok = false;
volatile bool g_battery_monitor_init_ok = false;
volatile bool g_twai_ready = false;

// Local hardware fault flag for LED indication. When true, heartbeat_task
// enables the LED red fault overlay regardless of g_target_state.
// Currently tracks relay availability — the relay is Safety's critical output
// component; when it is unavailable Safety cannot cut actuator power.
volatile bool g_safety_local_fault = false;

// CAN RX task handle — used to verify quiesce during recovery.
TaskHandle_t g_can_rx_task_handle = nullptr;

// Debounce counters for push-button and RF disengage (engage is immediate).
uint8_t g_push_button_clear_count = 0;
bool g_push_button_debounced = false; // debounced "active" state
uint8_t g_rf_remote_clear_count = 0;
bool g_rf_remote_debounced = false; // debounced "active" state

// Ultrasonic obstacle debounce/hysteresis state.
// Engage/clear require N consecutive reads to avoid one-sample chatter.
uint8_t g_ultrasonic_close_count = 0;
uint8_t g_ultrasonic_clear_count = 0;
bool g_ultrasonic_too_close_debounced = false;

// ============================================================================
// Component Configurations
// ============================================================================

estop_input_config_t g_push_button_cfg;
estop_input_config_t g_rf_remote_cfg;
relay_srd05vdc_config_t g_relay_cfg;
ultrasonic_a02yyuw_config_t g_ultrasonic_cfg;
battery_monitor_config_t g_battery_cfg;

// Retry pacing: last timestamp (ms) when retry_failed_components ran.
// Retries are infinite (no cap) but paced at RETRY_INTERVAL_MS to avoid
// hammering init calls on every safety loop tick.
uint32_t g_last_retry_ms = 0;
uint32_t g_boot_start_ms = 0;

// Time, WDT, and component health utilities are provided by node_utils.h.
// Local wrappers forward TAG so call sites stay concise.
/** @brief Return current time in milliseconds (forwarded from node_utils). */
uint32_t get_time_ms()
{
	return node_get_time_ms();
}

/** @brief Register the calling task with the ESP task watchdog (forwarded from node_utils). */
void task_wdt_add_self_or_log(const char *n)
{
	node_task_wdt_add_self_or_log(TAG, n);
}

/** @brief Reset the task watchdog for the calling task (forwarded from node_utils). */
void task_wdt_reset_or_log(const char *n, bool *f)
{
	node_task_wdt_reset_or_log(TAG, n, f);
}

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
bool wait_for_heartbeat_alive(int node_id, TickType_t timeout, TickType_t poll_interval)
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

/**
 * @brief Log the initialization status of all Safety ESP32 peripherals.
 *
 * Emits one INFO or ERROR line per component showing GPIO assignments,
 * configuration details, and pass/fail status.  Bypass-disabled
 * components are logged as BYPASSED at WARNING level.
 *
 * @param twai_ready       TWAI driver initialized successfully
 * @param relay_init_ok    Power relay initialized
 * @param heartbeat_ready  WS2812 status LED initialized
 */
void log_startup_device_status(bool twai_ready, bool relay_init_ok, bool heartbeat_ready)
{
#ifdef CONFIG_BYPASS_CAN_TWAI
	ESP_LOGW(TAG_INIT, "TWAI: BYPASSED: disabled");
#else
	if (twai_ready)
	{
		ESP_LOGI(TAG_INIT, "TWAI: CONFIGURED: tx_gpio=%d rx_gpio=%d", TWAI_TX_GPIO, TWAI_RX_GPIO);
	}
	else
	{
		ESP_LOGE(TAG_INIT, "TWAI: FAILED: tx_gpio=%d rx_gpio=%d", TWAI_TX_GPIO, TWAI_RX_GPIO);
	}
#endif

#ifdef CONFIG_BYPASS_INPUT_PUSH_BUTTON
	ESP_LOGW(TAG_INIT, "PUSH_BUTTON: BYPASSED: disabled (input_gpio=%d active_level=%s)", g_push_button_cfg.gpio,
	         g_push_button_cfg.active_level ? "HIGH" : "LOW");
#else
	if (g_push_button_init_ok)
		ESP_LOGI(TAG_INIT, "PUSH_BUTTON: CONFIGURED: input_gpio=%d active_level=%s", g_push_button_cfg.gpio,
		         g_push_button_cfg.active_level ? "HIGH" : "LOW");
	else
		ESP_LOGE(TAG_INIT, "PUSH_BUTTON: FAILED: input_gpio=%d", g_push_button_cfg.gpio);
#endif

#ifdef CONFIG_BYPASS_INPUT_RF_REMOTE
	ESP_LOGW(TAG_INIT, "RF_REMOTE: BYPASSED: disabled (input_gpio=%d active_level=%s)", g_rf_remote_cfg.gpio,
	         g_rf_remote_cfg.active_level ? "HIGH" : "LOW");
#else
	if (g_rf_remote_init_ok)
		ESP_LOGI(TAG_INIT, "RF_REMOTE: CONFIGURED: input_gpio=%d active_level=%s", g_rf_remote_cfg.gpio,
		         g_rf_remote_cfg.active_level ? "HIGH" : "LOW");
	else
		ESP_LOGE(TAG_INIT, "RF_REMOTE: FAILED: input_gpio=%d", g_rf_remote_cfg.gpio);
#endif

#ifdef CONFIG_BYPASS_INPUT_ULTRASONIC
	ESP_LOGW(TAG_INIT, "ULTRASONIC: BYPASSED: disabled (uart=%d tx_gpio=%d rx_gpio=%d)", g_ultrasonic_cfg.uart_num,
	         g_ultrasonic_cfg.tx_gpio, g_ultrasonic_cfg.rx_gpio);
#else
	if (g_ultrasonic_init_ok)
		ESP_LOGI(TAG_INIT, "ULTRASONIC: CONFIGURED: uart=%d tx_gpio=%d rx_gpio=%d baud=%d", g_ultrasonic_cfg.uart_num,
		         g_ultrasonic_cfg.tx_gpio, g_ultrasonic_cfg.rx_gpio, g_ultrasonic_cfg.baud_rate);
	else
		ESP_LOGE(TAG_INIT, "ULTRASONIC: FAILED: uart=%d", g_ultrasonic_cfg.uart_num);
#endif

	if (relay_init_ok)
		ESP_LOGI(TAG_INIT, "RELAY: CONFIGURED: gpio=%d active_level=HIGH start=DISABLED", g_relay_cfg.gpio);
	else
		ESP_LOGE(TAG_INIT, "RELAY: FAILED: gpio=%d", g_relay_cfg.gpio);

#ifdef CONFIG_BYPASS_INPUT_BATTERY_MONITOR
	ESP_LOGW(TAG_INIT, "BATTERY_MONITOR: BYPASSED: disabled (voltage_gpio=%d current_gpio=%d)", BATTERY_VOLTAGE_GPIO,
	         BATTERY_CURRENT_GPIO);
#else
	if (g_battery_monitor_init_ok)
		ESP_LOGI(TAG_INIT, "BATTERY_MONITOR: CONFIGURED: voltage_gpio=%d current_gpio=%d", BATTERY_VOLTAGE_GPIO,
		         BATTERY_CURRENT_GPIO);
	else
		ESP_LOGE(TAG_INIT, "BATTERY_MONITOR: FAILED: voltage_gpio=%d current_gpio=%d", BATTERY_VOLTAGE_GPIO,
		         BATTERY_CURRENT_GPIO);
#endif

#ifdef CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS
	ESP_LOGW(TAG_INIT, "PLANNER: BYPASSED: liveness/state/autonomy checks disabled");
#endif

#ifdef CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS
	ESP_LOGW(TAG_INIT, "CONTROL: BYPASSED: liveness/state checks disabled");
#endif

	if (heartbeat_ready)
		ESP_LOGI(TAG_INIT, "HEARTBEAT_LED: CONFIGURED");
	else
		ESP_LOGW(TAG_INIT, "HEARTBEAT_LED: UNAVAILABLE (non-critical)");
}

/** @brief Log that a component has been lost (forwarded from node_utils). */
void log_component_lost(const char *name, const char *detail)
{
	node_log_component_lost(TAG, name, detail);
}

/** @brief Log that a component has been recovered (forwarded from node_utils). */
void log_component_regained(const char *name)
{
	node_log_component_regained(TAG, name);
}

/** @brief Mark a component as lost and log the failure (forwarded from node_utils). */
void mark_component_lost(volatile bool *ready, const char *name, const char *detail)
{
	node_mark_component_lost(ready, TAG, name, detail);
}

/**
 * @brief Convert a Safety fault code to a logging string.
 *
 * @param fault_code  Fault code to convert
 * @return Static string label for logging
 */
const char *safety_fault_to_log_string(node_fault_t fault_code)
{
	return node_fault_to_string(fault_code);
}

// ============================================================================
// Recovery State
// ============================================================================

uint8_t g_can_tx_fail_count = 0;
volatile bool g_can_recovery_in_progress = false;
uint8_t g_can_tx_in_flight = 0;

// Spinlock for CAN TX tracking / recovery (used by safety_task + heartbeat_task)
portMUX_TYPE g_can_tx_lock = portMUX_INITIALIZER_UNLOCKED;

/**
 * @brief Wait for CAN RX and heartbeat tasks to finish in-flight TWAI calls.
 *
 * Must be called after setting g_twai_ready = false, before touching
 * the TWAI driver for recovery.  Forwarded from node_utils.
 */
void quiesce_can_rx()
{
	node_quiesce_can_rx(&g_can_tx_lock, &g_can_tx_in_flight);
}

// Rate-limit retry failure logs: log on 1st attempt, then every RETRY_LOG_EVERY_N
// attempts. At RETRY_INTERVAL_MS = 500ms, N=20 gives ~10 second intervals.
const uint16_t RETRY_LOG_EVERY_N = 20;

/**
 * @brief Re-initialize any failed components at a rate-limited cadence.
 *
 * Called every safety loop tick, but paced at RETRY_INTERVAL_MS to
 * avoid hammering init calls.  Retries are infinite with no cap.
 * Covers push button, RF remote, relay, ultrasonic, and TWAI.
 * TWAI recovery uses a full teardown/reinit cycle with an active
 * probe heartbeat to verify bus connectivity.
 */
void retry_failed_components(void)
{
	// Pace retries at RETRY_INTERVAL_MS to avoid hammering init calls every
	// safety loop tick. Retries remain infinite (no cap).
	uint32_t now_ms = get_time_ms();
	if ((now_ms - g_last_retry_ms) < RETRY_INTERVAL_MS)
		return;
	g_last_retry_ms = now_ms;

#ifndef CONFIG_BYPASS_INPUT_PUSH_BUTTON
	if (!g_push_button_init_ok)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = estop_input_init(&g_push_button_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_PUSH_BUTTON
		if (!recovered)
		{
			s_retry_count++;
			if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0)
			{
				ESP_LOGW(TAG, "PUSH_BUTTON retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			g_push_button_init_ok = true;
			log_component_regained("PUSH_BUTTON");
		}
	}
#endif

#ifndef CONFIG_BYPASS_INPUT_RF_REMOTE
	if (!g_rf_remote_init_ok)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = estop_input_init(&g_rf_remote_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_RF_REMOTE
		if (!recovered)
		{
			s_retry_count++;
			if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0)
			{
				ESP_LOGW(TAG, "RF_REMOTE retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			g_rf_remote_init_ok = true;
			log_component_regained("RF_REMOTE");
		}
	}
#endif

	if (!g_relay_init_ok)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = relay_srd05vdc_init(&g_relay_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_POWER_RELAY
		if (!recovered)
		{
			s_retry_count++;
			if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0)
			{
				ESP_LOGW(TAG, "RELAY retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			g_relay_init_ok = true;
			log_component_regained("RELAY (driver-level)");
		}
	}

#ifndef CONFIG_BYPASS_INPUT_ULTRASONIC
	if (!g_ultrasonic_init_ok)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = ultrasonic_a02yyuw_init(&g_ultrasonic_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_ULTRASONIC
		if (!recovered)
		{
			s_retry_count++;
			if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0)
			{
				ESP_LOGW(TAG, "ULTRASONIC retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			g_ultrasonic_init_ok = true;
			g_ultrasonic_health_prev = false;
			g_ultrasonic_health_seen = false;
			g_ultrasonic_init_tick = xTaskGetTickCount();
		}
	}
#endif

#ifndef CONFIG_BYPASS_INPUT_BATTERY_MONITOR
	if (!g_battery_monitor_init_ok)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = battery_monitor_init(&g_battery_cfg);
		bool recovered = (err == ESP_OK);
		if (!recovered)
		{
			s_retry_count++;
			if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0)
			{
				ESP_LOGW(TAG, "BATTERY_MONITOR retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
			}
		}
		if (recovered)
		{
			s_retry_count = 0;
			g_battery_monitor_init_ok = true;
			log_component_regained("BATTERY_MONITOR");
		}
	}
#endif

#ifndef CONFIG_BYPASS_CAN_TWAI
	if (!g_twai_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		// Skip if another task is already performing CAN recovery.
		taskENTER_CRITICAL(&g_can_tx_lock);
		bool skip = g_can_recovery_in_progress;
		if (!skip)
			g_can_recovery_in_progress = true;
		taskEXIT_CRITICAL(&g_can_tx_lock);
		if (skip)
			return;

		// g_twai_ready is already false; quiesce before touching driver.
		quiesce_can_rx();

		// Suppress internal can_twai_recover() logs unless
		// CONFIG_LOG_CAN_RECOVERY is enabled; then rate-limit to 1st + every Nth.
#ifdef CONFIG_LOG_CAN_RECOVERY
		const char *verbose_tag =
			(s_retry_count == 0 || ((s_retry_count + 1) % RETRY_LOG_EVERY_N) == 0) ? TAG : nullptr;
#else
		const char *verbose_tag = nullptr;
#endif
		esp_err_t err = can_twai_recover(TWAI_TX_GPIO, TWAI_RX_GPIO, verbose_tag);
		bool recovered = false;
		if (err == ESP_OK)
		{
			// Active probe: send one heartbeat frame to verify bus is truly connected.
			// After reinit the driver starts RUNNING with zero error counters, so a
			// passive bus_ok() check always passes. Sending a real frame forces TEC
			// accumulation on a disconnected bus (TEC += ~8 per failed TX).
			uint8_t probe_data[8] = {};
			node_heartbeat_t probe_hb = {
				.sequence = 0,
				.state = g_target_state,
				.fault_code = g_fault_code,
				.status_flags = 0,
				.stop_flags = g_stop_flags,
			};
			can_encode_heartbeat(probe_data, &probe_hb);
			(void)can_twai_send(CAN_ID_SAFETY_HEARTBEAT, probe_data, pdMS_TO_TICKS(50));
			vTaskDelay(pdMS_TO_TICKS(50));
			twai_status_info_t probe_status;
			recovered = (twai_get_status_info(&probe_status) == ESP_OK && probe_status.state == TWAI_STATE_RUNNING &&
			             probe_status.tx_error_counter == 0);
		}
#ifdef CONFIG_LOG_RETRY_TWAI
		if (!recovered)
		{
			s_retry_count++;
			if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0)
			{
				if (err != ESP_OK)
				{
					ESP_LOGW(TAG, "TWAI retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
				}
				else
				{
					ESP_LOGW(TAG, "TWAI retry failed (%u attempts): bus unstable after reinit", s_retry_count);
				}
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			g_twai_ready = true;
		}

		taskENTER_CRITICAL(&g_can_tx_lock);
		g_can_recovery_in_progress = false;
		taskEXIT_CRITICAL(&g_can_tx_lock);
	}
#endif
}

/**
 * @brief Track CAN TX results and trigger bus recovery on consecutive failures.
 *
 * Increments a consecutive-failure counter on TX errors and resets it
 * on success.  When the counter reaches CAN_TX_FAIL_THRESHOLD, checks
 * bus health via can_twai_bus_ok().  If the bus is unhealthy, marks
 * g_twai_ready = false to initiate recovery in retry_failed_components(),
 * unless CONFIG_BYPASS_CAN_TWAI is enabled.
 *
 * Bus health checks and logging are performed outside critical sections
 * because can_twai_bus_ok() and ESP_LOG use locks/semaphores that
 * cannot be acquired with interrupts disabled on single-core ESP32-C6.
 *
 * @param err  Result of the CAN transmit operation
 */
void track_can_tx(esp_err_t err)
{
	bool trigger_recovery = false;
	bool check_bus = false;

	taskENTER_CRITICAL(&g_can_tx_lock);
	can_tx_track_inputs_t t = {
		.fail_count = g_can_tx_fail_count,
		.threshold = CAN_TX_FAIL_THRESHOLD,
		.tx_ok = (err == ESP_OK),
	};
	can_tx_track_result_t r = can_tx_track(&t);
	g_can_tx_fail_count = r.new_fail_count;
	if (r.trigger_recovery)
	{
		g_can_tx_fail_count = 0; // reset regardless
		check_bus = true;
	}
	taskEXIT_CRITICAL(&g_can_tx_lock);

	// Bus check and logging happen outside the critical section so that
	// twai_get_status_info() and ESP_LOG (which use locks/semaphores)
	// are called with interrupts enabled.
	if (check_bus)
	{
		bool bus_ok = can_twai_bus_ok();
		if (!bus_ok)
		{
#ifdef CONFIG_BYPASS_CAN_TWAI
			return;
#else
			taskENTER_CRITICAL(&g_can_tx_lock);
			if (!g_can_recovery_in_progress)
			{
				g_can_recovery_in_progress = true;
				trigger_recovery = true;
			}
			taskEXIT_CRITICAL(&g_can_tx_lock);
#endif
		}
		// else: bus OK but TX still failing — no partner on bus, nothing to do.
	}

	if (trigger_recovery)
	{
#ifdef CONFIG_LOG_CAN_RECOVERY
		ESP_LOGE(TAG, "CAN bus unhealthy after %d TX failures", CAN_TX_FAIL_THRESHOLD);
#endif
		taskENTER_CRITICAL(&g_can_tx_lock);
		g_twai_ready = false;
		// Recovery handled by retry_failed_components() at 500ms pace.
		g_can_recovery_in_progress = false;
		taskEXIT_CRITICAL(&g_can_tx_lock);
	}
}

// ============================================================================
// Safety Heartbeat Send Helper
// ============================================================================

/**
 * @brief Encode and transmit the Safety heartbeat frame on CAN.
 *
 * Atomically snapshots the current target state, e-stop fault code,
 * and sequence number, encodes them into the CAN heartbeat format,
 * and sends via TWAI.  Called from safety_task on state change
 * (immediate) and from heartbeat_task on the 100ms periodic timer.
 * Guards against concurrent recovery by checking g_twai_ready and
 * tracking in-flight TX count under spinlock.
 *
 * @param log_as_change  true for state-change-triggered sends (logged as "change"),
 *                       false for periodic sends (logged as "periodic")
 */
void send_safety_heartbeat(bool log_as_change)
{
	// Reserve an in-flight TX slot only when TWAI is ready and recovery is idle.
	// This closes the check/send race with recovery teardown.
	taskENTER_CRITICAL(&g_can_tx_lock);
	if (!g_twai_ready || g_can_recovery_in_progress || g_can_tx_in_flight == UINT8_MAX)
	{
		taskEXIT_CRITICAL(&g_can_tx_lock);
		return;
	}
	g_can_tx_in_flight++;
	taskEXIT_CRITICAL(&g_can_tx_lock);

#ifdef CONFIG_LOG_CAN_RECOVERY
	// Heap integrity diagnostic — detect corruption early (before ESP_LOG triggers abort).
	// heap_caps_check_integrity_all returns true if heap is intact.
	if (!heap_caps_check_integrity_all(true))
	{
		ESP_LOGE(TAG, "HEAP CORRUPTION detected in send_safety_heartbeat! free=%lu",
		         (unsigned long)esp_get_free_heap_size());
	}
#endif

	uint8_t data[8] = {};
	node_seq_t seq = 0;
	node_state_t target_state = NODE_STATE_NOT_READY;
	node_fault_t fault_code = NODE_FAULT_NONE;
	node_stop_t stop_flags = NODE_STOP_NONE;
	taskENTER_CRITICAL(&g_safety_hb_seq_lock);
	seq = g_safety_hb_seq;
	g_safety_hb_seq = (node_seq_t)(seq + 1);
	target_state = g_target_state;
	fault_code = g_fault_code;
	stop_flags = g_stop_flags;
	taskEXIT_CRITICAL(&g_safety_hb_seq_lock);
	node_heartbeat_t hb = {
		.sequence = seq,
		.state = target_state,
		.fault_code = fault_code,
		.status_flags = 0,
		.stop_flags = stop_flags,
	};
	can_encode_heartbeat(data, &hb);
	esp_err_t err = can_twai_send(CAN_ID_SAFETY_HEARTBEAT, data, pdMS_TO_TICKS(10));

	taskENTER_CRITICAL(&g_can_tx_lock);
	if (g_can_tx_in_flight > 0)
		g_can_tx_in_flight--;
	taskEXIT_CRITICAL(&g_can_tx_lock);

	track_can_tx(err);

	(void)log_as_change; // only used when CONFIG_LOG_CAN_HEARTBEAT_TX is enabled
#ifdef CONFIG_LOG_CAN_HEARTBEAT_TX
	if (err != ESP_OK)
	{
		if (!g_hb_tx_failing)
		{
			ESP_LOGE(TAG_TX, "Heartbeat TX failing: %s", esp_err_to_name(err));
			g_hb_tx_failing = true;
		}
	}
	else
	{
		if (g_hb_tx_failing)
		{
			ESP_LOGI(TAG_TX, "Heartbeat TX recovered");
			g_hb_tx_failing = false;
		}
		ESP_LOGI(TAG_TX, "HB TX: target=%s fault=%s stop=%s trigger=%s", node_state_to_string(target_state),
		         safety_fault_to_log_string(fault_code), node_stop_to_string(stop_flags),
		         log_as_change ? "change" : "periodic");
	}
#endif
}

// ============================================================================
// CAN RX Task
// ============================================================================

/**
 * @brief Decode a heartbeat CAN frame and update monitor + mirrored state.
 *
 * Parses the heartbeat payload, feeds the heartbeat monitor with the
 * new sequence number, and atomically mirrors the state, fault code,
 * status_flags and stop_flags into the caller-supplied volatile globals under
 * g_hb_mirror_lock.
 *
 * @param msg          Received CAN message
 * @param node_handle  Node handle from heartbeat_monitor_register()
 * @param state_out    [out] Volatile pointer to store the node's reported state
 * @param fault_out    [out] Volatile pointer to store the node's fault code
 * @param status_flags_out [out] Volatile pointer to store the node's heartbeat status flags
 * @param stop_out     [out] Volatile pointer to store node stop flags
 * @param label        Node name for diagnostic logging (e.g. "Planner")
 * @return true if the heartbeat was valid, false on decode failure
 */
bool process_heartbeat_rx(const twai_message_t &msg, int node_handle, volatile node_state_t *state_out,
                          volatile node_fault_t *fault_out, volatile node_status_flags_t *status_flags_out,
                          volatile node_stop_t *stop_out, [[maybe_unused]] const char *label)
{
	node_heartbeat_t hb;
	if (!can_decode_heartbeat(msg.data, msg.data_length_code, &hb))
	{
		return false;
	}
	heartbeat_monitor_update(&g_hb_monitor, node_handle, hb.sequence, hb.state);

	taskENTER_CRITICAL(&g_hb_mirror_lock);
	*state_out = hb.state;
	*fault_out = hb.fault_code;
	*status_flags_out = hb.status_flags;
	*stop_out = hb.stop_flags;
	taskEXIT_CRITICAL(&g_hb_mirror_lock);

#ifdef CONFIG_LOG_CAN_HEARTBEAT_RX
	ESP_LOGI(TAG_RX, "%s HB RX: seq=%u state=%s fault=%s stop=%s status_flags=0x%02X", label, hb.sequence,
	         node_state_to_string(hb.state), node_fault_to_string(hb.fault_code), node_stop_to_string(hb.stop_flags),
	         hb.status_flags);
#endif
	return true;
}

/**
 * @brief FreeRTOS task that receives CAN heartbeats from Planner and Control.
 *
 * Runs at highest priority (7).  Processes Planner heartbeats (0x110)
 * and Control heartbeats (0x120), updating the heartbeat monitor
 * timestamps and mirroring state/fault/status_flags/stop_flags into shared globals.
 * Sleeps 100ms when TWAI is down to avoid starving lower-priority tasks.
 *
 * @param param  Unused (NULL)
 */
void can_rx_task(void *param)
{
	(void)param;
	task_wdt_add_self_or_log("can_rx_task");
	bool wdt_reset_failed = false;
	twai_message_t msg{};
	while (true)
	{
		task_wdt_reset_or_log("can_rx_task", &wdt_reset_failed);
		// Skip TWAI API calls while driver is down or being recovered.
		// Sleeping 100ms (not 5ms) prevents this priority-7 task from
		// starving lower-priority tasks when the bus is unavailable.
		if (!g_twai_ready)
		{
			vTaskDelay(pdMS_TO_TICKS(100));
			continue;
		}
		if (can_twai_receive(&msg, CAN_RX_TIMEOUT) != ESP_OK)
		{
			vTaskDelay(pdMS_TO_TICKS(5));
			continue;
		}
		if (msg.extd || msg.rtr)
			continue;

		// Process Planner heartbeat (0x110)
		if (msg.identifier == CAN_ID_PLANNER_HEARTBEAT)
		{
			if (!process_heartbeat_rx(msg, g_node_planner, &g_planner_state, &g_planner_fault_code,
			                          &g_planner_status_flags, &g_planner_stop_flags, "Planner"))
				continue;
		}

		// Process Control ESP32 heartbeat (0x120)
		else if (msg.identifier == CAN_ID_CONTROL_HEARTBEAT)
		{
			if (!process_heartbeat_rx(msg, g_node_control, &g_control_state, &g_control_fault_code,
			                          &g_control_status_flags, &g_control_stop_flags, "Control"))
				continue;
		}
	}
}

// ============================================================================
// Safety Task
// ============================================================================

/**
 * @brief FreeRTOS task that runs the safety monitoring loop at 20 Hz.
 *
 * Each tick: retries failed components, reads e-stop inputs (push
 * button, RF remote, ultrasonic) with debounce, checks heartbeat
 * timeouts for Planner and Control, evaluates all safety conditions
 * via the pure safety_evaluate() function, manages the power relay,
 * runs the system state machine via system_state_step(), and sends
 * an immediate heartbeat on state change.  Runs at priority 6.
 *
 * Decision logic (e-stop bitmask evaluation, relay) is delegated to
 * the pure safety_logic module.  State advancement is delegated to
 * the pure system_state module.  Both are unit-testable on host.
 *
 * @param param  Unused (NULL)
 */
void safety_task(void *param)
{
	(void)param;
	task_wdt_add_self_or_log("safety_task");
	bool wdt_reset_failed = false;
	// One-shot autonomy request gate:
	// - latch on Planner request rising edge
	// - consume when transitioning READY -> ENABLE
	// - re-arm only after request drops
	bool request_level_prev = false;
	bool request_latched = false;
	uint32_t active_target_entered_ms = 0;
#ifdef CONFIG_LOG_SAFETY_FAULT_CHANGES
	node_fault_t prev_fault_code = NODE_FAULT_NONE;
	node_stop_t prev_stop_flags = NODE_STOP_NONE;
#endif
#ifdef CONFIG_LOG_ACTUATOR_POWER_RELAY_STATE
	bool prev_relay_enabled = false;
#endif

	while (true)
	{
		retry_failed_components();
		uint32_t now_ms = get_time_ms();

		// Read hardware inputs into the pure-function input struct.
		// Push-button and RF remote use disengage debounce: engage is
		// immediate (safety-critical), disengage requires ESTOP_DISENGAGE_COUNT
		// consecutive clear reads to filter contact bounce / RF glitches.
		bool raw_push_button = g_push_button_init_ok ? estop_input_is_active(&g_push_button_cfg) : true;
		bool raw_rf_remote = g_rf_remote_init_ok ? estop_input_is_active(&g_rf_remote_cfg) : true;

#ifdef CONFIG_LOG_INPUT_PUSH_BUTTON
		{
			static bool s_prev = false;
			static bool s_first = true;
			if (raw_push_button != s_prev || s_first)
			{
				ESP_LOGI(TAG, "Push button %s", raw_push_button ? "PRESSED" : "released");
				s_prev = raw_push_button;
				s_first = false;
			}
		}
#endif
#ifdef CONFIG_LOG_INPUT_RF_REMOTE
		{
			static bool s_prev = false;
			static bool s_first = true;
			if (raw_rf_remote != s_prev || s_first)
			{
				ESP_LOGI(TAG, "RF remote %s", raw_rf_remote ? "ENGAGED" : "disengaged");
				s_prev = raw_rf_remote;
				s_first = false;
			}
		}
#endif

		// Push-button debounce
		if (raw_push_button)
		{
			g_push_button_debounced = true;
			g_push_button_clear_count = 0;
		}
		else
		{
			if (g_push_button_clear_count < ESTOP_DISENGAGE_COUNT)
				g_push_button_clear_count++;
			if (g_push_button_clear_count >= ESTOP_DISENGAGE_COUNT)
				g_push_button_debounced = false;
		}

		// RF remote debounce
		if (raw_rf_remote)
		{
			g_rf_remote_debounced = true;
			g_rf_remote_clear_count = 0;
		}
		else
		{
			if (g_rf_remote_clear_count < ESTOP_DISENGAGE_COUNT)
				g_rf_remote_clear_count++;
			if (g_rf_remote_clear_count >= ESTOP_DISENGAGE_COUNT)
				g_rf_remote_debounced = false;
		}

		// Ultrasonic health with hysteresis: require ULTRASONIC_HEALTH_HYSTERESIS
		// consecutive agreeing samples before changing the reported health state.
		bool raw_ultrasonic_healthy = g_ultrasonic_init_ok ? ultrasonic_a02yyuw_is_healthy() : false;
		bool filtered_ultrasonic_healthy;
		if (g_ultrasonic_health_seen)
		{
			if (raw_ultrasonic_healthy != g_ultrasonic_health_prev)
			{
				g_ultrasonic_health_counter++;
				if (g_ultrasonic_health_counter >= ULTRASONIC_HEALTH_HYSTERESIS)
				{
					g_ultrasonic_health_prev = raw_ultrasonic_healthy;
					g_ultrasonic_health_counter = 0;
				}
			}
			else
			{
				g_ultrasonic_health_counter = 0;
			}
			filtered_ultrasonic_healthy = g_ultrasonic_health_prev;
		}
		else
		{
			// First sample: accept immediately without logging regained
			g_ultrasonic_health_prev = raw_ultrasonic_healthy;
			g_ultrasonic_health_seen = true;
			g_ultrasonic_health_counter = 0;
			filtered_ultrasonic_healthy = raw_ultrasonic_healthy;
		}

		// Ultrasonic obstacle debounce + hysteresis:
		// - Engage after ULTRASONIC_ENGAGE_COUNT consecutive close reads.
		// - While latched, use a larger clear threshold to avoid chatter near edge.
		// - Require ULTRASONIC_DISENGAGE_COUNT consecutive clear reads to disengage.
		bool raw_ultrasonic_too_close = false;
		if (g_ultrasonic_init_ok)
		{
			uint16_t threshold_mm =
				g_ultrasonic_too_close_debounced ? ULTRASONIC_CLEAR_DISTANCE_MM : ULTRASONIC_STOP_DISTANCE_MM;
			raw_ultrasonic_too_close = ultrasonic_a02yyuw_is_too_close(threshold_mm, NULL);
		}

		if (raw_ultrasonic_too_close)
		{
			if (g_ultrasonic_close_count < ULTRASONIC_ENGAGE_COUNT)
				g_ultrasonic_close_count++;
			if (g_ultrasonic_close_count >= ULTRASONIC_ENGAGE_COUNT)
				g_ultrasonic_too_close_debounced = true;
			g_ultrasonic_clear_count = 0;
		}
		else
		{
			g_ultrasonic_close_count = 0;
			if (g_ultrasonic_clear_count < ULTRASONIC_DISENGAGE_COUNT)
				g_ultrasonic_clear_count++;
			if (g_ultrasonic_clear_count >= ULTRASONIC_DISENGAGE_COUNT)
				g_ultrasonic_too_close_debounced = false;
		}

		safety_inputs_t inputs = {
			.push_button_active = g_push_button_debounced,
			.rf_remote_active = g_rf_remote_debounced,
			.ultrasonic_too_close = g_ultrasonic_too_close_debounced,
			.ultrasonic_healthy = filtered_ultrasonic_healthy,
			.planner_alive = true,  // updated below from heartbeat monitor
			.control_alive = true,  // updated below from heartbeat monitor
			.planner_issue = false, // updated from grouped snapshot below
			.control_issue = false, // updated from grouped snapshot below
			.planner_stop = 0,      // updated from grouped snapshot below
			.control_stop = 0,      // updated from grouped snapshot below
		};

		node_state_t planner_state = NODE_STATE_INIT;
		[[maybe_unused]] node_fault_t planner_fault_code = NODE_FAULT_NONE;
		node_status_flags_t planner_status_flags = 0;
		node_state_t control_state = NODE_STATE_INIT;
		[[maybe_unused]] node_fault_t control_fault_code = NODE_FAULT_NONE;
		node_status_flags_t control_status_flags = 0;
		node_stop_t planner_stop_flags = NODE_STOP_NONE;
		node_stop_t control_stop_flags = NODE_STOP_NONE;
		taskENTER_CRITICAL(&g_hb_mirror_lock);
		planner_state = g_planner_state;
		planner_fault_code = g_planner_fault_code;
		planner_status_flags = g_planner_status_flags;
		planner_stop_flags = g_planner_stop_flags;
		control_state = g_control_state;
		control_fault_code = g_control_fault_code;
		control_status_flags = g_control_status_flags;
		control_stop_flags = g_control_stop_flags;
		taskEXIT_CRITICAL(&g_hb_mirror_lock);
		inputs.planner_issue = node_fault_is_planner_issue(planner_fault_code);
		inputs.control_issue = node_fault_is_control_issue(control_fault_code);
		inputs.planner_stop = planner_stop_flags;
		inputs.control_stop = control_stop_flags;

		// Check heartbeat timeouts
		heartbeat_monitor_check_timeouts(&g_hb_monitor);
		inputs.planner_alive = heartbeat_monitor_is_alive(&g_hb_monitor, g_node_planner);
		inputs.control_alive = heartbeat_monitor_is_alive(&g_hb_monitor, g_node_control);

		// Apply test bypasses
#ifdef CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS
		inputs.planner_alive = true;
		inputs.planner_issue = false;
		inputs.planner_stop = NODE_STOP_NONE;
#endif
#ifdef CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS
		inputs.control_alive = true;
		inputs.control_issue = false;
		inputs.control_stop = NODE_STOP_NONE;
#endif
#ifdef CONFIG_BYPASS_INPUT_PUSH_BUTTON
		inputs.push_button_active = false;
#endif
#ifdef CONFIG_BYPASS_INPUT_RF_REMOTE
		inputs.rf_remote_active = false;
#endif
#ifdef CONFIG_BYPASS_INPUT_ULTRASONIC
		inputs.ultrasonic_too_close = false;
		inputs.ultrasonic_healthy = true;
#endif

		// Ultrasonic health edge logging uses the hysteresis-filtered value.
		// Transition tracking is maintained by the hysteresis block above;
		// we only log here when the filtered state actually changes.
		//
		// A startup grace period (ULTRASONIC_LOG_GRACE_TICKS) suppresses
		// LOST/REGAINED logging while the sensor's UART RX task establishes
		// its first valid measurement. Without this, the sensor always
		// appears briefly unhealthy at boot (no data yet) then immediately
		// recovers, producing a spurious LOST -> REGAINED pair.
#ifndef CONFIG_BYPASS_INPUT_ULTRASONIC
		{
			static bool s_ultrasonic_log_prev = false;
			static bool s_ultrasonic_log_seen = false;
			if (g_ultrasonic_init_ok)
			{
				TickType_t since_init = xTaskGetTickCount() - g_ultrasonic_init_tick;
				if (since_init < ULTRASONIC_LOG_GRACE_TICKS)
				{
					// Grace period: don't log, but track state so the
					// baseline is correct when the window expires.
					s_ultrasonic_log_prev = filtered_ultrasonic_healthy;
					s_ultrasonic_log_seen = true;
				}
				else if (!s_ultrasonic_log_seen)
				{
					// First sample after grace: accept as baseline, no log.
					s_ultrasonic_log_prev = filtered_ultrasonic_healthy;
					s_ultrasonic_log_seen = true;
				}
				else if (filtered_ultrasonic_healthy != s_ultrasonic_log_prev)
				{
					if (filtered_ultrasonic_healthy)
					{
						log_component_regained("ULTRASONIC");
					}
					else
					{
						log_component_lost("ULTRASONIC", "runtime health timeout");
					}
					s_ultrasonic_log_prev = filtered_ultrasonic_healthy;
				}
			}
		}
#endif

#ifdef CONFIG_LOG_SAFETY_ESTOP_INPUTS
		ESP_LOGI(TAG, "button=%d remote=%d ultra_close=%d ultra_healthy=%d planner=%s(%s) control=%s(%s)",
		         inputs.push_button_active, inputs.rf_remote_active, inputs.ultrasonic_too_close,
		         inputs.ultrasonic_healthy, inputs.planner_alive ? "alive" : "DEAD",
		         node_state_to_string(planner_state), inputs.control_alive ? "alive" : "DEAD",
		         node_state_to_string(control_state));
#endif

		// Update battery monitor (non-safety-critical, informational only)
#ifndef CONFIG_BYPASS_INPUT_BATTERY_MONITOR
		if (g_battery_monitor_init_ok)
		{
			battery_monitor_update(now_ms);
		}
#endif

		// Evaluate all safety conditions (pure function — no side effects)
		safety_decision_t decision = safety_evaluate(&inputs);

		// Missing critical output/transport components force fail-safe behavior.
		// Relay unavailable = system cannot cut power = force relay fault.
		if (!g_relay_init_ok)
		{
			decision.stop_active = true;
			decision.fault_code |= NODE_FAULT_SAFETY_RELAY_UNAVAILABLE;
			decision.relay_enable = false;
		}

		// Snapshot state values used by heartbeat send path.
		// Keep these updates atomic with respect to send_safety_heartbeat().
		node_state_t current_target = g_target_state;

		// Run system state machine (pure function)
		system_state_inputs_t ss_in = {
			.current_target = current_target,
			.now_ms = now_ms,
			.boot_start_ms = g_boot_start_ms,
			.init_dwell_ms = INIT_DWELL_MS,
			.stop_active = decision.stop_active,
			.planner_state = planner_state,
			.control_state = control_state,
			.planner_alive = inputs.planner_alive,
			.control_alive = inputs.control_alive,
			.planner_enable_complete = (planner_status_flags & NODE_STATUS_FLAG_ENABLE_COMPLETE) != 0,
			.control_enable_complete = (control_status_flags & NODE_STATUS_FLAG_ENABLE_COMPLETE) != 0,
			.autonomy_request = false,
			.autonomy_hold = false,
			.active_entry_grace =
				(current_target == NODE_STATE_ACTIVE && (now_ms - active_target_entered_ms) < ACTIVE_ENTRY_GRACE_MS),
		};

		// Planner bypass: simulate a cooperative Planner for bench bring-up.
		// Bootstrap behavior:
		// - When Safety target is NOT_READY/READY, force Planner READY so
		//   Safety can advance NOT_READY -> READY without a real Planner.
		// - When Safety target is ENABLE/ACTIVE, force Planner ENABLE and
		//   enable_complete so Safety can continue ENABLE -> ACTIVE.
#ifdef CONFIG_BYPASS_PLANNER_STATE_MIRROR
		if (g_target_state == NODE_STATE_ACTIVE || g_target_state == NODE_STATE_ENABLE)
			ss_in.planner_state = NODE_STATE_ENABLE;
		else
			ss_in.planner_state = NODE_STATE_READY;
		ss_in.planner_enable_complete = true;
#endif
		// Control bypass: same bootstrap-aware mirroring for Control.
#ifdef CONFIG_BYPASS_CONTROL_STATE_MIRROR
		if (g_target_state == NODE_STATE_ACTIVE || g_target_state == NODE_STATE_ENABLE)
			ss_in.control_state = NODE_STATE_ENABLE;
		else
			ss_in.control_state = NODE_STATE_READY;
		ss_in.control_enable_complete = true;
#endif

		bool planner_request_level = (planner_status_flags & NODE_STATUS_FLAG_AUTONOMY_REQUEST) != 0;
		bool request_accept_window =
			(ss_in.current_target == NODE_STATE_READY && !ss_in.stop_active && ss_in.planner_alive &&
		     ss_in.control_alive && ss_in.planner_state == NODE_STATE_READY && ss_in.control_state == NODE_STATE_READY);
#ifdef CONFIG_BYPASS_PLANNER_AUTONOMY_GATE
		planner_request_level = true;
#endif

		if (!planner_request_level)
		{
#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
			if (request_level_prev)
				ESP_LOGI(TAG, "Autonomy request re-armed (request dropped)");
#endif
			request_level_prev = false;
		}
		else if (!request_level_prev)
		{
			request_level_prev = true;
			if (request_accept_window)
			{
				request_latched = true;
#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
				ESP_LOGI(TAG, "Autonomy request latched");
#endif
			}
		}

		ss_in.autonomy_request = request_latched;
		ss_in.autonomy_hold = planner_request_level;
#ifdef CONFIG_BYPASS_PLANNER_AUTONOMY_GATE
		ss_in.autonomy_request = true;
		ss_in.autonomy_hold = true;
#endif

#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
		static bool prev_waiting_for_request = false;
		bool waiting_for_request =
			(ss_in.current_target == NODE_STATE_READY && !ss_in.stop_active && ss_in.planner_alive &&
		     ss_in.control_alive && ss_in.planner_state == NODE_STATE_READY &&
		     ss_in.control_state == NODE_STATE_READY && !ss_in.autonomy_request);
		if (waiting_for_request && !prev_waiting_for_request)
		{
			ESP_LOGI(TAG, "Waiting for autonomy request");
		}
		else if (!waiting_for_request && prev_waiting_for_request)
		{
			ESP_LOGI(TAG, "Autonomy request wait cleared");
		}
		prev_waiting_for_request = waiting_for_request;
#endif

		system_state_result_t ss_out = system_state_step(&ss_in);

		if (ss_out.new_target == NODE_STATE_ACTIVE)
		{
			if (current_target != NODE_STATE_ACTIVE)
				active_target_entered_ms = now_ms;
		}
		else
		{
			active_target_entered_ms = 0;
		}

		// Relay power is allowed only when e-stop is clear and target is
		// in the autonomy power window (ENABLE/ACTIVE).
		bool relay_should_enable =
			decision.relay_enable && (ss_out.new_target == NODE_STATE_ENABLE || ss_out.new_target == NODE_STATE_ACTIVE);

#ifndef CONFIG_BYPASS_PLANNER_AUTONOMY_GATE
		if (ss_out.target_changed && current_target == NODE_STATE_READY && ss_out.new_target == NODE_STATE_ENABLE &&
		    request_latched)
		{
			request_latched = false;
#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
			ESP_LOGI(TAG, "Autonomy request consumed");
#endif
		}
#endif

#ifdef CONFIG_LOG_SAFETY_STATE_TICK
		ESP_LOGI(TAG, "tick stop_active=%d planner=%s control=%s req=%d target=%s -> %s", decision.stop_active,
		         node_state_to_string(ss_in.planner_state), node_state_to_string(ss_in.control_state),
		         ss_in.autonomy_request, node_state_to_string(current_target), node_state_to_string(ss_out.new_target));
#endif

#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
		if (ss_out.target_changed)
		{
			const char *reason = "none";
			if (current_target == NODE_STATE_READY && ss_out.new_target == NODE_STATE_ENABLE)
				reason = "autonomy_request";
			else if (current_target == NODE_STATE_ENABLE && ss_out.new_target == NODE_STATE_ACTIVE)
				reason = "enable_complete";
			else if (ss_out.new_target == NODE_STATE_READY || ss_out.new_target == NODE_STATE_NOT_READY)
			{
				if ((current_target == NODE_STATE_ENABLE || current_target == NODE_STATE_ACTIVE) &&
				    !ss_in.autonomy_hold && decision.fault_code == NODE_FAULT_NONE &&
				    decision.stop_flags == NODE_STOP_NONE)
					reason = "autonomy_halt";
				else if (ss_out.new_target == NODE_STATE_NOT_READY && decision.fault_code == NODE_FAULT_NONE &&
				         decision.stop_flags == NODE_STOP_NONE)
					reason = "not_ready";
				else if (decision.stop_flags != NODE_STOP_NONE)
					reason = node_stop_to_string(decision.stop_flags);
				else
					reason = safety_fault_to_log_string(decision.fault_code);
			}

			ESP_LOGI(TAG, "Target: %s -> %s (reason=%s)", node_state_to_string(current_target),
			         node_state_to_string(ss_out.new_target), reason);

			// Log node detail when retreat is caused by a specific node
			if ((ss_out.new_target == NODE_STATE_READY || ss_out.new_target == NODE_STATE_NOT_READY) &&
			    (decision.fault_code != NODE_FAULT_NONE || decision.stop_flags != NODE_STOP_NONE))
			{
				if (decision.fault_code & NODE_FAULT_SAFETY_CONTROL_ISSUE)
					ESP_LOGI(TAG, "  Control issue: %s", node_fault_to_string(control_fault_code));
				if (decision.fault_code & NODE_FAULT_SAFETY_PLANNER_ISSUE)
					ESP_LOGI(TAG, "  Planner issue: %s", node_fault_to_string(planner_fault_code));
				if ((decision.stop_flags & NODE_STOP_OPERATOR_ANY) != 0)
					ESP_LOGI(TAG, "  Control stop: %s", node_stop_to_string(control_stop_flags));
				if (decision.stop_flags & NODE_STOP_APP_REQUEST)
					ESP_LOGI(TAG, "  Planner stop: %s", node_stop_to_string(planner_stop_flags));
			}
		}
#endif

		// Log stop/fault code changes (edge-triggered)
#ifdef CONFIG_LOG_SAFETY_FAULT_CHANGES
		if (decision.fault_code != prev_fault_code)
		{
			ESP_LOGI(TAG, "Safety fault: %s -> %s", safety_fault_to_log_string(prev_fault_code),
			         safety_fault_to_log_string(decision.fault_code));
		}
		if (decision.stop_flags != prev_stop_flags)
		{
			ESP_LOGI(TAG, "Safety stop: %s -> %s", node_stop_to_string(prev_stop_flags),
			         node_stop_to_string(decision.stop_flags));
		}
		prev_fault_code = decision.fault_code;
		prev_stop_flags = decision.stop_flags;
#endif

		// Publish new target + cause channels atomically for heartbeat snapshots.
		taskENTER_CRITICAL(&g_safety_hb_seq_lock);
		g_fault_code = decision.fault_code;
		g_stop_flags = decision.stop_flags;
		g_target_state = ss_out.new_target;
		taskEXIT_CRITICAL(&g_safety_hb_seq_lock);

		// Control power relay based on safety decision
		if (g_relay_init_ok)
		{
			if (relay_should_enable)
			{
				esp_err_t relay_err = relay_srd05vdc_enable(&g_relay_cfg);
				if (relay_err != ESP_OK)
				{
					ESP_LOGE(TAG, "relay_srd05vdc_enable FAILED: %s — relay may be stuck OFF",
					         esp_err_to_name(relay_err));
					mark_component_lost(&g_relay_init_ok, "RELAY", "runtime enable command failed");
					// Immediately force e-stop in this iteration — don't wait
					// for the next loop cycle to detect g_relay_init_ok == false.
					taskENTER_CRITICAL(&g_safety_hb_seq_lock);
					g_fault_code = NODE_FAULT_SAFETY_RELAY_UNAVAILABLE;
					g_stop_flags = NODE_STOP_NONE;
					g_target_state = NODE_STATE_NOT_READY;
					taskEXIT_CRITICAL(&g_safety_hb_seq_lock);
					send_safety_heartbeat(true);
				}
#ifdef CONFIG_LOG_ACTUATOR_POWER_RELAY_STATE
				else if (!prev_relay_enabled)
					ESP_LOGI(TAG, "Power relay ENABLED");
#endif
			}
			else
			{
				esp_err_t relay_err = relay_srd05vdc_disable(&g_relay_cfg);
				if (relay_err != ESP_OK)
				{
					ESP_LOGE(TAG, "relay_srd05vdc_disable FAILED: %s — relay may be stuck ON!",
					         esp_err_to_name(relay_err));
					mark_component_lost(&g_relay_init_ok, "RELAY", "runtime disable command failed");
					// Immediately force e-stop — relay state is indeterminate.
					taskENTER_CRITICAL(&g_safety_hb_seq_lock);
					g_fault_code = NODE_FAULT_SAFETY_RELAY_UNAVAILABLE;
					g_stop_flags = NODE_STOP_NONE;
					g_target_state = NODE_STATE_NOT_READY;
					taskEXIT_CRITICAL(&g_safety_hb_seq_lock);
					send_safety_heartbeat(true);
				}
#ifdef CONFIG_LOG_ACTUATOR_POWER_RELAY_STATE
				else if (prev_relay_enabled)
					ESP_LOGI(TAG, "Power relay DISABLED");
#endif
			}
#ifdef CONFIG_LOG_ACTUATOR_POWER_RELAY_STATE
			prev_relay_enabled = relay_should_enable;
#endif
		}

		// Track local hardware fault for LED indication (red on fault).
		g_safety_local_fault = !g_relay_init_ok;

		// Send immediate heartbeat on target state change
		if (ss_out.target_changed)
		{
			send_safety_heartbeat(true);
		}

		task_wdt_reset_or_log("safety_task", &wdt_reset_failed);
		vTaskDelay(SAFETY_LOOP_INTERVAL);
	}
}

// ============================================================================
// Heartbeat Task
// ============================================================================

/**
 * @brief FreeRTOS task that sends periodic heartbeats and updates the LED.
 *
 * Runs at priority 4 (lowest of the three tasks).  Every
 * HEARTBEAT_SEND_INTERVAL (100ms) updates the WS2812 status LED
 * color to reflect the current target state (with red fault overlay
 * when local hardware fault is active) and transmits the Safety heartbeat
 * CAN frame.
 *
 * @param param  Unused (NULL)
 */
void heartbeat_task(void *param)
{
	(void)param;
	task_wdt_add_self_or_log("heartbeat_task");
	bool wdt_reset_failed = false;

	while (true)
	{
		task_wdt_reset_or_log("heartbeat_task", &wdt_reset_failed);

		led_ws2812_set_fault_overlay(g_safety_local_fault);
		led_ws2812_set_state(g_target_state);

		// Send Safety heartbeat (periodic)
		send_safety_heartbeat(false);

		// Send battery status at 1 Hz (every 10th heartbeat tick = 10 × 100ms)
		{
			static uint8_t battery_tx_divider = 0;
			if (++battery_tx_divider >= 10)
			{
				battery_tx_divider = 0;

				battery_status_t bat = {};
#ifdef CONFIG_BYPASS_INPUT_BATTERY_MONITOR
				bat.voltage_mv = 48000;
				bat.current_10ma = 0;
				bat.soc_pct = 50;
				bat.flags = 0;
#else
				if (g_battery_monitor_init_ok)
				{
					battery_monitor_get_status(&bat);
				}
				else
				{
					bat.flags = BATTERY_FLAG_SENSOR_FAULT;
				}
#endif

#ifndef CONFIG_BYPASS_CAN_TWAI
				uint8_t bat_data[8] = {};
				can_encode_battery_status(bat_data, &bat);

				taskENTER_CRITICAL(&g_can_tx_lock);
				bool can_send = g_twai_ready && !g_can_recovery_in_progress && g_can_tx_in_flight < UINT8_MAX;
				if (can_send)
					g_can_tx_in_flight++;
				taskEXIT_CRITICAL(&g_can_tx_lock);

				if (can_send)
				{
					esp_err_t err = can_twai_send(CAN_ID_SAFETY_BATTERY_STATUS, bat_data, pdMS_TO_TICKS(10));

					taskENTER_CRITICAL(&g_can_tx_lock);
					if (g_can_tx_in_flight > 0)
						g_can_tx_in_flight--;
					taskEXIT_CRITICAL(&g_can_tx_lock);

					track_can_tx(err);
				}
#endif
			}
		}

		vTaskDelay(HEARTBEAT_SEND_INTERVAL);
	}
}

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
	g_boot_start_ms = get_time_ms();

	// Initialize heartbeat monitor
	heartbeat_monitor_config_t hb_cfg = {.name = "SAFETY"};
	heartbeat_monitor_init(&g_hb_monitor, &hb_cfg);

	// Register nodes to monitor (Safety only cares about Planner and Control)
	g_node_planner = heartbeat_monitor_register(&g_hb_monitor, "Planner", HEARTBEAT_TIMEOUT_MS);
	g_node_control = heartbeat_monitor_register(&g_hb_monitor, "Control", HEARTBEAT_TIMEOUT_MS);

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

	esp_err_t err = estop_input_init(&g_push_button_cfg);
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
	}

	// Configure ultrasonic sensor (A02YYUW)
	g_ultrasonic_cfg = {
		.uart_num = ULTRASONIC_A02YYUW_UART,
		.tx_gpio = ULTRASONIC_A02YYUW_TX_GPIO,
		.rx_gpio = ULTRASONIC_A02YYUW_RX_GPIO,
		.baud_rate = ULTRASONIC_A02YYUW_BAUD_RATE,
	};

	err = ultrasonic_a02yyuw_init(&g_ultrasonic_cfg);
#ifdef CONFIG_BYPASS_INPUT_ULTRASONIC
	g_ultrasonic_init_ok = true;
#else
	g_ultrasonic_init_ok = (err == ESP_OK);
	g_ultrasonic_health_prev = false;
	g_ultrasonic_health_seen = false;
	g_ultrasonic_init_tick = xTaskGetTickCount();
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG_INIT, "Ultrasonic init failed: %s (will retry in background)", esp_err_to_name(err));
	}
#endif

	// Initialize battery monitor (voltage + current sensing)
	//   - Pack voltage: 180kΩ/10kΩ resistor divider → GPIO 0 (ADC1_CH0)
	//   - Pack current: LEM HTFS-200-P (5V supply, 6.25mV/A) with
	//     10kΩ/15kΩ output divider → GPIO 1 (ADC1_CH1)
	//   - Non-safety-critical: failure does NOT trigger e-stop
	g_battery_cfg = {
		.voltage_gpio = BATTERY_VOLTAGE_GPIO,
		.current_gpio = BATTERY_CURRENT_GPIO,
		.divider_ratio = 19,          // 180kΩ / 10kΩ → ratio 1:19
		.current_zero_mv = 2500,      // HTFS at 5V: VCC/2 = 2.5V at 0A
		.current_sens_uv = 6.25f,     // HTFS-200-P: 6.25mV/A = 6.25µV/mA
		.current_output_scale = 0.6f, // 15k/(10k+15k) output divider
		.capacity_mah = 150000,       // 150Ah nominal (adjust for your batteries)
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
	err = led_ws2812_init();
	bool heartbeat_ready = (err == ESP_OK);
	if (!heartbeat_ready)
	{
		ESP_LOGE(TAG_INIT, "HEARTBEAT_LED init failed: %s (non-critical, no retry)", esp_err_to_name(err));
	}
	else
	{
		led_ws2812_set_state(NODE_STATE_INIT);
	}

	log_startup_device_status(g_twai_ready, g_relay_init_ok, heartbeat_ready);

	// Reflect local hardware fault on LED immediately so it shows red during
	// the heartbeat-wait window (before heartbeat_task starts polling).
	g_safety_local_fault = !g_relay_init_ok;
	if (heartbeat_ready && g_safety_local_fault)
	{
		led_ws2812_set_fault_overlay(true);
	}

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

	// Start CAN RX task first so heartbeat frames can be received during init wait.
#ifndef CONFIG_BYPASS_CAN_TWAI
	if (xTaskCreate(can_rx_task, "can_rx", CAN_RX_TASK_STACK, nullptr, CAN_RX_TASK_PRIO, &g_can_rx_task_handle) !=
	    pdPASS)
	{
		ESP_LOGE(TAG_INIT, "Failed to create CAN RX task, restarting");
		esp_restart();
	}
#endif

	// Wait for Planner/Control heartbeats (if not bypassed and CAN/TWAI enabled)
#if !defined(CONFIG_BYPASS_CAN_TWAI) && \
	(!defined(CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS) || !defined(CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS))
	static constexpr TickType_t HB_INIT_WAIT = pdMS_TO_TICKS(1000);
	static constexpr TickType_t HB_INIT_POLL = pdMS_TO_TICKS(50);
#endif

#if !defined(CONFIG_BYPASS_CAN_TWAI) && !defined(CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS)
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
} // namespace

/**
 * @brief ESP-IDF application entry point for the Safety ESP32.
 *
 * Creates the main_task on a dedicated stack to perform peripheral
 * initialization and spawn runtime tasks.  Restarts the system if
 * task creation fails.
 */
extern "C" void app_main(void)
{
	if (xTaskCreate(main_task, "main_task", 8192, nullptr, 5, nullptr) != pdPASS)
	{
		ESP_LOGE("SAFETY_INIT", "Failed to create main_task, restarting");
		esp_restart();
	}
}
