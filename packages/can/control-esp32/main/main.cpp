/**
 * @file main.cpp
 * @brief Control ESP32 main application — autonomous actuator control via CAN commands.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "esp_heap_caps.h"
#include "esp_task_wdt.h"

#ifdef CONFIG_THROTTLE_TEST_MODE
#include "serial_input.h"
#endif

#include "can_protocol.h"
#include "can_twai.h"
#include "led_ws2812.h"
#include "digipot_mcp41hv51.h"
#include "stepper_motor_uim2852.h"
#include "relay_dpdt_my5nj.h"
#include "adc_12bitsar.h"
#include "optocoupler_pc817.h"
#include "control_logic.h"
#include "heartbeat_monitor.h"
#include "node_utils.h"

// ============================================================================
// Production Build Guard
// ============================================================================
// If CONFIG_PRODUCTION_BUILD is set, all BYPASS_* flags must be disabled.
// This prevents accidental deployment of bench-test firmware onto the vehicle.
#ifdef CONFIG_PRODUCTION_BUILD
#ifdef CONFIG_BYPASS_SAFETY_TARGET_MIRROR
#error "PRODUCTION_BUILD: CONFIG_BYPASS_SAFETY_TARGET_MIRROR must be disabled"
#endif
#ifdef CONFIG_BYPASS_SAFETY_ESTOP_MIRROR
#error "PRODUCTION_BUILD: CONFIG_BYPASS_SAFETY_ESTOP_MIRROR must be disabled"
#endif
#ifdef CONFIG_BYPASS_SAFETY_LIVENESS_CHECKS
#error "PRODUCTION_BUILD: CONFIG_BYPASS_SAFETY_LIVENESS_CHECKS must be disabled"
#endif
#ifdef CONFIG_BYPASS_PLANNER_COMMAND_INPUTS
#error "PRODUCTION_BUILD: CONFIG_BYPASS_PLANNER_COMMAND_INPUTS must be disabled"
#endif
#ifdef CONFIG_BYPASS_PLANNER_COMMAND_STALE_CHECKS
#error "PRODUCTION_BUILD: CONFIG_BYPASS_PLANNER_COMMAND_STALE_CHECKS must be disabled"
#endif
#ifdef CONFIG_BYPASS_INPUT_PEDAL_ADC
#error "PRODUCTION_BUILD: CONFIG_BYPASS_INPUT_PEDAL_ADC must be disabled"
#endif
#ifdef CONFIG_BYPASS_INPUT_FR_SENSOR
#error "PRODUCTION_BUILD: CONFIG_BYPASS_INPUT_FR_SENSOR must be disabled"
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_THROTTLE
#error "PRODUCTION_BUILD: CONFIG_BYPASS_ACTUATOR_THROTTLE must be disabled"
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
#error "PRODUCTION_BUILD: CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING must be disabled"
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
#error "PRODUCTION_BUILD: CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING must be disabled"
#endif
#ifdef CONFIG_THROTTLE_TEST_MODE
#error "PRODUCTION_BUILD: CONFIG_THROTTLE_TEST_MODE must be disabled"
#endif
#endif // CONFIG_PRODUCTION_BUILD

// Control ESP32 - Autonomous control execution
//
// Receives commands from Planner via CAN and actuates throttle, steering, and braking.
// Safety ESP32 commands target state via heartbeat (0x100).
// State behavior and transitions are documented in control-esp32/README.md.

namespace
{

const char *TAG = "CONTROL";
const char *TAG_INIT = "CONTROL_INIT";
const char *TAG_TX __attribute__((unused)) = "CONTROL_TX";
const char *TAG_RX __attribute__((unused)) = "CONTROL_RX";

// ============================================================================
// Task Configuration
// ============================================================================
// Stack sizes are intentionally the same to keep headroom consistent while
// bringing up peripherals and CAN parsing paths.
// Priority order matters:
//   CAN RX (highest)      - avoid dropping inbound Safety/Planner frames
//   Control loop (middle) - run state + actuator decisions deterministically
//   Heartbeat (lowest)    - periodic status, less latency-sensitive

constexpr int CAN_RX_TASK_STACK = 8192;
constexpr int CONTROL_TASK_STACK = 8192;
constexpr int HEARTBEAT_TASK_STACK = 8192;
constexpr UBaseType_t CAN_RX_TASK_PRIO = 7;
constexpr UBaseType_t CONTROL_TASK_PRIO = 6;
constexpr UBaseType_t HEARTBEAT_TASK_PRIO = 4;

// ============================================================================
// Timing Constants
// ============================================================================
// Keep these values aligned with system contracts:
// - Control loop must run faster than planner command timeout windows.
// - Heartbeat interval follows shared CAN protocol constant.
// - Enable/pedal timings gate safe transitions (no abrupt takeover).

constexpr TickType_t CAN_RX_TIMEOUT = pdMS_TO_TICKS(10);                             // wait per RX poll
constexpr TickType_t CONTROL_LOOP_INTERVAL = pdMS_TO_TICKS(20);                      // state/actuation tick (50 Hz)
constexpr TickType_t HEARTBEAT_SEND_INTERVAL = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS); // protocol cadence
constexpr int16_t THROTTLE_LEVEL_MIN = 0;                                            // digipot minimum level
constexpr int16_t THROTTLE_LEVEL_MAX = 255;                                          // digipot maximum level
constexpr uint8_t PT_STREAM_STARTUP_FRAMES = 2;                // one active segment + one spare segment
constexpr uint32_t INIT_DWELL_MS = 500;                        // minimum dwell in INIT before READY
constexpr uint32_t ENABLE_SEQUENCE_MS = 200;                   // ENABLE dwell before enable_complete
constexpr uint32_t THROTTLE_SLEW_INTERVAL_MS = 100;            // throttle slew timing interval
constexpr int16_t THROTTLE_SLEW_STEP = 12;                     // throttle: max wiper delta per interval (0-255 range)
constexpr TickType_t PLANNER_CMD_TIMEOUT = pdMS_TO_TICKS(500); // stale planner command cutoff
constexpr uint8_t PLANNER_CMD_STALE_COUNT = 10;                // same sequence seen this many cycles = stale
constexpr uint16_t PEDAL_ADC_THRESHOLD_MV = 500;               // pedal override trigger threshold
constexpr uint32_t PEDAL_REARM_MS = 500;                       // pedal must stay released this long to re-arm
constexpr uint8_t PEDAL_ADC_OVERSAMPLE_COUNT = 8;              // samples averaged per pedal read
constexpr uint8_t PEDAL_ADC_TRIGGER_COUNT = 3;                 // consecutive above-threshold reads to trigger

// Stepper position error threshold (pulses). If the actual encoder position
// diverges from the last commanded target by more than this amount while the
// motor reports in-position/stopped, an external physical override is assumed.
// At 16 microsteps (3200 pulses/rev), 200 pulses ~ 22.5 degrees.
constexpr int32_t STEPPER_POSITION_ERROR_THRESHOLD = 200;

// Stepper liveness watchdog: if no CAN response from a motor within this
// window, treat as NODE_FAULT_CONTROL_MOTOR_COMM and retreat to safe state.
// Must be longer than the query interval to allow at least one round-trip.
constexpr TickType_t STEPPER_LIVENESS_TIMEOUT = pdMS_TO_TICKS(500);

// ============================================================================
// Stepper Command Envelope Limits (pulses)
// ============================================================================
// These define the safe operating range for planner steering/braking commands.
// Out-of-range commands are clamped before actuation. The same values are
// programmed into the motor's hardware software-limits (LM) during configure
// as a secondary safety net.

constexpr int32_t STEERING_POSITION_MIN = -3200 * 50; // negative limit (pulses)
constexpr int32_t STEERING_POSITION_MAX = 3200 * 50;  // positive limit (pulses)
constexpr int16_t BRAKING_POSITION_MIN = 0;           // lower limit (pulses, full brake)
constexpr int16_t BRAKING_POSITION_MAX = 28000;       // upper limit (pulses, no brake)

// ============================================================================
// Retry & Fault Constants
// ============================================================================
// Fault tolerance tuning:
// - CAN_TX_CONSEC_FAIL_THRESHOLD filters transient TX errors before recovery.
// - RETRY_INTERVAL_MS paces component re-init attempts (no max attempts).

constexpr uint8_t CAN_TX_CONSEC_FAIL_THRESHOLD = 5;
constexpr uint32_t RETRY_INTERVAL_MS = 500; // component retry cadence (infinite, no cap)

// ============================================================================
// GPIO Pin Assignments
// ============================================================================

// CAN bus (WAVESHARE SN65HVD230 transceiver)
constexpr gpio_num_t TWAI_TX_GPIO = GPIO_NUM_4;
constexpr gpio_num_t TWAI_RX_GPIO = GPIO_NUM_5;

// MCP41HV51 digital potentiometer (throttle level selection via SPI)
constexpr gpio_num_t DIGIPOT_SDI_GPIO = GPIO_NUM_2;
constexpr gpio_num_t DIGIPOT_SCK_GPIO = GPIO_NUM_3;
constexpr gpio_num_t DIGIPOT_CS_GPIO = GPIO_NUM_6;

// DPDT relay (MY5NJ — throttle source switching + pedal bypass)
constexpr gpio_num_t DPDT_RELAY_GPIO = GPIO_NUM_10; // 2N5551 base (via 680R) for MY5NJ

// Pedal ADC (voltage divider from pot wiper)
constexpr adc_channel_t PEDAL_ADC_CHANNEL = ADC_CHANNEL_0; // GPIO 0

// F/R PC817 channels - detect gear selector contacts
constexpr gpio_num_t FR_FORWARD_GPIO = GPIO_NUM_22; // forward contact optocoupler
constexpr gpio_num_t FR_REVERSE_GPIO = GPIO_NUM_23; // reverse contact optocoupler

// ============================================================================
// Component Configurations
// ============================================================================

digipot_mcp41hv51_config_t g_digipot_cfg = {
	.spi_host = SPI2_HOST,
	.sdi = DIGIPOT_SDI_GPIO,
	.sck = DIGIPOT_SCK_GPIO,
	.cs = DIGIPOT_CS_GPIO,
};

relay_dpdt_my5nj_config_t g_dpdt_relay_cfg = {
	.gpio = DPDT_RELAY_GPIO,
};

adc_12bitsar_config_t g_pedal_adc_cfg = {
	.adc_unit = ADC_UNIT_1,
	.adc_channel = PEDAL_ADC_CHANNEL,
};

optocoupler_pc817_config_t g_fr_pc817_cfg = {
	.forward_gpio = FR_FORWARD_GPIO,
	.reverse_gpio = FR_REVERSE_GPIO,
};

// ============================================================================
// Thread-Safe Shared State (CAN RX task -> Control/Heartbeat tasks)
// ============================================================================

// Snapshot of data written by CAN RX task, read by Control and Heartbeat tasks.
// All access must be protected by g_cmd_lock.
struct command_snapshot_t
{
	node_state_t target_state;       // from Safety heartbeat
	node_fault_t safety_fault_flags; // from Safety heartbeat fault_flags channel
	node_stop_t safety_stop_flags;   // from Safety heartbeat stop_flags channel
	int16_t throttle_cmd;
	int32_t steering_cmd;
	int16_t braking_cmd;
	node_fault_t motor_fault_code; // set by CAN RX on stepper error
	TickType_t last_planner_cmd_tick;
	node_seq_t planner_cmd_last_seq;
	uint8_t planner_cmd_stale_count;
};

portMUX_TYPE g_cmd_lock = portMUX_INITIALIZER_UNLOCKED;
command_snapshot_t g_cmd = {}; // protected by g_cmd_lock

// ============================================================================
// Local State (Control task only - no lock needed)
// ============================================================================

// State machine
volatile node_state_t g_control_state = NODE_STATE_INIT;
volatile node_fault_t g_fault_flags = NODE_FAULT_NONE;
volatile node_stop_t g_stop_flags = NODE_STOP_NONE;
volatile node_status_flags_t g_heartbeat_status_flags = 0; // NODE_STATUS_FLAG_* for next heartbeat

// Throttle
volatile int16_t g_throttle_current = 0;

// Driver override inputs (maintained in main, not a separate policy component)
bool g_pedal_input_ready = false;
bool g_fr_inputs_ready = false;
uint16_t g_pedal_mv = 0;
bool g_pedal_was_above = false;
uint32_t g_pedal_below_threshold_since = 0;
uint8_t g_pedal_above_count = 0; // consecutive above-threshold cycle counter

// Timing state
uint32_t g_enable_start_ms = 0;
uint32_t g_last_throttle_change_ms = 0;
bool g_enable_work_done = false;
uint32_t g_boot_start_ms = 0;

// Heartbeat
volatile node_seq_t g_heartbeat_seq = 0;

// Spinlock for heartbeat sequence (shared by control_task immediate send + heartbeat_task)
portMUX_TYPE g_hb_seq_lock = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// Recovery State
// ============================================================================

struct recovery_state_t
{
	uint8_t can_tx_fail_count; // consecutive CAN TX failures
};

recovery_state_t g_recovery = {};
volatile bool g_can_recovery_in_progress = false;
uint8_t g_can_tx_in_flight = 0;

// Component health tracking for startup checks and persistent retries.
volatile bool g_twai_ready = false;
volatile bool g_digipot_ready = false;
volatile bool g_dpdt_relay_ready = false;
volatile bool g_stepper_steering_ready = false;
volatile bool g_stepper_braking_ready = false;

// CAN RX task handle — used to verify quiesce during recovery.
#ifndef CONFIG_THROTTLE_TEST_MODE
TaskHandle_t g_can_rx_task_handle = nullptr;
#endif

// Spinlock for CAN TX tracking / recovery (used by control_task + heartbeat_task)
portMUX_TYPE g_can_tx_lock = portMUX_INITIALIZER_UNLOCKED;

// Retry pacing: last timestamp (ms) when retry_failed_components ran.
// Retries are infinite (no cap) but paced at RETRY_INTERVAL_MS to avoid
// hammering init calls on every control loop tick.
uint32_t g_last_retry_ms = 0;

// Heartbeat monitor: detect Safety heartbeat timeout on Control.
heartbeat_monitor_t g_hb_monitor;
int g_node_safety = -1;

// ============================================================================
// Stepper Motor Instances (UIM2852CA)
// ============================================================================

[[maybe_unused]] stepper_motor_uim2852_t g_steering_stepper = {};
[[maybe_unused]] stepper_motor_uim2852_t g_braking_stepper = {};

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

/**
 * @brief Clamp Safety's raw target state to the valid command set.
 *
 * Safety only sends NOT_READY, READY, ENABLE, or ACTIVE as target
 * commands.  Any other value (corruption, version mismatch) is treated
 * as NOT_READY for a safe retreat.
 *
 * @param raw_state  Raw target state byte from Safety heartbeat
 * @return Sanitized target state (one of NOT_READY/READY/ENABLE/ACTIVE)
 */
node_state_t sanitize_safety_target_state(node_state_t raw_state)
{
	switch (raw_state)
	{
	case NODE_STATE_NOT_READY:
	case NODE_STATE_READY:
	case NODE_STATE_ENABLE:
	case NODE_STATE_ACTIVE:
		return raw_state;
	default:
		// Safety target commands only use NOT_READY/READY/ENABLE/ACTIVE.
		// Treat all other values as NOT_READY (safe retreat).
		return NODE_STATE_NOT_READY;
	}
}

void mark_component_lost(volatile bool *ready, const char *name, const char *detail);

// ============================================================================
// Driver Input Helpers (pedal ADC + F/R PC817)
// ============================================================================

/**
 * @brief Initialize the pedal ADC input and take an initial reading.
 *
 * Deinitializes any prior ADC driver state, re-initializes with the
 * global pedal ADC config, takes one validation read, and seeds the
 * pedal-above-threshold tracker.  Sets g_pedal_input_ready on success.
 *
 * @return ESP_OK on success, or an error code from the ADC driver
 */
[[maybe_unused]] esp_err_t init_pedal_input(void)
{
	adc_12bitsar_deinit();
	g_pedal_input_ready = false;

	esp_err_t err = adc_12bitsar_init(&g_pedal_adc_cfg);
	if (err != ESP_OK)
		return err;

	err = adc_12bitsar_read_mv_checked(&g_pedal_mv);
	if (err != ESP_OK)
		return err;
	g_pedal_was_above = (g_pedal_mv >= PEDAL_ADC_THRESHOLD_MV);
	g_pedal_above_count = g_pedal_was_above ? PEDAL_ADC_TRIGGER_COUNT : 0;
	g_pedal_below_threshold_since = 0;

	g_pedal_input_ready = true;
	return ESP_OK;
}

/**
 * @brief Initialize the F/R gear selector optocoupler inputs.
 *
 * Configures the PC817 dual-optocoupler GPIOs and validates the
 * initial gear state (must not be NEUTRAL or INVALID unless bypassed).
 * Sets g_fr_inputs_ready on success.
 *
 * @return ESP_OK on success, ESP_ERR_INVALID_STATE if initial gear is invalid
 */
[[maybe_unused]] esp_err_t init_fr_inputs(void)
{
	g_fr_inputs_ready = false;

	esp_err_t err = optocoupler_pc817_init(&g_fr_pc817_cfg);
	if (err != ESP_OK)
		return err;

	// Accept any initial FR state.  Pre-bypass, the anti-arcing switch
	// can't conduct, so FORWARD reads as NEUTRAL and REVERSE reads as
	// INVALID — both are expected.  The precondition check (not-in-reverse)
	// gates READY, and the full truth table becomes available once the
	// DPDT relay is energized at COMPLETE_ENABLE (entry to ACTIVE).

	g_fr_inputs_ready = true;
	return ESP_OK;
}

/**
 * @brief Poll pedal ADC and F/R optocoupler, updating global input state.
 *
 * Reads the pedal voltage, applies the override threshold and re-arm
 * logic, and calls optocoupler_pc817_update() for debounced F/R state.
 * Marks components lost on runtime read failures.
 *
 * @param now_ms  Current timestamp in milliseconds
 */
void update_driver_inputs(uint32_t now_ms)
{
#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
	if (g_pedal_input_ready)
	{
		// Oversample: average PEDAL_ADC_OVERSAMPLE_COUNT readings to reject noise
		uint32_t sample_sum = 0;
		uint8_t good_samples = 0;
		for (uint8_t i = 0; i < PEDAL_ADC_OVERSAMPLE_COUNT; ++i)
		{
			uint16_t sample = 0;
			if (adc_12bitsar_read_mv_checked(&sample) == ESP_OK)
			{
				sample_sum += sample;
				++good_samples;
			}
		}

		if (good_samples == 0)
		{
			mark_component_lost(&g_pedal_input_ready, "PEDAL_INPUT", "runtime adc read failure");
		}
		else
		{
			g_pedal_mv = (uint16_t)(sample_sum / good_samples);

			if (g_pedal_mv >= PEDAL_ADC_THRESHOLD_MV)
			{
				if (g_pedal_above_count < PEDAL_ADC_TRIGGER_COUNT)
					++g_pedal_above_count;

#ifdef CONFIG_LOG_INPUT_PEDAL_ADC
				ESP_LOGI(TAG, "Pedal: %u mV (threshold: %u mV, count: %u/%u)", g_pedal_mv,
				         (unsigned)PEDAL_ADC_THRESHOLD_MV, g_pedal_above_count, (unsigned)PEDAL_ADC_TRIGGER_COUNT);
#endif

				if (g_pedal_above_count >= PEDAL_ADC_TRIGGER_COUNT)
				{
#ifdef CONFIG_LOG_INPUT_PEDAL_EVENTS
					if (!g_pedal_was_above)
					{
						ESP_LOGI(TAG, "Pedal threshold met: %u mV (threshold: %u mV)", g_pedal_mv,
						         (unsigned)PEDAL_ADC_THRESHOLD_MV);
					}
#endif
					g_pedal_was_above = true;
					g_pedal_below_threshold_since = 0;
				}
			}
			else
			{
				g_pedal_above_count = 0;

				if (g_pedal_was_above && g_pedal_below_threshold_since == 0)
					g_pedal_below_threshold_since = now_ms;

				if (g_pedal_below_threshold_since > 0 && (now_ms - g_pedal_below_threshold_since) >= PEDAL_REARM_MS)
				{
					g_pedal_was_above = false;
					g_pedal_below_threshold_since = 0;

#ifdef CONFIG_LOG_INPUT_PEDAL_EVENTS
					ESP_LOGI(TAG, "Pedal re-armed");
#endif
				}
			}
		}
	}
#endif

	if (g_fr_inputs_ready)
		optocoupler_pc817_update(now_ms);
}

/**
 * @brief Check whether the driver pedal is currently pressed above threshold.
 *
 * @return true if pedal input is ready and debounce trigger count is met
 */
bool pedal_pressed(void)
{
	return g_pedal_input_ready && (g_pedal_above_count >= PEDAL_ADC_TRIGGER_COUNT);
}

/**
 * @brief Check whether the driver pedal has been released long enough to re-arm.
 *
 * The pedal must stay below threshold for PEDAL_REARM_MS after an
 * override before autonomous control can resume.  Returns true if the
 * pedal input is unavailable (fail-safe: assume released).
 *
 * @return true if pedal is re-armed or input is unavailable
 */
bool pedal_rearmed(void)
{
	if (!g_pedal_input_ready)
		return true;
	if (g_pedal_mv >= PEDAL_ADC_THRESHOLD_MV)
		return false;
	return !g_pedal_was_above;
}

/**
 * @brief Return the current debounced F/R gear state.
 *
 * Returns NEUTRAL if the F/R input is not initialized (fail-safe).
 *
 * @return Debounced gear state (FORWARD, REVERSE, NEUTRAL, or INVALID)
 */
fr_state_t fr_state_debounced(void)
{
	if (!g_fr_inputs_ready)
		return FR_STATE_NEUTRAL;
	return optocoupler_pc817_get_state();
}

#ifndef CONFIG_THROTTLE_TEST_MODE
/**
 * @brief Log the initialization status of all Control ESP32 peripherals.
 *
 * Emits one INFO or ERROR line per component showing GPIO assignments,
 * configuration details, and pass/fail status.  Bypass-disabled
 * components are logged as BYPASSED at WARNING level.
 *
 * @param twai_ready              TWAI driver initialized successfully
 * @param digipot_ready           MCP41HV51 digipot initialized
 * @param dpdt_relay_ready        MY5NJ DPDT relay initialized
 * @param pedal_ready             Pedal ADC initialized and validated
 * @param fr_ready                F/R optocoupler initialized and valid
 * @param stepper_steering_ready  Steering stepper initialized and configured
 * @param stepper_braking_ready   Braking stepper initialized and configured
 * @param heartbeat_ready         WS2812 status LED initialized
 */
void log_startup_device_status(bool twai_ready, bool digipot_ready, bool dpdt_relay_ready, bool pedal_ready,
                               bool fr_ready, bool stepper_steering_ready, bool stepper_braking_ready,
                               bool heartbeat_ready)
{
	if (twai_ready)
		ESP_LOGI(TAG_INIT, "TWAI: CONFIGURED: tx_gpio=%d rx_gpio=%d", TWAI_TX_GPIO, TWAI_RX_GPIO);
	else
		ESP_LOGE(TAG_INIT, "TWAI: FAILED: tx_gpio=%d rx_gpio=%d", TWAI_TX_GPIO, TWAI_RX_GPIO);

#ifdef CONFIG_BYPASS_ACTUATOR_THROTTLE
	ESP_LOGW(TAG_INIT, "DIGIPOT: BYPASSED: disabled (sdi=%d sck=%d cs=%d)", g_digipot_cfg.sdi, g_digipot_cfg.sck,
	         g_digipot_cfg.cs);
	ESP_LOGW(TAG_INIT, "DPDT_RELAY: BYPASSED: disabled (gpio=%d)", g_dpdt_relay_cfg.gpio);
#else
	if (digipot_ready)
	{
		ESP_LOGI(TAG_INIT, "DIGIPOT: CONFIGURED: sdi=%d sck=%d cs=%d start=WIPER_0", g_digipot_cfg.sdi,
		         g_digipot_cfg.sck, g_digipot_cfg.cs);
	}
	else
	{
		ESP_LOGE(TAG_INIT, "DIGIPOT: FAILED: sdi=%d sck=%d cs=%d", g_digipot_cfg.sdi, g_digipot_cfg.sck,
		         g_digipot_cfg.cs);
	}

	if (dpdt_relay_ready)
	{
		ESP_LOGI(TAG_INIT, "DPDT_RELAY: CONFIGURED: gpio=%d active_level=HIGH start=DE-ENERGIZED",
		         g_dpdt_relay_cfg.gpio);
	}
	else
	{
		ESP_LOGE(TAG_INIT, "DPDT_RELAY: FAILED: gpio=%d active_level=HIGH", g_dpdt_relay_cfg.gpio);
	}
#endif

#ifdef CONFIG_BYPASS_INPUT_PEDAL_ADC
	ESP_LOGW(TAG_INIT, "PEDAL_INPUT: BYPASSED: pedal override disabled (forced released, threshold_mv=%u)",
	         (unsigned)PEDAL_ADC_THRESHOLD_MV);
#else
	if (pedal_ready)
	{
		ESP_LOGI(TAG_INIT, "PEDAL_INPUT: CONFIGURED: adc_channel=ADC%d_CH%d adc_mode=%s threshold_mv=%u initial_mv=%u",
		         g_pedal_adc_cfg.adc_unit + 1, g_pedal_adc_cfg.adc_channel,
		         adc_12bitsar_is_calibrated() ? "curve_fit" : "raw", (unsigned)PEDAL_ADC_THRESHOLD_MV, g_pedal_mv);
	}
	else
	{
		ESP_LOGE(TAG_INIT, "PEDAL_INPUT: FAILED: adc_channel=ADC%d_CH%d threshold_mv=%u", g_pedal_adc_cfg.adc_unit + 1,
		         g_pedal_adc_cfg.adc_channel, (unsigned)PEDAL_ADC_THRESHOLD_MV);
	}
#endif

#ifdef CONFIG_BYPASS_INPUT_FR_SENSOR
	ESP_LOGW(TAG_INIT, "FR_INPUT_FORWARD: BYPASSED: input forced (input_gpio=%d)", g_fr_pc817_cfg.forward_gpio);
	ESP_LOGW(TAG_INIT, "FR_INPUT_REVERSE: BYPASSED: input forced (input_gpio=%d)", g_fr_pc817_cfg.reverse_gpio);
#else
	if (fr_ready)
	{
		ESP_LOGI(TAG_INIT, "FR_INPUT_FORWARD: OK: input_gpio=%d", g_fr_pc817_cfg.forward_gpio);
		ESP_LOGI(TAG_INIT, "FR_INPUT_REVERSE: OK: input_gpio=%d", g_fr_pc817_cfg.reverse_gpio);
	}
	else
	{
		ESP_LOGE(TAG_INIT, "FR_INPUT_FORWARD: FAILED: input_gpio=%d", g_fr_pc817_cfg.forward_gpio);
		ESP_LOGE(TAG_INIT, "FR_INPUT_REVERSE: FAILED: input_gpio=%d", g_fr_pc817_cfg.reverse_gpio);
	}
#endif

#ifdef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
	ESP_LOGW(TAG_INIT, "STEPPER_STEERING: BYPASSED: actuator disabled (node_id=%u)", (unsigned)UIM2852_NODE_STEERING);
#else
	if (stepper_steering_ready)
	{
		ESP_LOGI(TAG_INIT, "STEPPER_STEERING: OK: node_id=%u init_check=passed", (unsigned)UIM2852_NODE_STEERING);
	}
	else
	{
		ESP_LOGE(TAG_INIT, "STEPPER_STEERING: FAILED: node_id=%u init_check=failed", (unsigned)UIM2852_NODE_STEERING);
	}
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
	ESP_LOGW(TAG_INIT, "STEPPER_BRAKING: BYPASSED: actuator disabled (node_id=%u)", (unsigned)UIM2852_NODE_BRAKING);
#else
	if (stepper_braking_ready)
	{
		ESP_LOGI(TAG_INIT, "STEPPER_BRAKING: OK: node_id=%u init_check=passed", (unsigned)UIM2852_NODE_BRAKING);
	}
	else
	{
		ESP_LOGE(TAG_INIT, "STEPPER_BRAKING: FAILED: node_id=%u init_check=failed", (unsigned)UIM2852_NODE_BRAKING);
	}
#endif

	if (heartbeat_ready)
		ESP_LOGI(TAG_INIT, "HEARTBEAT_LED: CONFIGURED");
	else
		ESP_LOGW(TAG_INIT, "HEARTBEAT_LED: UNAVAILABLE (non-critical)");
}
#endif // !CONFIG_THROTTLE_TEST_MODE

/**
 * @brief Check whether all non-bypassed base components report ready.
 *
 * Evaluates the health flags for every component that is not
 * compile-time bypassed, excluding stepper comm availability.
 *
 * Stepper comm is intentionally excluded from this baseline readiness check:
 * steppers are powered by Safety's relay only in ENABLE/ACTIVE, so requiring
 * them in NOT_READY/READY would create a power-dependency deadlock.
 *
 * @return true if every required non-stepper component is initialized and healthy
 */
bool all_required_components_ready(void)
{
	bool ready = true;
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	if (!g_digipot_ready)
		ready = false;
	if (!g_dpdt_relay_ready)
		ready = false;
#endif
#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
	if (!g_pedal_input_ready)
		ready = false;
#endif
#ifndef CONFIG_BYPASS_INPUT_FR_SENSOR
	if (!g_fr_inputs_ready)
		ready = false;
#endif
	return ready;
}

/**
 * @brief Return the highest-priority fault code for the first unhealthy base component.
 *
 * Scans non-bypassed base components in priority order (throttle, relay,
 * sensors) and returns the fault code for the first one that is not ready.
 * Stepper comm health is handled by runtime motor fault paths in ENABLE/ACTIVE.
 * Returns NODE_FAULT_NONE if all baseline components are healthy.
 *
 * @return Fault code corresponding to the first failed component
 */
node_fault_t primary_fault_from_component_health(void)
{
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	if (!g_digipot_ready)
		return NODE_FAULT_CONTROL_THROTTLE_INIT;
	if (!g_dpdt_relay_ready)
		return NODE_FAULT_CONTROL_RELAY_INIT;
#endif
#if !defined(CONFIG_BYPASS_INPUT_PEDAL_ADC) || !defined(CONFIG_BYPASS_INPUT_FR_SENSOR)
	if (!g_pedal_input_ready || !g_fr_inputs_ready)
		return NODE_FAULT_CONTROL_SENSOR_INVALID;
#endif
	return NODE_FAULT_NONE;
}

/**
 * @brief Convert an F/R gear state enum to a human-readable string.
 *
 * @param state  Gear state to convert
 * @return Static string label ("forward", "reverse", "neutral", "wiring_fault", or "unknown")
 */
const char *fr_state_to_string(fr_state_t state)
{
	switch (state)
	{
	case FR_STATE_FORWARD:
		return "forward";
	case FR_STATE_REVERSE:
		return "reverse";
	case FR_STATE_NEUTRAL:
		return "neutral";
	case FR_STATE_INVALID:
		return "wiring_fault";
	default:
		return "unknown";
	}
}

/**
 * @brief Build a diagnostic detail string for NODE_FAULT_CONTROL_SENSOR_INVALID.
 *
 * Identifies which sensor input caused the fault: F/R invalid reading,
 * pedal input unavailable, F/R input unavailable, or a combination.
 *
 * @param fr_state       Current F/R gear state
 * @param has_fr_sample  true if at least one F/R reading has been taken
 * @return Static detail string describing the sensor failure
 */
const char *sensor_fault_detail_string(fr_state_t fr_state, bool has_fr_sample)
{
	if (has_fr_sample && fr_state == FR_STATE_INVALID)
	{
		return "fr_state_invalid";
	}

	bool pedal_unavailable = false;
	bool fr_unavailable = false;

#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
	pedal_unavailable = !g_pedal_input_ready;
#endif
#ifndef CONFIG_BYPASS_INPUT_FR_SENSOR
	fr_unavailable = !g_fr_inputs_ready;
#endif

	if (pedal_unavailable && fr_unavailable)
		return "pedal_input+fr_input_unavailable";
	if (pedal_unavailable)
		return "pedal_input_unavailable";
	if (fr_unavailable)
		return "fr_input_unavailable";
	return "unspecified";
}

/**
 * @brief Return a diagnostic detail string for a given fault code, or NULL.
 *
 * Currently only provides detail for NODE_FAULT_CONTROL_SENSOR_INVALID.
 * All other fault codes return NULL (no additional detail).
 *
 * @param fault_flags    Active fault flags
 * @param fr_state       Current F/R gear state
 * @param has_fr_sample  true if at least one F/R reading has been taken
 * @return Detail string or NULL if no additional detail is available
 */
const char *fault_detail_string(node_fault_t fault_flags, fr_state_t fr_state, bool has_fr_sample)
{
	if (fault_flags == NODE_FAULT_CONTROL_SENSOR_INVALID)
	{
		return sensor_fault_detail_string(fr_state, has_fr_sample);
	}
	return nullptr;
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
 * @brief Mark a component lost if a runtime operation returned an error.
 *
 * No-op when err is ESP_OK.  Otherwise delegates to mark_component_lost()
 * to clear the ready flag and log the failure.
 *
 * @param err     Error code from the component operation
 * @param ready   Pointer to the component's health flag
 * @param name    Component name for logging
 * @param detail  Failure detail string for logging
 */
[[maybe_unused]] void handle_runtime_error(esp_err_t err, volatile bool *ready, const char *name, const char *detail)
{
	if (err == ESP_OK)
		return;
	mark_component_lost(ready, name, detail);
}

/**
 * @brief Execute the safe shutdown sequence for a stepper motor.
 *
 * Issues emergency stop then disables the motor driver (MO=0).
 * Each step is best-effort via handle_runtime_error — failures are
 * logged but do not abort the sequence so subsequent steps still execute.
 *
 * @param motor  Motor instance to shut down
 * @param ready  Pointer to the motor's health flag
 * @param label  Motor name for error logging (e.g. "STEPPER_STEERING")
 */
[[maybe_unused]] void stepper_shutdown(stepper_motor_uim2852_t *motor, volatile bool *ready, const char *label)
{
	handle_runtime_error(stepper_motor_uim2852_emergency_stop(motor), ready, label, "emergency stop command failed");
	handle_runtime_error(stepper_motor_uim2852_disable(motor), ready, label, "disable command failed");
}

/**
 * @brief Apply base component-health fault gating.
 *
 * If any required base component is unhealthy, force fault_flags to the
 * highest-priority component fault and retreat state to NOT_READY.
 * If all required base components recover and the current fault code is a
 * base component fault, clear it.
 */
void update_control_state_from_component_health(void)
{
	if (!all_required_components_ready())
	{
		taskENTER_CRITICAL(&g_hb_seq_lock);
		g_fault_flags = primary_fault_from_component_health();
		if (g_control_state != NODE_STATE_INIT)
			g_control_state = NODE_STATE_NOT_READY;
		taskEXIT_CRITICAL(&g_hb_seq_lock);
		return;
	}

	if (g_fault_flags == NODE_FAULT_CONTROL_THROTTLE_INIT || g_fault_flags == NODE_FAULT_CONTROL_RELAY_INIT)
	{
		taskENTER_CRITICAL(&g_hb_seq_lock);
		g_fault_flags = NODE_FAULT_NONE;
		taskEXIT_CRITICAL(&g_hb_seq_lock);
	}
}

/**
 * @brief Wait for CAN RX and heartbeat tasks to finish in-flight TWAI calls.
 *
 * Must be called after setting g_twai_ready = false, before touching
 * the TWAI driver for recovery.  Forwarded from node_utils.
 */
void quiesce_can_rx()
{
	can_twai_quiesce(&g_can_tx_lock, &g_can_tx_in_flight);
}

// Rate-limit retry failure logs: log on 1st attempt, then every RETRY_LOG_EVERY_N
// attempts. At RETRY_INTERVAL_MS = 500ms, N=20 gives ~10 second intervals.
const uint16_t RETRY_LOG_EVERY_N = 20;

/**
 * @brief Re-initialize any failed components at a rate-limited cadence.
 *
 * Called every control loop tick, but paced at RETRY_INTERVAL_MS to
 * avoid hammering init calls.  Retries are infinite with no cap.
 * For TWAI recovery, performs a full teardown/reinit cycle with an
 * active probe heartbeat to verify bus connectivity.  On success,
 * calls update_control_state_from_component_health() to potentially
 * clear base component issue causes.
 */
void retry_failed_components(void)
{
	// Pace retries at RETRY_INTERVAL_MS to avoid hammering init calls every
	// control loop tick. Retries remain infinite (no cap).
	uint32_t now_ms = get_time_ms();
	if ((now_ms - g_last_retry_ms) < RETRY_INTERVAL_MS)
		return;
	g_last_retry_ms = now_ms;

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
				.state = g_control_state,
				.fault_flags = g_fault_flags,
				.status_flags = g_heartbeat_status_flags,
				.stop_flags = g_stop_flags,
			};
			can_encode_heartbeat(probe_data, &probe_hb);
			(void)can_twai_send(CAN_ID_CONTROL_HEARTBEAT, probe_data, pdMS_TO_TICKS(50));
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

#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	if (!g_digipot_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = digipot_mcp41hv51_init(&g_digipot_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_DIGIPOT
		if (!recovered)
		{
			s_retry_count++;
			if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0)
			{
				ESP_LOGW(TAG, "DIGIPOT retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			log_component_regained("DIGIPOT (driver-level)");
		}
		g_digipot_ready = recovered;
	}

	if (!g_dpdt_relay_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = relay_dpdt_my5nj_init(&g_dpdt_relay_cfg);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_DPDT_RELAY
		if (!recovered)
		{
			s_retry_count++;
			if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0)
			{
				ESP_LOGW(TAG, "DPDT_RELAY retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			log_component_regained("DPDT_RELAY (driver-level)");
		}
		g_dpdt_relay_ready = recovered;
	}
#endif

#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
	if (!g_pedal_input_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = init_pedal_input();
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_PEDAL_INPUT
		if (!recovered)
		{
			s_retry_count++;
			if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0)
			{
				ESP_LOGW(TAG, "PEDAL_INPUT retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			log_component_regained("PEDAL_INPUT");
		}
		g_pedal_input_ready = recovered;
	}
#endif

#ifndef CONFIG_BYPASS_INPUT_FR_SENSOR
	if (!g_fr_inputs_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		esp_err_t err = init_fr_inputs();
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_FR_INPUT
		if (!recovered)
		{
			s_retry_count++;
			if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0)
			{
				ESP_LOGW(TAG, "FR_INPUT_FORWARD retry failed (%u attempts): %s (input_gpio=%d)", s_retry_count,
				         esp_err_to_name(err), g_fr_pc817_cfg.forward_gpio);
				ESP_LOGW(TAG, "FR_INPUT_REVERSE retry failed (%u attempts): %s (input_gpio=%d)", s_retry_count,
				         esp_err_to_name(err), g_fr_pc817_cfg.reverse_gpio);
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			fr_state_t recovered_state = optocoupler_pc817_get_state();
			char fr_regained[48];
			snprintf(fr_regained, sizeof(fr_regained), "FR_INPUT (fr=%s)", fr_state_to_string(recovered_state));
			log_component_regained(fr_regained);
		}
		g_fr_inputs_ready = recovered;
	}
#endif

#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
	if (!g_stepper_steering_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		stepper_motor_uim2852_config_t s_cfg = STEPPER_MOTOR_UIM2852_CONFIG_DEFAULT();
		s_cfg.node_id = UIM2852_NODE_STEERING;
		esp_err_t err = stepper_motor_uim2852_init(&g_steering_stepper, &s_cfg);
		if (err == ESP_OK)
		{
			(void)stepper_motor_uim2852_disable(&g_steering_stepper); // MO=0 hot-start
			err = stepper_motor_uim2852_configure(&g_steering_stepper);
		}
		if (err == ESP_OK)
			err = stepper_motor_uim2852_set_limits(&g_steering_stepper, STEERING_POSITION_MIN, STEERING_POSITION_MAX);
		if (err == ESP_OK)
			err = stepper_motor_uim2852_disable(&g_steering_stepper);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_STEPPER_STEERING
		if (!recovered)
		{
			s_retry_count++;
			if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0)
			{
				ESP_LOGW(TAG, "STEPPER_STEERING retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			log_component_regained("STEPPER_STEERING");
		}
		g_stepper_steering_ready = recovered;
	}
#endif

#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
	if (!g_stepper_braking_ready)
	{
		[[maybe_unused]] static uint16_t s_retry_count = 0;
		stepper_motor_uim2852_config_t b_cfg = STEPPER_MOTOR_UIM2852_CONFIG_DEFAULT();
		b_cfg.node_id = UIM2852_NODE_BRAKING;
		esp_err_t err = stepper_motor_uim2852_init(&g_braking_stepper, &b_cfg);
		if (err == ESP_OK)
		{
			(void)stepper_motor_uim2852_disable(&g_braking_stepper); // MO=0 hot-start
			err = stepper_motor_uim2852_configure(&g_braking_stepper);
		}
		if (err == ESP_OK)
			err = stepper_motor_uim2852_set_limits(&g_braking_stepper, BRAKING_POSITION_MIN, BRAKING_POSITION_MAX);
		if (err == ESP_OK)
			err = stepper_motor_uim2852_disable(&g_braking_stepper);
		bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_STEPPER_BRAKING
		if (!recovered)
		{
			s_retry_count++;
			if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0)
			{
				ESP_LOGW(TAG, "STEPPER_BRAKING retry failed (%u attempts): %s", s_retry_count, esp_err_to_name(err));
			}
		}
#endif
		if (recovered)
		{
			s_retry_count = 0;
			log_component_regained("STEPPER_BRAKING");
		}
		g_stepper_braking_ready = recovered;
	}
#endif

	update_control_state_from_component_health();
}

// ============================================================================
// LED State Mapping
// ============================================================================

/**
 * @brief Track CAN TX results and trigger bus recovery on consecutive failures.
 *
 * Increments a consecutive-failure counter on TX errors and resets it
 * on success.  When the counter reaches CAN_TX_CONSEC_FAIL_THRESHOLD,
 * checks bus health via can_twai_bus_ok().  If the bus is unhealthy,
 * marks g_twai_ready = false to initiate recovery in
 * retry_failed_components().
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
		.fail_count = g_recovery.can_tx_fail_count,
		.threshold = CAN_TX_CONSEC_FAIL_THRESHOLD,
		.tx_ok = (err == ESP_OK),
	};
	can_tx_track_result_t r = can_tx_track(&t);
	g_recovery.can_tx_fail_count = r.new_fail_count;
	if (r.trigger_recovery)
	{
		g_recovery.can_tx_fail_count = 0; // reset regardless
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
			taskENTER_CRITICAL(&g_can_tx_lock);
			if (!g_can_recovery_in_progress)
			{
				g_can_recovery_in_progress = true;
				trigger_recovery = true;
			}
			taskEXIT_CRITICAL(&g_can_tx_lock);
		}
		// else: bus OK but TX still failing — no partner on bus, nothing to do.
	}

	if (trigger_recovery)
	{
#ifdef CONFIG_LOG_CAN_RECOVERY
		ESP_LOGE(TAG, "CAN bus unhealthy after %d TX failures", CAN_TX_CONSEC_FAIL_THRESHOLD);
#endif
		taskENTER_CRITICAL(&g_can_tx_lock);
		g_twai_ready = false;
		// Recovery handled by retry_failed_components() at 500ms pace.
		g_can_recovery_in_progress = false;
		taskEXIT_CRITICAL(&g_can_tx_lock);
	}
}

// ============================================================================
// Control Heartbeat Send Helper
// ============================================================================

/**
 * @brief Encode and transmit the Control heartbeat frame on CAN.
 *
 * Atomically snapshots the current state, fault code, heartbeat status flags,
 * and sequence number, encodes them into the CAN heartbeat format, and
 * sends via TWAI.  Called from heartbeat_task (periodic 100ms) and
 * from control_task on state change (immediate).  Guards against
 * concurrent recovery by checking g_twai_ready and tracking in-flight
 * TX count under spinlock.
 */
void send_control_heartbeat(void)
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
		ESP_LOGE(TAG, "HEAP CORRUPTION detected in send_control_heartbeat! free=%lu",
		         (unsigned long)esp_get_free_heap_size());
	}
#endif

	uint8_t hb_data[8] = {};

	node_seq_t seq = 0;
	node_state_t state = NODE_STATE_INIT;
	node_fault_t fault_flags = NODE_FAULT_NONE;
	node_stop_t stop_flags = NODE_STOP_NONE;
	node_status_flags_t status_flags = 0;
	taskENTER_CRITICAL(&g_hb_seq_lock);
	seq = g_heartbeat_seq;
	g_heartbeat_seq = (node_seq_t)(seq + 1);
	state = g_control_state;
	fault_flags = g_fault_flags;
	stop_flags = g_stop_flags;
	status_flags = g_heartbeat_status_flags;
	taskEXIT_CRITICAL(&g_hb_seq_lock);

	node_heartbeat_t hb_msg = {
		.sequence = seq,
		.state = state,
		.fault_flags = fault_flags,
		.status_flags = status_flags,
		.stop_flags = stop_flags,
	};
	can_encode_heartbeat(hb_data, &hb_msg);

	esp_err_t err = can_twai_send(CAN_ID_CONTROL_HEARTBEAT, hb_data, pdMS_TO_TICKS(10));

	taskENTER_CRITICAL(&g_can_tx_lock);
	if (g_can_tx_in_flight > 0)
		g_can_tx_in_flight--;
	taskEXIT_CRITICAL(&g_can_tx_lock);

	track_can_tx(err);
}

// ============================================================================
// State Machine Action Helpers
// ============================================================================

/**
 * @brief Convert an override reason enum to a human-readable string.
 *
 * @param reason  Override reason to convert
 * @return Static string label for logging
 */
[[maybe_unused]] const char *override_reason_to_string(override_reason_t reason)
{
	switch (reason)
	{
	case OVERRIDE_REASON_THROTTLE:
		return "throttle";
	case OVERRIDE_REASON_REVERSE:
		return "reverse";
	case OVERRIDE_REASON_STEERING:
		return "steering";
	case OVERRIDE_REASON_BRAKING:
		return "braking";
	case OVERRIDE_REASON_NONE:
	default:
		return "none";
	}
}

/**
 * @brief Convert a disable reason enum to a human-readable string.
 *
 * @param reason  Disable reason to convert
 * @return Static string label for logging
 */
[[maybe_unused]] const char *disable_reason_to_string(disable_reason_t reason)
{
	switch (reason)
	{
	case CONTROL_DISABLE_REASON_SAFETY_RETREAT:
		return "safety_retreat";
	case CONTROL_DISABLE_REASON_MOTOR_FAULT:
		return "motor_fault";
	case CONTROL_DISABLE_REASON_SENSOR_INVALID:
		return "sensor_invalid";
	case CONTROL_DISABLE_REASON_INTERNAL:
		return "internal";
	case CONTROL_DISABLE_REASON_NONE:
	default:
		return "none";
	}
}

/**
 * @brief Convert an enable-abort reason enum to a human-readable string.
 *
 * @param reason  Abort reason to convert
 * @return Static string label for logging
 */
[[maybe_unused]] const char *abort_reason_to_string(abort_reason_t reason)
{
	switch (reason)
	{
	case CONTROL_ABORT_REASON_SAFETY_RETREAT:
		return "safety_retreat";
	case CONTROL_ABORT_REASON_PEDAL_PRESSED:
		return "pedal_pressed";
	case CONTROL_ABORT_REASON_FR_IN_REVERSE:
		return "fr_in_reverse";
	case CONTROL_ABORT_REASON_MOTOR_FAULT:
		return "motor_fault";
	case CONTROL_ABORT_REASON_SENSOR_INVALID:
		return "sensor_invalid";
	case CONTROL_ABORT_REASON_NONE:
	default:
		return "none";
	}
}

static bool tick_deadline_reached(TickType_t now, TickType_t deadline)
{
	return (int32_t)(now - deadline) >= 0;
}

/**
 * @brief Feed one PT row on the motor's 100 ms cadence, reusing the last target when needed.
 *
 * The control loop runs at 20 ms, but the planner only updates every 100 ms.
 * This helper maintains a one-segment cushion in the motor's PT FIFO by:
 * - preloading two PT rows when PT motion starts (or restarts after FIFO empty)
 * - enqueueing one new PT row every PT frame interval
 * - attempting the enqueue up to one control tick early so a single late tick
 *   can retry before the current motor segment expires
 *
 * The position passed in is the latest clamped planner-derived target. If the
 * planner skipped a cycle, the caller simply passes the same target again, and
 * the helper keeps the FIFO alive by repeating that hold position.
 *
 * @param motor           Motor instance
 * @param position        Absolute PT target position to queue
 * @param now_tick        Current RTOS tick
 * @param next_feed_tick  [in/out] Next scheduled PT enqueue deadline
 * @param fed_frame       [out] True if at least one PT frame was sent
 * @return ESP_OK on success, or the first pt_feed error encountered
 */
static esp_err_t feed_pt_stream_if_due(stepper_motor_uim2852_t *motor, int32_t position, TickType_t now_tick,
                                       TickType_t *next_feed_tick, bool *fed_frame)
{
	if (!motor || !next_feed_tick || !fed_frame)
		return ESP_ERR_INVALID_ARG;

	*fed_frame = false;

	TickType_t interval_ticks = pdMS_TO_TICKS(motor->pt_frame_time_ms);
	if (interval_ticks == 0)
		interval_ticks = 1;

	TickType_t early_margin = CONTROL_LOOP_INTERVAL;
	if (early_margin > interval_ticks)
		early_margin = interval_ticks;

	bool priming_stream = !motor->pt_motion_started;
	uint8_t frames_to_send = 0;

	if (priming_stream)
	{
		frames_to_send = 1;
		if (motor->pt_prefill_count < PT_STREAM_STARTUP_FRAMES)
			frames_to_send = (uint8_t)(PT_STREAM_STARTUP_FRAMES - motor->pt_prefill_count);
	}
	else
	{
		if (*next_feed_tick == 0)
			*next_feed_tick = now_tick + interval_ticks;

		TickType_t send_check_tick = now_tick + early_margin;
		if (tick_deadline_reached(send_check_tick, *next_feed_tick))
			frames_to_send = 1;
	}

	for (uint8_t i = 0; i < frames_to_send; ++i)
	{
		esp_err_t err = stepper_motor_uim2852_pt_feed(motor, position, motor->pt_frame_time_ms);
		if (err != ESP_OK)
			return err;
		*fed_frame = true;
	}

	if (*fed_frame)
	{
		if (priming_stream)
			*next_feed_tick = now_tick + interval_ticks;
		else
			*next_feed_tick += interval_ticks;
	}

	return ESP_OK;
}

/**
 * @brief Immediately disable all autonomous actuators to a safe state.
 *
 * Executes the full shutdown sequence: disables and emergency-stops
 * the digital potentiometer, de-energizes the DPDT relay, stops PT
 * interpolation on both steppers, then runs stepper_shutdown()
 * (emergency stop, brake, disable) on each motor.
 * Best-effort — individual failures are logged but do not prevent
 * subsequent shutdown steps.
 */
void disable_autonomous_actuators(void)
{
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	handle_runtime_error(digipot_mcp41hv51_disable(), &g_digipot_ready, "DIGIPOT", "disable command failed");
	handle_runtime_error(digipot_mcp41hv51_emergency_stop(), &g_digipot_ready, "DIGIPOT",
	                     "emergency stop command failed");
	handle_runtime_error(relay_dpdt_my5nj_deenergize(), &g_dpdt_relay_ready, "DPDT_RELAY",
	                     "de-energize command failed");
#endif

	// Stop PT interpolation before emergency stop (clears FIFO consumption).
	// Best-effort: errors are ignored since emergency stop follows immediately.
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
	(void)stepper_motor_uim2852_pt_stop(&g_steering_stepper);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
	(void)stepper_motor_uim2852_pt_stop(&g_braking_stepper);
#endif

#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
	stepper_shutdown(&g_steering_stepper, &g_stepper_steering_ready, "STEPPER_STEERING");
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
	stepper_shutdown(&g_braking_stepper, &g_stepper_braking_ready, "STEPPER_BRAKING");
#endif
}

/**
 * @brief Handle a driver override event by disabling all autonomous actuators.
 *
 * Logs the override reason (if logging is enabled) and delegates to
 * disable_autonomous_actuators() for the full shutdown sequence.
 *
 * @param reason  Override trigger (pedal, F/R change, steering/braking error)
 */
void execute_trigger_override(override_reason_t reason)
{
#ifdef CONFIG_LOG_CONTROL_OVERRIDE
	ESP_LOGI(TAG, "OVERRIDE TRIGGERED (reason=%s)", override_reason_to_string(reason));
#else
	(void)reason;
#endif

	disable_autonomous_actuators();
}

/**
 * @brief Disable autonomy without classifying the event as a driver override.
 *
 * Used when Safety retreats the target state, or when a motor/sensor
 * fault forces a transition out of ACTIVE.  Logs the reason and
 * delegates to disable_autonomous_actuators().
 *
 * @param reason              Why autonomy is being disabled
 * @param safety_fault_flags   Safety fault bitmask (for Safety-retreat logging)
 * @param safety_stop_flags   Safety stop bitmask (for Safety-retreat logging)
 * @param control_fault_flags  Control's own issue code (for local issue logging)
 */
void execute_disable_autonomy(disable_reason_t reason, node_fault_t safety_fault_flags, node_stop_t safety_stop_flags,
                              node_fault_t control_fault_flags)
{
#if defined(CONFIG_LOG_CONTROL_STATE_CHANGES) || defined(CONFIG_LOG_CONTROL_ENABLE_SEQUENCE)
	if (reason == CONTROL_DISABLE_REASON_SAFETY_RETREAT)
	{
		ESP_LOGI(TAG, "AUTONOMY DISABLED (reason=%s, safety_fault=%s, safety_stop=%s)",
		         disable_reason_to_string(reason), node_fault_to_string(safety_fault_flags),
		         node_stop_to_string(safety_stop_flags));
	}
	else
	{
		ESP_LOGI(TAG, "AUTONOMY DISABLED (reason=%s, control_issue=%s)", disable_reason_to_string(reason),
		         node_fault_to_string(control_fault_flags));
	}
#else
	(void)reason;
	(void)safety_fault_flags;
	(void)safety_stop_flags;
	(void)control_fault_flags;
#endif

	disable_autonomous_actuators();
}

/**
 * @brief Begin the autonomous enable sequence.
 *
 * Sets the digital potentiometer to wiper position 0 (idle).
 * The DPDT relay is NOT energized here — both poles (throttle source
 * switching and pedal bypass) switch together in execute_complete_enable().
 */
void execute_start_enable()
{
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
	ESP_LOGI(TAG, "Starting autonomous enable sequence");
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	handle_runtime_error(digipot_mcp41hv51_set_wiper(0), &g_digipot_ready, "DIGIPOT", "set wiper command failed");
#endif
}

/**
 * @brief Complete the autonomous enable sequence.
 *
 * Releases stepper brakes, enables motor drivers (MO=1), configures
 * and starts PT interpolation mode, energizes the DPDT relay (both
 * throttle source switching and pedal bypass), and enables the
 * digital potentiometer autonomous mode.  If any stepper enable or
 * PT setup fails, rolls back all changes and returns without completing.
 */
void execute_complete_enable()
{
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
	ESP_LOGI(TAG, "Completing autonomous enable sequence");
#endif
	esp_err_t steer_err = ESP_OK;
	esp_err_t brake_err = ESP_OK;

#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
	steer_err = stepper_motor_uim2852_enable(&g_steering_stepper);
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
	if (steer_err != ESP_OK)
		ESP_LOGI(TAG, "Steering enable failed: %s", esp_err_to_name(steer_err));
#endif
	vTaskDelay(pdMS_TO_TICKS(5));
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
	brake_err = stepper_motor_uim2852_enable(&g_braking_stepper);
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
	if (brake_err != ESP_OK)
		ESP_LOGI(TAG, "Braking enable failed: %s", esp_err_to_name(brake_err));
#endif
#endif

	if (steer_err != ESP_OK || brake_err != ESP_OK)
	{
		ESP_LOGE(TAG, "Stepper enable failed, aborting autonomous enable");
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
		stepper_motor_uim2852_disable(&g_steering_stepper);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
		stepper_motor_uim2852_disable(&g_braking_stepper);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
		handle_runtime_error(relay_dpdt_my5nj_deenergize(), &g_dpdt_relay_ready, "DPDT_RELAY",
		                     "de-energize command failed");
		handle_runtime_error(digipot_mcp41hv51_disable(), &g_digipot_ready, "DIGIPOT", "disable command failed");
#endif
		return;
	}

	// Configure and arm PT FIFO mode.
	// This must happen after MO=1 succeeds and before the control loop
	// begins feeding position commands.
	esp_err_t pt_err = ESP_OK;
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
	pt_err = stepper_motor_uim2852_pt_configure(&g_steering_stepper);
	if (pt_err == ESP_OK)
		pt_err = stepper_motor_uim2852_pt_start(&g_steering_stepper);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
	if (pt_err == ESP_OK)
		pt_err = stepper_motor_uim2852_pt_configure(&g_braking_stepper);
	if (pt_err == ESP_OK)
		pt_err = stepper_motor_uim2852_pt_start(&g_braking_stepper);
#endif
	if (pt_err != ESP_OK)
	{
		ESP_LOGE(TAG, "PT mode setup failed: %s — aborting enable", esp_err_to_name(pt_err));
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
		stepper_motor_uim2852_disable(&g_steering_stepper);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
		stepper_motor_uim2852_disable(&g_braking_stepper);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
		handle_runtime_error(relay_dpdt_my5nj_deenergize(), &g_dpdt_relay_ready, "DPDT_RELAY",
		                     "de-energize command failed");
		handle_runtime_error(digipot_mcp41hv51_disable(), &g_digipot_ready, "DIGIPOT", "disable command failed");
#endif
		return;
	}

#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	// Energize DPDT relay (both poles: throttle source + pedal bypass) and wait for contacts to settle
	handle_runtime_error(relay_dpdt_my5nj_energize(), &g_dpdt_relay_ready, "DPDT_RELAY", "energize command failed");
	vTaskDelay(pdMS_TO_TICKS(50)); // relay contact settle time

	handle_runtime_error(digipot_mcp41hv51_enable_autonomous(), &g_digipot_ready, "DIGIPOT",
	                     "enable autonomous command failed");
#endif
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
	ESP_LOGI(TAG, "AUTONOMOUS MODE ACTIVE");
#endif
}

/**
 * @brief Abort the enable sequence and return actuators to safe state.
 *
 * De-energizes the DPDT relay and disables the digital potentiometer.
 * Called when Safety retreats, the pedal is pressed, or the F/R
 * selector leaves Forward during the enable dwell.
 *
 * @param reason  Why the enable sequence is being aborted
 */
void execute_abort_enable(abort_reason_t reason)
{
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
	ESP_LOGI(TAG, "Enable sequence aborted (reason=%s)", abort_reason_to_string(reason));
#else
	(void)reason;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	handle_runtime_error(relay_dpdt_my5nj_deenergize(), &g_dpdt_relay_ready, "DPDT_RELAY",
	                     "de-energize command failed");
	handle_runtime_error(digipot_mcp41hv51_disable(), &g_digipot_ready, "DIGIPOT", "disable command failed");
#endif
}

#ifndef CONFIG_THROTTLE_TEST_MODE
// ============================================================================
// CAN RX Task
// ============================================================================

/**
 * @brief FreeRTOS task that receives and dispatches inbound CAN frames.
 *
 * Runs at highest priority (7).  Processes Safety heartbeats (0x100),
 * Planner commands (0x111), and extended stepper motor responses.
 * Updates the shared command_snapshot under g_cmd_lock and feeds the
 * heartbeat monitor.  Sleeps 100ms when TWAI is down to avoid
 * starving lower-priority tasks.
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

		if (msg.rtr)
			continue;

#if !defined(CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING) || !defined(CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING)
		// Handle extended frames from UIM2852CA stepper motors.
		// Try braking first — both motors share producer_id=1, so order
		// matters during sequential init (braking inits first).
		if (msg.extd)
		{
			bool matched = false;
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
			if (stepper_motor_uim2852_process_frame(&g_braking_stepper, &msg))
			{
				matched = true;
				if (stepper_motor_uim2852_stall_detected(&g_braking_stepper) ||
				    stepper_motor_uim2852_has_error(&g_braking_stepper))
				{
					ESP_LOGE(TAG_RX, "STEPPER_BRAKING: fault: stall=%d err=%d locked=%d",
					         g_braking_stepper.status.stall_detected, g_braking_stepper.status.error_detected,
					         g_braking_stepper.status.system_locked);
					if (g_braking_stepper.status.error_detected)
						ESP_LOGE(TAG_RX, "STEPPER_BRAKING: last_error: code=0x%02X cw=0x%02X sub=%u",
						         g_braking_stepper.last_error.error_code, g_braking_stepper.last_error.related_cw,
						         g_braking_stepper.last_error.subindex);
					taskENTER_CRITICAL(&g_cmd_lock);
					g_cmd.motor_fault_code = NODE_FAULT_CONTROL_MOTOR_COMM;
					taskEXIT_CRITICAL(&g_cmd_lock);
				}
			}
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
			if (!matched && stepper_motor_uim2852_process_frame(&g_steering_stepper, &msg))
			{
				matched = true;
				if (stepper_motor_uim2852_stall_detected(&g_steering_stepper) ||
				    stepper_motor_uim2852_has_error(&g_steering_stepper))
				{
					ESP_LOGE(TAG_RX, "STEPPER_STEERING: fault: stall=%d err=%d locked=%d",
					         g_steering_stepper.status.stall_detected, g_steering_stepper.status.error_detected,
					         g_steering_stepper.status.system_locked);
					if (g_steering_stepper.status.error_detected)
						ESP_LOGE(TAG_RX, "STEPPER_STEERING: last_error: code=0x%02X cw=0x%02X sub=%u",
						         g_steering_stepper.last_error.error_code, g_steering_stepper.last_error.related_cw,
						         g_steering_stepper.last_error.subindex);
					taskENTER_CRITICAL(&g_cmd_lock);
					g_cmd.motor_fault_code = NODE_FAULT_CONTROL_MOTOR_COMM;
					taskEXIT_CRITICAL(&g_cmd_lock);
				}
			}
#endif
			(void)matched;
			continue;
		}
#else
		if (msg.extd)
			continue;
#endif

		// Process Planner command (0x111)
		if (msg.identifier == CAN_ID_PLANNER_COMMAND)
		{
			planner_command_t cmd;
			if (!can_decode_planner_command(msg.data, msg.data_length_code, &cmd))
			{
				continue;
			}

			int16_t throttle_level = (int16_t)cmd.throttle;

			taskENTER_CRITICAL(&g_cmd_lock);
			g_cmd.last_planner_cmd_tick = xTaskGetTickCount();

			// Stale command detection: track sequence changes
			if (cmd.sequence == g_cmd.planner_cmd_last_seq)
			{
				if (g_cmd.planner_cmd_stale_count < 255)
					g_cmd.planner_cmd_stale_count = (uint8_t)(g_cmd.planner_cmd_stale_count + 1);
			}
			else
			{
				g_cmd.planner_cmd_last_seq = cmd.sequence;
				g_cmd.planner_cmd_stale_count = 0;
			}

			g_cmd.throttle_cmd = throttle_level;
			g_cmd.steering_cmd = ((int32_t)cmd.steering_position - 360) * (3200 * 50) / 360;
			g_cmd.braking_cmd = (int16_t)((3 - (int32_t)cmd.braking_position) * 28000 / 3);
			taskEXIT_CRITICAL(&g_cmd_lock);
		}

		// Process Safety heartbeat (0x100)
		else if (msg.identifier == CAN_ID_SAFETY_HEARTBEAT)
		{
			node_heartbeat_t hb;
			if (!can_decode_heartbeat(msg.data, msg.data_length_code, &hb))
			{
				continue;
			}

			node_state_t new_target = sanitize_safety_target_state(hb.state); // Safety system target
			node_fault_t safety_fault_flags = hb.fault_flags;
			node_stop_t safety_stop_flags = hb.stop_flags;

			taskENTER_CRITICAL(&g_cmd_lock);
			g_cmd.target_state = new_target;
			g_cmd.safety_fault_flags = safety_fault_flags;
			g_cmd.safety_stop_flags = safety_stop_flags;
			taskEXIT_CRITICAL(&g_cmd_lock);

			heartbeat_monitor_update(&g_hb_monitor, g_node_safety, hb.sequence, new_target);
		}
	}
}

// ============================================================================
// Control Task
// ============================================================================

/**
 * @brief FreeRTOS task that runs the control state machine at 50 Hz.
 *
 * Each tick: retries failed components, snapshots CAN RX state,
 * polls driver inputs (pedal ADC, F/R sensor), evaluates stepper
 * liveness, runs the pure control_compute_step() function, executes
 * hardware actions (throttle, steering, braking, relays), and sends
 * an immediate heartbeat on state change.  Runs at priority 6.
 *
 * @param param  Unused (NULL)
 */
void control_task(void *param)
{
	(void)param;
	task_wdt_add_self_or_log("control_task");
	bool wdt_reset_failed = false;

	// Declare loop-state variables before any goto to satisfy C++ scoping rules
#ifdef CONFIG_LOG_CONTROL_STATE_CHANGES
	node_state_t prev_state = 0xFF;
#endif
#ifdef CONFIG_LOG_CONTROL_FAULT_CHANGES
	node_fault_t prev_fault_flags = 0xFF;
#endif
#ifdef CONFIG_LOG_CONTROL_SAFETY_TARGET_CHANGES
	node_state_t prev_target_state = 0xFF;
	node_fault_t prev_safety_fault = 0xFF;
	node_stop_t prev_safety_stop = 0xFF;
#endif
	int32_t last_steering_sent = STEPPER_DEDUP_RESET_STEERING;
	int16_t last_braking_sent = STEPPER_DEDUP_RESET_BRAKING;
	TickType_t next_steering_pt_feed_tick = 0;
	TickType_t next_braking_pt_feed_tick = 0;

	g_throttle_current = 0;

#ifdef CONFIG_LOG_CONTROL_STATE_CHANGES
	prev_state = g_control_state;
#endif
#ifdef CONFIG_LOG_CONTROL_FAULT_CHANGES
	prev_fault_flags = g_fault_flags;
#endif
#ifdef CONFIG_LOG_CONTROL_SAFETY_TARGET_CHANGES
	taskENTER_CRITICAL(&g_cmd_lock);
	prev_target_state = g_cmd.target_state;
	prev_safety_fault = g_cmd.safety_fault_flags;
	prev_safety_stop = g_cmd.safety_stop_flags;
	taskEXIT_CRITICAL(&g_cmd_lock);
#endif

	while (true)
	{
		uint32_t now_ms = get_time_ms();
		TickType_t now_tick = xTaskGetTickCount();

		retry_failed_components();

		// Take a thread-safe snapshot of CAN RX state
		command_snapshot_t cmd_local;
		taskENTER_CRITICAL(&g_cmd_lock);
		cmd_local = g_cmd;
		// Clear one-shot motor fault flag after reading
		g_cmd.motor_fault_code = NODE_FAULT_NONE;
		taskEXIT_CRITICAL(&g_cmd_lock);

		if (cmd_local.motor_fault_code == NODE_FAULT_CONTROL_MOTOR_COMM)
		{
			mark_component_lost(&g_stepper_steering_ready, "STEPPER_STEERING", "runtime communication fault");
			mark_component_lost(&g_stepper_braking_ready, "STEPPER_BRAKING", "runtime communication fault");
		}

		update_driver_inputs(now_ms);
		fr_state_t fr_state = fr_state_debounced();

		// Apply test bypasses
#ifdef CONFIG_BYPASS_INPUT_FR_SENSOR
		fr_state = FR_STATE_FORWARD;
#endif
#ifdef CONFIG_BYPASS_SAFETY_TARGET_MIRROR
		cmd_local.target_state = NODE_STATE_ACTIVE;
#endif
#ifdef CONFIG_BYPASS_SAFETY_ESTOP_MIRROR
		cmd_local.safety_fault_flags = NODE_FAULT_NONE;
		cmd_local.safety_stop_flags = NODE_STOP_NONE;
#endif
#ifdef CONFIG_BYPASS_PLANNER_COMMAND_INPUTS
		cmd_local.throttle_cmd = 0;
		cmd_local.steering_cmd = 0;
		cmd_local.braking_cmd = 0;
#endif

		// Check Safety heartbeat timeout — retreat to READY if Safety is gone.
		heartbeat_monitor_check_timeouts(&g_hb_monitor);
		bool safety_alive = heartbeat_monitor_is_alive(&g_hb_monitor, g_node_safety);
#ifdef CONFIG_BYPASS_SAFETY_LIVENESS_CHECKS
		safety_alive = true;
#endif
		if (!safety_alive)
		{
			cmd_local.target_state = NODE_STATE_NOT_READY; // retreat to safe state
		}

		// Check Planner command freshness - zero throttle if stale
#ifndef CONFIG_BYPASS_PLANNER_COMMAND_STALE_CHECKS
		TickType_t planner_age = xTaskGetTickCount() - cmd_local.last_planner_cmd_tick;
		bool planner_cmd_stale = false;

		if (g_control_state == NODE_STATE_ACTIVE && cmd_local.last_planner_cmd_tick > 0)
		{
			// Timeout: no command received for PLANNER_CMD_TIMEOUT
			if (planner_age > PLANNER_CMD_TIMEOUT)
			{
				planner_cmd_stale = true;
			}
			// Stale sequence: same sequence seen PLANNER_CMD_STALE_COUNT times
			// AND command is also past half the timeout window. This prevents
			// false stale detection when Planner legitimately retransmits the
			// same command rapidly within the timeout.
			else if (cmd_local.planner_cmd_stale_count >= PLANNER_CMD_STALE_COUNT &&
			         planner_age > (PLANNER_CMD_TIMEOUT / 2))
			{
				planner_cmd_stale = true;
			}
		}

		if (planner_cmd_stale)
		{
			cmd_local.throttle_cmd = 0;
			// Keep last steering/braking (don't jerk)
		}
#endif

		// Build inputs for pure state machine step
		bool pedal_pressed_now = pedal_pressed();
		bool pedal_rearmed_now = pedal_rearmed();
#ifdef CONFIG_BYPASS_INPUT_PEDAL_ADC
		pedal_pressed_now = false;
		pedal_rearmed_now = true;
#endif

		// Compute stepper position error (only meaningful when motor is not
		// actively moving — check in-position/stopped AND error > threshold).
		bool steering_pos_err = false;
		bool braking_pos_err = false;
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
		if (g_stepper_steering_ready && !stepper_motor_uim2852_is_motion_in_progress(&g_steering_stepper))
		{
			steering_pos_err =
				(stepper_motor_uim2852_position_error(&g_steering_stepper) > STEPPER_POSITION_ERROR_THRESHOLD);
		}
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
		if (g_stepper_braking_ready && !stepper_motor_uim2852_is_motion_in_progress(&g_braking_stepper))
		{
			braking_pos_err =
				(stepper_motor_uim2852_position_error(&g_braking_stepper) > STEPPER_POSITION_ERROR_THRESHOLD);
		}
#endif

		control_inputs_t step_in = {
			.target_state = cmd_local.target_state,
			.throttle_cmd = cmd_local.throttle_cmd,
			.steering_cmd = cmd_local.steering_cmd,
			.braking_cmd = cmd_local.braking_cmd,
			.motor_fault_code = cmd_local.motor_fault_code,
			.stop_flags = g_stop_flags,
			.fr_state = fr_state,
			.pedal_pressed = pedal_pressed_now,
			.pedal_rearmed = pedal_rearmed_now,
			.steering_position_error = steering_pos_err,
			.braking_position_error = braking_pos_err,
			.now_ms = now_ms,
			.boot_start_ms = g_boot_start_ms,
			.init_dwell_ms = INIT_DWELL_MS,
			.enable_start_ms = g_enable_start_ms,
			.enable_sequence_ms = ENABLE_SEQUENCE_MS,
			.enable_work_done = g_enable_work_done,
			.throttle_current = g_throttle_current,
			.last_throttle_change_ms = g_last_throttle_change_ms,
			.throttle_slew_interval_ms = THROTTLE_SLEW_INTERVAL_MS,
			.throttle_slew_step = THROTTLE_SLEW_STEP,
			.throttle_min = THROTTLE_LEVEL_MIN,
			.throttle_max = THROTTLE_LEVEL_MAX,
			.last_steering_sent = last_steering_sent,
			.last_braking_sent = last_braking_sent,
			.steering_min = STEERING_POSITION_MIN,
			.steering_max = STEERING_POSITION_MAX,
			.braking_min = BRAKING_POSITION_MIN,
			.braking_max = BRAKING_POSITION_MAX,
		};

		// Compute next state (pure function — no side effects)
		control_step_result_t step = control_compute_step(g_control_state, g_fault_flags, &step_in);

#ifdef CONFIG_LOG_CONTROL_STATE_TICK
		ESP_LOGI(
			TAG,
			"tick state=%s target=%s safety_fault=%s safety_stop=%s fr=%d pedal=%d/%d thr=%d/%d actions=0x%02X -> %s",
			node_state_to_string(g_control_state), node_state_to_string(cmd_local.target_state),
			node_fault_to_string(cmd_local.safety_fault_flags), node_stop_to_string(cmd_local.safety_stop_flags),
			fr_state, pedal_pressed_now, pedal_rearmed_now, g_throttle_current, cmd_local.throttle_cmd, step.actions,
			node_state_to_string(step.new_state));
#endif

		// Log precondition failures while not autonomy-ready (edge-triggered)
#ifdef CONFIG_LOG_CONTROL_PRECONDITION_BLOCKED
		{
			static precondition_fail_t prev_precondition_fail = PRECONDITION_OK;
			if (step.precondition_fail != PRECONDITION_OK && step.precondition_fail != prev_precondition_fail)
			{
				ESP_LOGW(TAG, "Enable blocked: fr=%s pedal_pressed=%d pedal_rearmed=%d fault=%s",
				         fr_state_to_string(fr_state), pedal_pressed_now, pedal_rearmed_now,
				         node_fault_to_string(g_fault_flags));
			}
			else if (step.precondition_fail == PRECONDITION_OK && prev_precondition_fail != PRECONDITION_OK)
			{
				ESP_LOGI(TAG, "Enable preconditions cleared");
			}
			prev_precondition_fail = step.precondition_fail;
		}
#endif

		// Execute hardware actions indicated by the step result
		if (step.actions & CONTROL_ACTION_TRIGGER_OVERRIDE)
		{
			execute_trigger_override(step.override_reason);
		}
		if (step.actions & CONTROL_ACTION_DISABLE_AUTONOMY)
		{
			execute_disable_autonomy(step.disable_reason, cmd_local.safety_fault_flags, cmd_local.safety_stop_flags,
			                         step.new_fault_flags);
		}
		if (step.actions & CONTROL_ACTION_START_ENABLE)
		{
			execute_start_enable();
		}
		if (step.actions & CONTROL_ACTION_COMPLETE_ENABLE)
		{
			bool steppers_ready_for_enable = true;
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
			steppers_ready_for_enable = steppers_ready_for_enable && g_stepper_steering_ready;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
			steppers_ready_for_enable = steppers_ready_for_enable && g_stepper_braking_ready;
#endif

			if (!steppers_ready_for_enable)
			{
				// Hold ENABLE and keep relay power on so stepper retries can recover.
				// Do not emit FAULT here; otherwise Safety would drop relay power and
				// stepper init would deadlock waiting for power.
				step.new_state = NODE_STATE_ENABLE;
				step.new_fault_flags = NODE_FAULT_NONE;
				step.enable_work_done = false; // retry COMPLETE_ENABLE on next tick
				step.status_flags = (node_status_flags_t)(step.status_flags & ~NODE_STATUS_FLAG_ENABLE_COMPLETE);
			}
			else
			{
				execute_complete_enable();
#if !defined(CONFIG_BYPASS_ACTUATOR_THROTTLE)
				// If complete_enable failed, digipot remains non-autonomous — keep retrying.
				if (!digipot_mcp41hv51_is_autonomous())
				{
					step.new_state = NODE_STATE_ENABLE;
					step.new_fault_flags = NODE_FAULT_NONE;
					step.enable_work_done = false; // retry COMPLETE_ENABLE on next tick
					step.status_flags = (node_status_flags_t)(step.status_flags & ~NODE_STATUS_FLAG_ENABLE_COMPLETE);
				}
#endif
			}
		}
		if (step.actions & CONTROL_ACTION_ABORT_ENABLE)
		{
			execute_abort_enable(step.abort_reason);
		}
		if (step.actions & CONTROL_ACTION_APPLY_THROTTLE)
		{
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
			handle_runtime_error(digipot_mcp41hv51_set_wiper((uint8_t)step.throttle_level), &g_digipot_ready, "DIGIPOT",
			                     "set wiper command failed");
#endif
#ifdef CONFIG_LOG_CONTROL_THROTTLE_CHANGES
			{
				static int16_t prev_level = -1;
				if (step.throttle_level != prev_level)
				{
					ESP_LOGI(TAG, "Throttle level: %d -> %d", prev_level, step.throttle_level);
					prev_level = step.throttle_level;
				}
			}
#endif
		}

#ifdef CONFIG_LOG_CONTROL_THROTTLE_TICK
		ESP_LOGI(TAG, "Throttle tick: current=%d target=%d", step.throttle_level, cmd_local.throttle_cmd);
#endif

		// Feed PT rows on the motor's 100 ms cadence using the latest clamped
		// planner-derived target. If Planner skips a cycle, we repeat the last
		// target so the FIFO does not run dry.
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
		if (g_stepper_steering_ready && step.new_state == NODE_STATE_ACTIVE)
		{
			bool fed_frame = false;
			esp_err_t err = feed_pt_stream_if_due(&g_steering_stepper, step.steering_position, now_tick,
			                                      &next_steering_pt_feed_tick, &fed_frame);
			if (err != ESP_OK && step.send_steering)
			{
				step.new_last_steering = last_steering_sent; // keep old value on failure
			}
#ifdef CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX
			if (err != ESP_OK)
			{
				ESP_LOGI(TAG_TX, "Steering PT feed failed: %s", esp_err_to_name(err));
			}
			else if (fed_frame && step.send_steering)
			{
				ESP_LOGI(TAG_TX, "Steering -> %d", step.steering_position);
			}
#endif
			if (fed_frame || err != ESP_OK)
				track_can_tx(err);
		}
		else
		{
			next_steering_pt_feed_tick = 0;
		}
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
		if (g_stepper_braking_ready && step.new_state == NODE_STATE_ACTIVE)
		{
			bool fed_frame = false;
			esp_err_t err = feed_pt_stream_if_due(&g_braking_stepper, step.braking_position, now_tick,
			                                      &next_braking_pt_feed_tick, &fed_frame);
			if (err != ESP_OK && step.send_braking)
			{
				step.new_last_braking = last_braking_sent; // keep old value on failure
			}
#ifdef CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX
			if (err != ESP_OK)
			{
				ESP_LOGI(TAG_TX, "Braking PT feed failed: %s", esp_err_to_name(err));
			}
			else if (fed_frame && step.send_braking)
			{
				ESP_LOGI(TAG_TX, "Braking -> %d", step.braking_position);
			}
#endif
			if (fed_frame || err != ESP_OK)
				track_can_tx(err);
		}
		else
		{
			next_braking_pt_feed_tick = 0;
		}
#endif

		// Detect fault entry (before overwriting g_fault_flags)
		if (step.new_fault_flags != NODE_FAULT_NONE && step.new_fault_flags != g_fault_flags)
		{
			const char *detail = fault_detail_string(step.new_fault_flags, fr_state, true);
			if (detail)
			{
				ESP_LOGE(TAG, "Fault asserted: %s (detail=%s)", node_fault_to_string(step.new_fault_flags), detail);
			}
			else
			{
				ESP_LOGE(TAG, "Fault asserted: %s", node_fault_to_string(step.new_fault_flags));
			}
		}

		// Detect state change for immediate heartbeat send
		node_state_t old_state = NODE_STATE_INIT;
		node_state_t new_state = NODE_STATE_INIT;

		// Apply state updates as an atomic group for heartbeat snapshots.
		node_status_flags_t status_flags = step.status_flags;
		taskENTER_CRITICAL(&g_hb_seq_lock);
		old_state = g_control_state;
		g_control_state = step.new_state;
		g_fault_flags = step.new_fault_flags;
		g_stop_flags = step.new_stop_flags;
		g_heartbeat_status_flags = status_flags;
		new_state = g_control_state;
		taskEXIT_CRITICAL(&g_hb_seq_lock);

		g_throttle_current = step.throttle_level;
		g_last_throttle_change_ms = step.throttle_change_ms;
		g_enable_start_ms = step.enable_start_ms;
		g_enable_work_done = step.enable_work_done;
		last_steering_sent = step.new_last_steering;
		last_braking_sent = step.new_last_braking;

		if (step.new_fault_flags == NODE_FAULT_CONTROL_THROTTLE_INIT && g_digipot_ready)
		{
			mark_component_lost(&g_digipot_ready, "DIGIPOT", "runtime init fault");
		}
		if (step.new_fault_flags == NODE_FAULT_CONTROL_RELAY_INIT && g_dpdt_relay_ready)
		{
			mark_component_lost(&g_dpdt_relay_ready, "DPDT_RELAY", "runtime init fault");
		}
		if (step.new_fault_flags == NODE_FAULT_CONTROL_SENSOR_INVALID && g_fr_inputs_ready)
		{
			char fr_detail[64];
			snprintf(fr_detail, sizeof(fr_detail), "runtime sensor invalid (fr=%s)", fr_state_to_string(fr_state));
			mark_component_lost(&g_fr_inputs_ready, "FR_INPUT", fr_detail);
		}
		if (step.new_fault_flags == NODE_FAULT_CONTROL_MOTOR_COMM)
		{
			mark_component_lost(&g_stepper_steering_ready, "STEPPER_STEERING", "runtime motor communication fault");
			mark_component_lost(&g_stepper_braking_ready, "STEPPER_BRAKING", "runtime motor communication fault");
		}

		if (!all_required_components_ready())
		{
			taskENTER_CRITICAL(&g_hb_seq_lock);
			g_control_state = NODE_STATE_NOT_READY;
			g_fault_flags = primary_fault_from_component_health();
			g_heartbeat_status_flags = 0;
			new_state = g_control_state;
			taskEXIT_CRITICAL(&g_hb_seq_lock);
		}
		else if (g_fault_flags == NODE_FAULT_CONTROL_THROTTLE_INIT || g_fault_flags == NODE_FAULT_CONTROL_RELAY_INIT)
		{
			taskENTER_CRITICAL(&g_hb_seq_lock);
			g_fault_flags = NODE_FAULT_NONE;
			g_heartbeat_status_flags = 0;
			new_state = g_control_state;
			taskEXIT_CRITICAL(&g_hb_seq_lock);
		}

		// Send immediate heartbeat on state change
		if (new_state != old_state)
		{
			send_control_heartbeat();
		}

		// Log control-local transitions using local snapshots (not volatile reads)
#ifdef CONFIG_LOG_CONTROL_STATE_CHANGES
		if (new_state != prev_state)
		{
			const char *reason = "none";
			if (step.actions & CONTROL_ACTION_TRIGGER_OVERRIDE)
				reason = override_reason_to_string(step.override_reason);
			else if (step.actions & CONTROL_ACTION_DISABLE_AUTONOMY)
				reason = disable_reason_to_string(step.disable_reason);
			else if (step.actions & CONTROL_ACTION_START_ENABLE)
				reason = "start_enable";
			else if (step.actions & CONTROL_ACTION_COMPLETE_ENABLE)
				reason = "enable_complete";
			else if (step.actions & CONTROL_ACTION_ABORT_ENABLE)
				reason = "abort_enable";

			ESP_LOGI(TAG, "State: %s -> %s (reason=%s)", node_state_to_string(prev_state),
			         node_state_to_string(new_state), reason);
		}
		prev_state = new_state;
#endif

#ifdef CONFIG_LOG_CONTROL_FAULT_CHANGES
		node_fault_t new_fault = step.new_fault_flags;
		if (new_fault != prev_fault_flags)
		{
			const char *detail = fault_detail_string(new_fault, fr_state, true);
			if (detail)
			{
				ESP_LOGI(TAG, "Fault: %s -> %s (detail=%s)", node_fault_to_string(prev_fault_flags),
				         node_fault_to_string(new_fault), detail);
			}
			else
			{
				ESP_LOGI(TAG, "Fault: %s -> %s", node_fault_to_string(prev_fault_flags),
				         node_fault_to_string(new_fault));
			}
		}
		prev_fault_flags = new_fault;
#endif

		// Log Safety mirrored state/fault changes separately
#ifdef CONFIG_LOG_CONTROL_SAFETY_TARGET_CHANGES
		if (cmd_local.target_state != prev_target_state)
		{
			ESP_LOGI(TAG, "Safety target: %s -> %s", node_state_to_string(prev_target_state),
			         node_state_to_string(cmd_local.target_state));
		}
		if (cmd_local.safety_fault_flags != prev_safety_fault)
		{
			ESP_LOGI(TAG, "Safety fault: %s -> %s", node_fault_to_string(prev_safety_fault),
			         node_fault_to_string(cmd_local.safety_fault_flags));
		}
		if (cmd_local.safety_stop_flags != prev_safety_stop)
		{
			ESP_LOGI(TAG, "Safety stop: %s -> %s", node_stop_to_string(prev_safety_stop),
			         node_stop_to_string(cmd_local.safety_stop_flags));
		}

		prev_target_state = cmd_local.target_state;
		prev_safety_fault = cmd_local.safety_fault_flags;
		prev_safety_stop = cmd_local.safety_stop_flags;
#endif

		task_wdt_reset_or_log("control_task", &wdt_reset_failed);
		vTaskDelay(CONTROL_LOOP_INTERVAL);
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
 * color to reflect the current control state and transmits the
 * Control heartbeat CAN frame.
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

		led_ws2812_set_fault_overlay(g_fault_flags != NODE_FAULT_NONE);
		led_ws2812_set_state(g_control_state);

		// Send heartbeat (periodic 100ms)
		send_control_heartbeat();

		vTaskDelay(HEARTBEAT_SEND_INTERVAL);
	}
}
#endif // !CONFIG_THROTTLE_TEST_MODE

// ============================================================================
// Throttle Test Task (CONFIG_THROTTLE_TEST_MODE)
// ============================================================================

#ifdef CONFIG_THROTTLE_TEST_MODE
/**
 * @brief Standalone throttle test task — serial-controlled throttle levels.
 *
 * Replaces the normal CAN-based control loop when CONFIG_THROTTLE_TEST_MODE
 * is enabled.  Reads wiper values 0-255 from USB serial (type digits, then
 * space/enter to commit) and drives the MCP41HV51 digipot / MY5NJ relay
 * hardware directly.  No CAN, steppers, or Safety/Planner dependencies.
 *
 * Safety behavior:
 *   - Requires F/R in FORWARD to arm.
 *   - Immediately disables all actuators if F/R leaves FORWARD or pedal
 *     is pressed above threshold.
 *   - Auto-recovers when conditions clear.
 *
 * @param param  Unused (NULL)
 */
void throttle_test_task(void *param)
{
	(void)param;
	task_wdt_add_self_or_log("throttle_test");
	bool wdt_reset_failed = false;

	static const char *TAG_TEST = "THROTTLE_TEST";

	esp_err_t console_err = serial_input_init();
	if (console_err != ESP_OK)
		ESP_LOGW(TAG_TEST, "Serial console init failed: %s (serial input may not work)", esp_err_to_name(console_err));

	ESP_LOGI(TAG_TEST, "========================================");
	ESP_LOGI(TAG_TEST, "  THROTTLE TEST MODE");
	ESP_LOGI(TAG_TEST, "  0-7: set throttle wiper (mapped)");
	ESP_LOGI(TAG_TEST, "  r:   toggle DPDT relay (MY5NJ)");
	ESP_LOGI(TAG_TEST, "  q:   quit (shutdown all)");
	ESP_LOGI(TAG_TEST, "  pedal: manual override (auto re-arms)");
	ESP_LOGI(TAG_TEST, "========================================");

	// ----------------------------------------------------------------
	// Arm immediately.  The DPDT relay energizes (both throttle source
	// switch and pedal bypass) and the throttle system is ready for
	// serial commands.  The cart will only move if F/R is physically
	// in FORWARD — the anti-arcing microswitch in the solenoid circuit
	// enforces this at the hardware level.
	// ----------------------------------------------------------------
	esp_err_t err = digipot_mcp41hv51_set_wiper(0);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG_TEST, "Digipot set wiper 0 failed: %s — cannot arm", esp_err_to_name(err));
		led_ws2812_set_fault_overlay(true);
		led_ws2812_set_state(NODE_STATE_NOT_READY);
		while (true)
		{
			task_wdt_reset_or_log("throttle_test", &wdt_reset_failed);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	err = relay_dpdt_my5nj_energize();
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG_TEST, "DPDT relay failed: %s — cannot arm", esp_err_to_name(err));
		(void)digipot_mcp41hv51_disable();
		led_ws2812_set_fault_overlay(true);
		led_ws2812_set_state(NODE_STATE_NOT_READY);
		while (true)
		{
			task_wdt_reset_or_log("throttle_test", &wdt_reset_failed);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	vTaskDelay(pdMS_TO_TICKS(50)); // relay contact settle

	err = digipot_mcp41hv51_enable_autonomous();
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG_TEST, "Digipot enable failed: %s — cannot arm", esp_err_to_name(err));
		(void)relay_dpdt_my5nj_deenergize();
		(void)digipot_mcp41hv51_disable();
		led_ws2812_set_fault_overlay(true);
		led_ws2812_set_state(NODE_STATE_NOT_READY);
		while (true)
		{
			task_wdt_reset_or_log("throttle_test", &wdt_reset_failed);
			vTaskDelay(pdMS_TO_TICKS(1000));
		}
	}

	ESP_LOGI(TAG_TEST, "Throttle system ready — DPDT relay ON, digipot autonomous");
	ESP_LOGI(TAG_TEST, "Type 0-255 + space/enter to set wiper. r=toggle relay, q=quit");
	ESP_LOGI(TAG_TEST, "F/R software enforced: REVERSE disables throttle, NEUTRAL zeros level");

	// ----------------------------------------------------------------
	// State machine
	//
	// ARMED:    DPDT relay ON, digipot active.
	//           Type 0-255 + space/enter to set wiper position.
	//           Pedal press or F/R REVERSE → OVERRIDE.
	//           F/R NEUTRAL → zero throttle (stay ARMED).
	// OVERRIDE: DPDT relay OFF (manual pedal drives).
	//           Pedal release + F/R not REVERSE → back to ARMED at wiper 0.
	// SHUTDOWN: everything OFF, idle forever.
	// ----------------------------------------------------------------
	enum test_state_t : uint8_t
	{
		TEST_ARMED,
		TEST_OVERRIDE,
		TEST_SHUTDOWN,
	};

	uint8_t current_wiper = 0;
	bool relay_on = true; // MY5NJ DPDT relay (GPIO 10) — energized on boot
	uint32_t last_status_log_ms = 0;

	serial_input_accum_t wiper_input = SERIAL_INPUT_ACCUM_INIT(255);

	// Post-bypass F/R gate: DPDT relay is energized so the full truth
	// table is readable.  Wait for debounce to settle, then start in
	// OVERRIDE if the cart is in REVERSE (loop will recover when cleared).
	vTaskDelay(pdMS_TO_TICKS(FR_PC817_DEBOUNCE_MS + 5));
	update_driver_inputs(get_time_ms());
	fr_state_t fr_arm = fr_state_debounced();
	test_state_t state;
	if (fr_arm == FR_STATE_REVERSE || fr_arm == FR_STATE_INVALID)
	{
		ESP_LOGW(TAG_TEST, "F/R is %s at arm — entering override until not in reverse", fr_state_to_string(fr_arm));
		(void)digipot_mcp41hv51_disable();
		(void)relay_dpdt_my5nj_deenergize();
		relay_on = false;
		current_wiper = 0;
		state = TEST_OVERRIDE;
		led_ws2812_set_fault_overlay(false);
		led_ws2812_set_state(NODE_STATE_NOT_READY);
	}
	else
	{
		ESP_LOGI(TAG_TEST, "Throttle ARMED at level 0 (F/R=%s)", fr_state_to_string(fr_arm));
		state = TEST_ARMED;
		led_ws2812_set_fault_overlay(false);
		led_ws2812_set_state(NODE_STATE_READY);
	}

	while (true)
	{
		task_wdt_reset_or_log("throttle_test", &wdt_reset_failed);
		uint32_t now_ms = get_time_ms();

		update_driver_inputs(now_ms);

		fr_state_t fr_now = fr_state_debounced();
#ifdef CONFIG_BYPASS_INPUT_FR_SENSOR
		fr_now = FR_STATE_FORWARD;
#endif
		bool fr_is_reverse = (fr_now == FR_STATE_REVERSE || fr_now == FR_STATE_INVALID);

		bool pedal_is_pressed = pedal_pressed();
#ifdef CONFIG_BYPASS_INPUT_PEDAL_ADC
		pedal_is_pressed = false;
#endif

		bool pedal_is_rearmed = pedal_rearmed();
#ifdef CONFIG_BYPASS_INPUT_PEDAL_ADC
		pedal_is_rearmed = true;
#endif

		// Read serial input (non-blocking)
		int ch = serial_input_read_char();

		// ---- Quit: immediate shutdown from any state ----
		if (ch == 'q' || ch == 'Q')
		{
			ESP_LOGI(TAG_TEST, "Quit requested");
			(void)digipot_mcp41hv51_disable();
			(void)digipot_mcp41hv51_emergency_stop();
			(void)relay_dpdt_my5nj_deenergize();
			led_ws2812_set_state(NODE_STATE_INIT);
			ESP_LOGI(TAG_TEST, "Shutdown complete — idle");
			serial_input_accum_reset(&wiper_input);
			state = TEST_SHUTDOWN;
		}

		// ---- State machine ----
		switch (state)
		{
		// ............................................................
		// ARMED: DPDT relay ON, digipot active.
		// Serial commands 0-7 control throttle level.
		// Pedal press or F/R REVERSE → OVERRIDE (manual control).
		// F/R NEUTRAL → zero throttle (stay ARMED).
		// ............................................................
		case TEST_ARMED:
		{
			// Diagnostic: log pedal ADC + wiper + FR every 2 seconds
			if (now_ms - last_status_log_ms >= 2000)
			{
				ESP_LOGI(TAG_TEST, "Armed: wiper=%u pedal=%u mV (threshold=%u) fr=%s", current_wiper,
				         (unsigned)g_pedal_mv, (unsigned)PEDAL_ADC_THRESHOLD_MV, fr_state_to_string(fr_now));
				last_status_log_ms = now_ms;
			}

			// Pedal press: hand control to driver (manual override)
			if (pedal_is_pressed)
			{
				ESP_LOGI(TAG_TEST, "Pedal pressed (%u mV) — manual override, DPDT relay off", (unsigned)g_pedal_mv);
				(void)digipot_mcp41hv51_disable();
				(void)relay_dpdt_my5nj_deenergize();
				relay_on = false;
				current_wiper = 0;
				serial_input_accum_reset(&wiper_input);
				led_ws2812_set_fault_overlay(false);
				led_ws2812_set_state(NODE_STATE_NOT_READY);
				state = TEST_OVERRIDE;
				break;
			}

			// F/R REVERSE: disable throttle (same as pedal override)
			if (fr_is_reverse)
			{
				ESP_LOGW(TAG_TEST, "F/R is %s — override, DPDT relay off", fr_state_to_string(fr_now));
				(void)digipot_mcp41hv51_disable();
				(void)relay_dpdt_my5nj_deenergize();
				relay_on = false;
				current_wiper = 0;
				serial_input_accum_reset(&wiper_input);
				led_ws2812_set_fault_overlay(false);
				led_ws2812_set_state(NODE_STATE_NOT_READY);
				state = TEST_OVERRIDE;
				break;
			}

			// F/R NEUTRAL: zero throttle but stay ARMED (cart can't
			// drive in neutral, resumes when switched to FORWARD)
			if (fr_now == FR_STATE_NEUTRAL && current_wiper > 0)
			{
				ESP_LOGW(TAG_TEST, "F/R is NEUTRAL — zeroing throttle (was wiper %u)", current_wiper);
				esp_err_t zerr = digipot_mcp41hv51_set_wiper(0);
				if (zerr == ESP_OK)
				{
					current_wiper = 0;
					led_ws2812_set_fault_overlay(false);
					led_ws2812_set_state(NODE_STATE_READY);
				}
			}

			// Handle serial throttle commands: type 0-255 then space/enter to commit.
			if (serial_input_accum_feed(&wiper_input, ch))
			{
				uint8_t wiper = (uint8_t)wiper_input.value;
				serial_input_accum_reset(&wiper_input);
				if (wiper != current_wiper)
				{
					esp_err_t serr = digipot_mcp41hv51_set_wiper(wiper);
					if (serr == ESP_OK)
					{
						current_wiper = wiper;
						ESP_LOGI(TAG_TEST, "Throttle wiper: %u", wiper);
						led_ws2812_set_fault_overlay(false);
						led_ws2812_set_state(wiper > 0 ? NODE_STATE_ACTIVE : NODE_STATE_READY);
					}
					else
					{
						ESP_LOGE(TAG_TEST, "Digipot set wiper %u failed: %s", wiper, esp_err_to_name(serr));
					}
				}
			}
			// Toggle DPDT relay (MY5NJ, GPIO 10)
			else if (ch == 'r' || ch == 'R')
			{
				esp_err_t rerr;
				if (relay_on)
				{
					rerr = relay_dpdt_my5nj_deenergize();
					if (rerr == ESP_OK)
					{
						relay_on = false;
						ESP_LOGI(TAG_TEST, "DPDT relay OFF — manual pedal drives Curtis, pedal switch in circuit");
					}
					else
					{
						ESP_LOGE(TAG_TEST, "DPDT relay deenergize failed: %s", esp_err_to_name(rerr));
					}
				}
				else
				{
					// Reset digipot to 0 before reconnecting to prevent sudden throttle jump
					esp_err_t dp_err = digipot_mcp41hv51_set_wiper(0);
					if (dp_err != ESP_OK)
					{
						ESP_LOGE(TAG_TEST, "Digipot reset to 0 failed: %s — aborting relay ON",
						         esp_err_to_name(dp_err));
					}
					else
					{
						rerr = relay_dpdt_my5nj_energize();
						if (rerr == ESP_OK)
						{
							relay_on = true;
							current_wiper = 0;
							ESP_LOGI(TAG_TEST, "DPDT relay ON — digipot drives Curtis (wiper reset to 0)");
						}
						else
						{
							ESP_LOGE(TAG_TEST, "DPDT relay energize failed: %s", esp_err_to_name(rerr));
						}
					}
				}
			}
			break;
		}

		// ............................................................
		// OVERRIDE: driver has manual control via pedal/potbox.
		// DPDT relay OFF (manual pedal drives, pedal switch in circuit).
		// Digipot disabled.  When pedal is released, rearmed, AND F/R
		// is not in REVERSE, re-enable autonomous at wiper 0.
		// ............................................................
		case TEST_OVERRIDE:
		{
			if (pedal_is_rearmed && !fr_is_reverse)
			{
				ESP_LOGI(TAG_TEST, "Override cleared (pedal rearmed, fr=%s) — re-arming at wiper 0",
				         fr_state_to_string(fr_now));

				esp_err_t rerr = digipot_mcp41hv51_set_wiper(0);
				if (rerr == ESP_OK)
					rerr = relay_dpdt_my5nj_energize();
				if (rerr == ESP_OK)
				{
					vTaskDelay(pdMS_TO_TICKS(50)); // relay contact settle
					rerr = digipot_mcp41hv51_enable_autonomous();
				}

				if (rerr != ESP_OK)
				{
					ESP_LOGE(TAG_TEST, "Re-arm failed: %s — staying in override", esp_err_to_name(rerr));
					// Clean up partial state
					(void)relay_dpdt_my5nj_deenergize();
					(void)digipot_mcp41hv51_disable();
					break;
				}

				current_wiper = 0;
				relay_on = true;
				led_ws2812_set_fault_overlay(false);
				led_ws2812_set_state(NODE_STATE_READY);
				ESP_LOGI(TAG_TEST, "Throttle ARMED at wiper 0 — enter command");
				state = TEST_ARMED;
			}
			else if (now_ms - last_status_log_ms >= 2000)
			{
				ESP_LOGI(TAG_TEST, "Override: manual control (fr=%s, pedal_rearmed=%d — need both clear)",
				         fr_state_to_string(fr_now), pedal_is_rearmed);
				last_status_log_ms = now_ms;
			}
			break;
		}

		// ............................................................
		// SHUTDOWN: everything off, idle forever.
		// ............................................................
		case TEST_SHUTDOWN:
		{
			while (true)
			{
				task_wdt_reset_or_log("throttle_test", &wdt_reset_failed);
				vTaskDelay(pdMS_TO_TICKS(1000));
			}
		}

		} // switch

		vTaskDelay(CONTROL_LOOP_INTERVAL);
	}
}
#endif // CONFIG_THROTTLE_TEST_MODE

// ============================================================================
// Main Task
// ============================================================================

/**
 * @brief Initialization task that configures all peripherals and spawns runtime tasks.
 *
 * Initializes TWAI, LED, heartbeat monitor, and all actuator/sensor
 * components with one startup attempt each.  Starts the CAN RX task
 * first so stepper init queries can receive motor responses.  Logs
 * startup status and active test bypasses, then creates the control
 * and heartbeat tasks before deleting itself.
 *
 * @param param  Unused (NULL)
 */
void main_task(void *param)
{
	(void)param;

	g_control_state = NODE_STATE_INIT;
	g_boot_start_ms = get_time_ms();

#ifdef CONFIG_THROTTLE_TEST_MODE
	// ----------------------------------------------------------------
	// Throttle Test Mode: init only throttle-related hardware, then
	// create the test task.  No CAN, steppers, or Safety/Planner.
	// ----------------------------------------------------------------

	esp_err_t err = led_ws2812_init();
	bool heartbeat_ready = (err == ESP_OK);
	if (heartbeat_ready)
		led_ws2812_set_state(NODE_STATE_INIT);

	g_digipot_ready = (digipot_mcp41hv51_init(&g_digipot_cfg) == ESP_OK);
	g_dpdt_relay_ready = (relay_dpdt_my5nj_init(&g_dpdt_relay_cfg) == ESP_OK);

#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
	g_pedal_input_ready = (init_pedal_input() == ESP_OK);
#else
	g_pedal_input_ready = true;
#endif

#ifndef CONFIG_BYPASS_INPUT_FR_SENSOR
	g_fr_inputs_ready = (init_fr_inputs() == ESP_OK);
#else
	g_fr_inputs_ready = true;
#endif

	ESP_LOGI(TAG_INIT, "--- THROTTLE TEST MODE ---");
	ESP_LOGI(TAG_INIT, "DIGIPOT: %s (sdi=%d sck=%d cs=%d)", g_digipot_ready ? "OK" : "FAILED", g_digipot_cfg.sdi,
	         g_digipot_cfg.sck, g_digipot_cfg.cs);
	ESP_LOGI(TAG_INIT, "DPDT_RELAY: %s (gpio=%d)", g_dpdt_relay_ready ? "OK" : "FAILED", g_dpdt_relay_cfg.gpio);
#ifdef CONFIG_BYPASS_INPUT_PEDAL_ADC
	ESP_LOGW(TAG_INIT, "PEDAL_INPUT: BYPASSED");
#else
	ESP_LOGI(TAG_INIT, "PEDAL_INPUT: %s", g_pedal_input_ready ? "OK" : "FAILED");
#endif
#ifdef CONFIG_BYPASS_INPUT_FR_SENSOR
	ESP_LOGW(TAG_INIT, "FR_INPUT: BYPASSED (forcing FORWARD)");
#else
	ESP_LOGI(TAG_INIT, "FR_INPUT: %s (fwd_gpio=%d rev_gpio=%d)", g_fr_inputs_ready ? "OK" : "FAILED",
	         g_fr_pc817_cfg.forward_gpio, g_fr_pc817_cfg.reverse_gpio);
#endif
	ESP_LOGI(TAG_INIT, "HEARTBEAT_LED: %s", heartbeat_ready ? "OK" : "UNAVAILABLE");
	ESP_LOGI(TAG_INIT, "Skipped: CAN/TWAI, steppers, Safety/Planner");

	if (!g_digipot_ready || !g_dpdt_relay_ready)
	{
		ESP_LOGE(TAG_INIT, "Required throttle hardware init failed — test task will report errors");
		if (heartbeat_ready)
		{
			led_ws2812_set_fault_overlay(true);
			led_ws2812_set_state(NODE_STATE_NOT_READY);
		}
	}

	if (xTaskCreate(throttle_test_task, "throttle_test", CONTROL_TASK_STACK, nullptr, CONTROL_TASK_PRIO, nullptr) !=
	    pdPASS)
	{
		ESP_LOGE(TAG_INIT, "Failed to create throttle test task, restarting");
		esp_restart();
	}

	vTaskDelete(nullptr);

#else // !CONFIG_THROTTLE_TEST_MODE

	// ----------------------------------------------------------------
	// Normal Mode: full system initialization
	// ----------------------------------------------------------------

	// Initialize TWAI once during startup. If it fails, mark FAULT and let
	// background per-component retries handle recovery.
	esp_err_t err = ESP_FAIL;
	err = can_twai_init_default(TWAI_TX_GPIO, TWAI_RX_GPIO);
	g_twai_ready = (err == ESP_OK);

	err = led_ws2812_init();
	bool heartbeat_ready = (err == ESP_OK);
	if (heartbeat_ready)
	{
		led_ws2812_set_state(NODE_STATE_INIT);
	}

	// Initialize heartbeat monitor to detect Safety heartbeat timeout.
	heartbeat_monitor_config_t hb_cfg = {.name = "CONTROL"};
	heartbeat_monitor_init(&g_hb_monitor, &hb_cfg);
	g_node_safety = heartbeat_monitor_register(&g_hb_monitor, "Safety", HEARTBEAT_TIMEOUT_MS);

	// Start CAN RX first so stepper init checks can receive responses.
	if (xTaskCreate(can_rx_task, "can_rx", CAN_RX_TASK_STACK, nullptr, CAN_RX_TASK_PRIO, &g_can_rx_task_handle) !=
	    pdPASS)
	{
		ESP_LOGE(TAG_INIT, "Failed to create CAN RX task, restarting");
		esp_restart();
	}

	// One startup attempt per component; retries happen later while faulted.
#ifndef CONFIG_BYPASS_ACTUATOR_THROTTLE
	g_digipot_ready = (digipot_mcp41hv51_init(&g_digipot_cfg) == ESP_OK);
	g_dpdt_relay_ready = (relay_dpdt_my5nj_init(&g_dpdt_relay_cfg) == ESP_OK);
#else
	g_digipot_ready = true;
	g_dpdt_relay_ready = true;
#endif

#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
	g_pedal_input_ready = (init_pedal_input() == ESP_OK);
#else
	g_pedal_input_ready = true;
#endif

#ifndef CONFIG_BYPASS_INPUT_FR_SENSOR
	g_fr_inputs_ready = (init_fr_inputs() == ESP_OK);
#else
	g_fr_inputs_ready = true;
#endif

	// Init braking first — both motors share producer_id=1, and the RX
	// handler tries braking first, so braking must init before steering
	// to avoid response misrouting.
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
	{
		stepper_motor_uim2852_config_t b_cfg = STEPPER_MOTOR_UIM2852_CONFIG_DEFAULT();
		b_cfg.node_id = UIM2852_NODE_BRAKING;
		esp_err_t err = stepper_motor_uim2852_init(&g_braking_stepper, &b_cfg);
		if (err == ESP_OK)
		{
			(void)stepper_motor_uim2852_disable(&g_braking_stepper); // MO=0 hot-start
			err = stepper_motor_uim2852_configure(&g_braking_stepper);
		}
		if (err == ESP_OK)
			err = stepper_motor_uim2852_set_limits(&g_braking_stepper, BRAKING_POSITION_MIN, BRAKING_POSITION_MAX);
		if (err == ESP_OK)
			err = stepper_motor_uim2852_disable(&g_braking_stepper);
		if (err == ESP_OK)
			(void)stepper_motor_uim2852_query_position(&g_braking_stepper);
		g_stepper_braking_ready = (err == ESP_OK);
	}
#else
	g_stepper_braking_ready = true;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
	{
		stepper_motor_uim2852_config_t s_cfg = STEPPER_MOTOR_UIM2852_CONFIG_DEFAULT();
		s_cfg.node_id = UIM2852_NODE_STEERING;
		esp_err_t err = stepper_motor_uim2852_init(&g_steering_stepper, &s_cfg);
		if (err == ESP_OK)
		{
			(void)stepper_motor_uim2852_disable(&g_steering_stepper); // MO=0 hot-start
			err = stepper_motor_uim2852_configure(&g_steering_stepper);
		}
		if (err == ESP_OK)
			err = stepper_motor_uim2852_set_limits(&g_steering_stepper, STEERING_POSITION_MIN, STEERING_POSITION_MAX);
		if (err == ESP_OK)
			err = stepper_motor_uim2852_disable(&g_steering_stepper);
		if (err == ESP_OK)
			(void)stepper_motor_uim2852_query_position(&g_steering_stepper);
		g_stepper_steering_ready = (err == ESP_OK);
	}
#else
	g_stepper_steering_ready = true;
#endif

	log_startup_device_status(g_twai_ready, g_digipot_ready, g_dpdt_relay_ready, g_pedal_input_ready, g_fr_inputs_ready,
	                          g_stepper_steering_ready, g_stepper_braking_ready, heartbeat_ready);

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
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: ACTUATOR_THROTTLE (digipot/DPDT relay skipped)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: ACTUATOR_STEPPER_STEERING (steering skipped)");
		any_bypass = true;
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
		ESP_LOGW(TAG_INIT, "BYPASS ACTIVE: ACTUATOR_STEPPER_BRAKING (braking skipped)");
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
		const char *detail = fault_detail_string(g_fault_flags, fr_state_debounced(), true);
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
		led_ws2812_set_state(g_control_state);
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
#endif // CONFIG_THROTTLE_TEST_MODE
}
} // namespace

/**
 * @brief ESP-IDF application entry point for the Control ESP32.
 *
 * Creates the main_task on a dedicated stack to perform peripheral
 * initialization and spawn runtime tasks.  Restarts the system if
 * task creation fails.
 */
extern "C" void app_main(void)
{
	if (xTaskCreate(main_task, "main_task", 8192, nullptr, 5, nullptr) != pdPASS)
	{
		ESP_LOGE("CONTROL_INIT", "Failed to create main_task, restarting");
		esp_restart();
	}
}
