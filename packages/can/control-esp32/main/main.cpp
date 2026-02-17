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

#include "can_protocol.hh"
#include "can_twai.hh"
#include "led_ws2812.hh"
#include "multiplexer_dg408djz.hh"
#include "stepper_motor_uim2852.h"
#include "relay_jd2912.hh"
#include "adc_12bitsar.hh"
#include "optocoupler_pc817.hh"
#include "control_logic.h"
#include "heartbeat_monitor.hh"



// Control ESP32 - Autonomous control execution
//
// Receives commands from Planner via CAN and actuates throttle, steering, and braking.
// Safety ESP32 commands target state via heartbeat (0x100).
// State behavior and transitions are documented in control-esp32/README.md.

namespace {

static const char *TAG = "CONTROL";
static const char *TAG_INIT = "CONTROL_INIT";
static const char *TAG_TX __attribute__((unused)) = "CONTROL_TX";
static const char *TAG_RX __attribute__((unused)) = "CONTROL_RX";

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

constexpr TickType_t CAN_RX_TIMEOUT = pdMS_TO_TICKS(10);  // wait per RX poll
constexpr TickType_t CONTROL_LOOP_INTERVAL = pdMS_TO_TICKS(20);  // state/actuation tick (50 Hz)
constexpr TickType_t HEARTBEAT_SEND_INTERVAL = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS);  // protocol cadence
constexpr uint32_t ENABLE_SEQUENCE_MS = 200;  // ENABLING dwell before enable_complete
constexpr uint32_t THROTTLE_SLEW_INTERVAL_MS = 100;  // throttle: max 1 level step per interval
constexpr TickType_t PLANNER_CMD_TIMEOUT = pdMS_TO_TICKS(500);  // stale planner command cutoff
constexpr uint16_t PEDAL_ADC_THRESHOLD_MV = 360;  // pedal override trigger threshold
constexpr uint32_t PEDAL_REARM_MS = 500;  // pedal must stay released this long to re-arm

// Stepper position error threshold (pulses). If the actual encoder position
// diverges from the last commanded target by more than this amount while the
// motor reports in-position/stopped, an external physical override is assumed.
// At 16 microsteps (3200 pulses/rev), 200 pulses ~ 22.5 degrees.
constexpr int32_t STEPPER_POSITION_ERROR_THRESHOLD = 200;

// ============================================================================
// Retry & Fault Constants
// ============================================================================
// Fault tolerance tuning:
// - CAN_TX_CONSEC_FAIL_THRESHOLD filters transient TX errors before recovery.
// - RETRY_INTERVAL_MS paces component re-init attempts (no max attempts).

constexpr uint8_t CAN_TX_CONSEC_FAIL_THRESHOLD = 5;
constexpr uint32_t RETRY_INTERVAL_MS = 500;  // component retry cadence (infinite, no cap)

// ============================================================================
// GPIO Pin Assignments
// ============================================================================

// CAN bus (WAVESHARE SN65HVD230 transceiver)
constexpr gpio_num_t TWAI_TX_GPIO = GPIO_NUM_4;
constexpr gpio_num_t TWAI_RX_GPIO = GPIO_NUM_5;

// DG408 analog multiplexer (throttle level selection)
constexpr gpio_num_t MUX_A0_GPIO = GPIO_NUM_2;
constexpr gpio_num_t MUX_A1_GPIO = GPIO_NUM_3;
constexpr gpio_num_t MUX_A2_GPIO = GPIO_NUM_6;
constexpr gpio_num_t MUX_EN_GPIO = GPIO_NUM_7;

// Relay outputs
constexpr gpio_num_t THROTTLE_RELAY_GPIO = GPIO_NUM_9;   // AEDIKO module
constexpr gpio_num_t ENABLE_MOSFET_GPIO = GPIO_NUM_10;   // IRLZ44N gate for JD-2912

// Pedal ADC (voltage divider from pot wiper)
constexpr adc_channel_t PEDAL_ADC_CHANNEL = ADC_CHANNEL_0;  // GPIO 0

// F/R PC817 channels - detect gear selector contacts
constexpr gpio_num_t FR_FORWARD_GPIO = GPIO_NUM_14;  // forward contact optocoupler
constexpr gpio_num_t FR_REVERSE_GPIO = GPIO_NUM_15;  // reverse contact optocoupler

// Status LED (WS2812)
constexpr gpio_num_t HEARTBEAT_LED_GPIO = GPIO_NUM_8;
constexpr uint8_t LED_LEVEL = 16;
constexpr uint8_t LED_ORANGE_GREEN = 8;

// ============================================================================
// Component Configurations
// ============================================================================

static multiplexer_dg408djz_config_t g_mux_cfg = {
    .a0 = MUX_A0_GPIO,
    .a1 = MUX_A1_GPIO,
    .a2 = MUX_A2_GPIO,
    .en = MUX_EN_GPIO,
    .relay = THROTTLE_RELAY_GPIO,
};

static relay_jd2912_config_t g_relay_cfg = {
    .gpio = ENABLE_MOSFET_GPIO,
    .active_high = true,
};

static adc_12bitsar_config_t g_pedal_adc_cfg = {
    .adc_unit = ADC_UNIT_1,
    .adc_channel = PEDAL_ADC_CHANNEL,
};

static optocoupler_pc817_config_t g_fr_pc817_cfg = {
    .forward_gpio = FR_FORWARD_GPIO,
    .reverse_gpio = FR_REVERSE_GPIO,
};

static led_ws2812_config_t g_heartbeat_led_cfg = {
    .gpio = HEARTBEAT_LED_GPIO,
    .interval_ticks = pdMS_TO_TICKS(500),
    .activity_window_ticks = pdMS_TO_TICKS(250),
    .label = "control",
    .idle_red = 0,
    .idle_green = 16,
    .idle_blue = 0,
    .activity_red = 0,
    .activity_green = 0,
    .activity_blue = 16,
    .error_red = 16,
    .error_green = 0,
    .error_blue = 0,
};

// ============================================================================
// Thread-Safe Shared State (CAN RX task -> Control/Heartbeat tasks)
// ============================================================================

// Snapshot of data written by CAN RX task, read by Control and Heartbeat tasks.
// All access must be protected by g_cmd_lock.
struct command_snapshot_t {
    uint8_t target_state;       // from Safety heartbeat
    uint8_t estop_fault_code;   // from Safety heartbeat (NODE_FAULT_ESTOP_*)
    int8_t throttle_target;
    int16_t steering_cmd;
    int16_t braking_cmd;
    uint8_t motor_fault_code;   // set by CAN RX on stepper error
};

static portMUX_TYPE g_cmd_lock = portMUX_INITIALIZER_UNLOCKED;
static command_snapshot_t g_cmd = {};  // protected by g_cmd_lock

// Timestamp of last Planner command received (set by CAN RX task, read by control_task)
static volatile TickType_t g_last_planner_cmd_tick = 0;

// Planner command stale detection (set by CAN RX, read by control_task)
static volatile uint8_t g_planner_cmd_last_seq = 0;
static volatile uint8_t g_planner_cmd_stale_count = 0;

// ============================================================================
// Local State (Control task only - no lock needed)
// ============================================================================

// State machine
static volatile uint8_t g_control_state = NODE_STATE_INIT;
static volatile uint8_t g_fault_code = NODE_FAULT_NONE;
static volatile uint8_t g_heartbeat_flags = 0;  // HEARTBEAT_FLAG_* for next heartbeat

// Throttle
static volatile int8_t g_throttle_current = 0;

// Driver override inputs (maintained in main, not a separate policy component)
static bool g_pedal_input_ready = false;
static bool g_fr_inputs_ready = false;
static uint16_t g_pedal_mv = 0;
static bool g_pedal_was_above = false;
static uint32_t g_pedal_below_threshold_since = 0;

// Timing state
static uint32_t g_enable_start_ms = 0;
static uint32_t g_last_throttle_change_ms = 0;
static bool g_enable_work_done = false;

// Heartbeat
static volatile uint8_t g_heartbeat_seq = 0;
[[maybe_unused]] static bool g_hb_tx_failing = false;  // edge-trigger for heartbeat TX failure logging

// Spinlock for heartbeat sequence (shared by control_task immediate send + heartbeat_task)
static portMUX_TYPE g_hb_seq_lock = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// Recovery State
// ============================================================================

struct recovery_state_t {
    uint8_t can_tx_fail_count;        // consecutive CAN TX failures
};

static recovery_state_t g_recovery = {};
static bool g_can_recovery_in_progress = false;

// Component health tracking for startup checks and persistent retries.
static bool g_twai_ready = false;
static bool g_mux_ready = false;
static bool g_relay_ready = false;
static bool g_stepper_steering_ready = false;
static bool g_stepper_braking_ready = false;

// CAN RX task handle — used to verify quiesce during recovery.
static TaskHandle_t g_can_rx_task_handle = nullptr;

// Spinlock for CAN TX tracking / recovery (used by control_task + heartbeat_task)
static portMUX_TYPE g_can_tx_lock = portMUX_INITIALIZER_UNLOCKED;

// Retry pacing: last timestamp (ms) when retry_failed_components ran.
// Retries are infinite (no cap) but paced at RETRY_INTERVAL_MS to avoid
// hammering init calls on every control loop tick.
static uint32_t g_last_retry_ms = 0;

// Heartbeat monitor: detect Safety heartbeat timeout on Control.
static heartbeat_monitor_t g_hb_monitor;
static int g_node_safety = -1;

// ============================================================================
// Stepper Motor Instances (UIM2852CA)
// ============================================================================

[[maybe_unused]] static stepper_motor_uim2852_t g_steering_stepper = {};
[[maybe_unused]] static stepper_motor_uim2852_t g_braking_stepper = {};

static uint32_t get_time_ms() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static uint8_t sanitize_safety_target_state(uint8_t raw_state) {
    switch (raw_state) {
        case NODE_STATE_READY:
        case NODE_STATE_ENABLING:
        case NODE_STATE_ACTIVE:
            return raw_state;
        default:
            // Safety target commands only use READY/ENABLING/ACTIVE.
            // Treat all other values as READY (safe retreat).
            return NODE_STATE_READY;
    }
}

static void mark_component_lost(bool *ready, const char *name, const char *detail);

// ============================================================================
// Driver Input Helpers (pedal ADC + F/R PC817)
// ============================================================================

[[maybe_unused]] static esp_err_t init_pedal_input(void) {
    adc_12bitsar_deinit();
    g_pedal_input_ready = false;

    esp_err_t err = adc_12bitsar_init(&g_pedal_adc_cfg);
    if (err != ESP_OK) return err;

    err = adc_12bitsar_read_mv_checked(&g_pedal_mv);
    if (err != ESP_OK) return err;
    g_pedal_was_above = (g_pedal_mv >= PEDAL_ADC_THRESHOLD_MV);
    g_pedal_below_threshold_since = 0;

    g_pedal_input_ready = true;
    return ESP_OK;
}

[[maybe_unused]] static esp_err_t init_fr_inputs(void) {
    g_fr_inputs_ready = false;

    esp_err_t err = optocoupler_pc817_init(&g_fr_pc817_cfg);
    if (err != ESP_OK) return err;

#ifndef CONFIG_BYPASS_INPUT_FR_SENSOR
    fr_state_t fr_initial = optocoupler_pc817_get_state();
    if (fr_initial == FR_STATE_NEUTRAL || fr_initial == FR_STATE_INVALID) {
        return ESP_ERR_INVALID_STATE;
    }
#endif

    g_fr_inputs_ready = true;
    return ESP_OK;
}

static void update_driver_inputs(uint32_t now_ms) {
#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
    if (g_pedal_input_ready) {
        uint16_t pedal_mv = 0;
        esp_err_t pedal_err = adc_12bitsar_read_mv_checked(&pedal_mv);
        if (pedal_err != ESP_OK) {
            mark_component_lost(&g_pedal_input_ready, "PEDAL_INPUT", "runtime adc read failure");
        } else {
            g_pedal_mv = pedal_mv;

    #ifdef CONFIG_LOG_INPUT_PEDAL_ADC
            ESP_LOGI(TAG, "Pedal: %u mV (threshold: %u mV)", g_pedal_mv, (unsigned)PEDAL_ADC_THRESHOLD_MV);
    #endif

            if (g_pedal_mv >= PEDAL_ADC_THRESHOLD_MV) {
                g_pedal_was_above = true;
                g_pedal_below_threshold_since = 0;
            } else {
                if (g_pedal_was_above && g_pedal_below_threshold_since == 0)
                    g_pedal_below_threshold_since = now_ms;

                if (g_pedal_below_threshold_since > 0 &&
                    (now_ms - g_pedal_below_threshold_since) >= PEDAL_REARM_MS) {
                    g_pedal_was_above = false;
    #ifdef CONFIG_LOG_INPUT_PEDAL_ADC
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

static bool pedal_pressed(void) {
    return g_pedal_input_ready && (g_pedal_mv >= PEDAL_ADC_THRESHOLD_MV);
}

static bool pedal_rearmed(void) {
    if (!g_pedal_input_ready) return true;
    if (g_pedal_mv >= PEDAL_ADC_THRESHOLD_MV) return false;
    return !g_pedal_was_above;
}

static fr_state_t fr_state_debounced(void) {
    if (!g_fr_inputs_ready) return FR_STATE_NEUTRAL;
    return optocoupler_pc817_get_state();
}

static void log_startup_device_status(bool twai_ready,
                                      bool mux_ready,
                                      bool relay_ready,
                                      bool pedal_ready,
                                      bool fr_ready,
                                      bool stepper_steering_ready,
                                      bool stepper_braking_ready,
                                      bool heartbeat_ready) {
    if (twai_ready)
        ESP_LOGI(TAG_INIT, "TWAI: OK: tx_gpio=%d rx_gpio=%d", TWAI_TX_GPIO, TWAI_RX_GPIO);
    else
        ESP_LOGE(TAG_INIT, "TWAI: FAILED: tx_gpio=%d rx_gpio=%d", TWAI_TX_GPIO, TWAI_RX_GPIO);

#ifdef CONFIG_BYPASS_ACTUATOR_MULTIPLEXER
    ESP_LOGW(TAG_INIT,
             "MULTIPLEXER: BYPASSED: disabled (a0_gpio=%d a1_gpio=%d a2_gpio=%d en_gpio=%d relay_gpio=%d)",
             g_mux_cfg.a0, g_mux_cfg.a1, g_mux_cfg.a2, g_mux_cfg.en, g_mux_cfg.relay);
#else
    if (mux_ready) {
        ESP_LOGI(TAG_INIT,
                 "MULTIPLEXER: OK: a0_gpio=%d a1_gpio=%d a2_gpio=%d en_gpio=%d relay_gpio=%d start=MANUAL",
                 g_mux_cfg.a0, g_mux_cfg.a1, g_mux_cfg.a2, g_mux_cfg.en, g_mux_cfg.relay);
    } else {
        ESP_LOGE(TAG_INIT,
                 "MULTIPLEXER: FAILED: a0_gpio=%d a1_gpio=%d a2_gpio=%d en_gpio=%d relay_gpio=%d",
                 g_mux_cfg.a0, g_mux_cfg.a1, g_mux_cfg.a2, g_mux_cfg.en, g_mux_cfg.relay);
    }
#endif

#ifdef CONFIG_BYPASS_ACTUATOR_PEDAL_RELAY
    ESP_LOGW(TAG_INIT,
             "PEDAL_RELAY: BYPASSED: disabled (gpio=%d active_level=%s)",
             g_relay_cfg.gpio, g_relay_cfg.active_high ? "HIGH" : "LOW");
#else
    if (relay_ready) {
        ESP_LOGI(TAG_INIT,
                 "PEDAL_RELAY: OK: gpio=%d active_level=%s start=DE-ENERGIZED",
                 g_relay_cfg.gpio, g_relay_cfg.active_high ? "HIGH" : "LOW");
    } else {
        ESP_LOGE(TAG_INIT,
                 "PEDAL_RELAY: FAILED: gpio=%d active_level=%s",
                 g_relay_cfg.gpio, g_relay_cfg.active_high ? "HIGH" : "LOW");
    }
#endif

#ifdef CONFIG_BYPASS_INPUT_PEDAL_ADC
    ESP_LOGW(TAG_INIT,
             "PEDAL_INPUT: BYPASSED: pedal override disabled (forced released, threshold_mv=%u)",
             (unsigned)PEDAL_ADC_THRESHOLD_MV);
#else
    if (pedal_ready) {
        ESP_LOGI(TAG_INIT,
                 "PEDAL_INPUT: OK: adc_channel=ADC%d_CH%d adc_mode=%s threshold_mv=%u initial_mv=%u",
                 g_pedal_adc_cfg.adc_unit + 1,
                 g_pedal_adc_cfg.adc_channel,
                 adc_12bitsar_is_calibrated() ? "curve_fit" : "raw",
                 (unsigned)PEDAL_ADC_THRESHOLD_MV,
                 g_pedal_mv);
    } else {
        ESP_LOGE(TAG_INIT,
                 "PEDAL_INPUT: FAILED: adc_channel=ADC%d_CH%d threshold_mv=%u",
                 g_pedal_adc_cfg.adc_unit + 1,
                 g_pedal_adc_cfg.adc_channel,
                 (unsigned)PEDAL_ADC_THRESHOLD_MV);
    }
#endif

#ifdef CONFIG_BYPASS_INPUT_FR_SENSOR
    ESP_LOGW(TAG_INIT,
             "FR_INPUT_FORWARD: BYPASSED: input forced (input_gpio=%d)",
             g_fr_pc817_cfg.forward_gpio);
    ESP_LOGW(TAG_INIT,
             "FR_INPUT_REVERSE: BYPASSED: input forced (input_gpio=%d)",
             g_fr_pc817_cfg.reverse_gpio);
#else
    if (fr_ready) {
        ESP_LOGI(TAG_INIT,
                 "FR_INPUT_FORWARD: OK: input_gpio=%d",
                 g_fr_pc817_cfg.forward_gpio);
        ESP_LOGI(TAG_INIT,
                 "FR_INPUT_REVERSE: OK: input_gpio=%d",
                 g_fr_pc817_cfg.reverse_gpio);
    } else {
        ESP_LOGE(TAG_INIT,
                 "FR_INPUT_FORWARD: FAILED: input_gpio=%d",
                 g_fr_pc817_cfg.forward_gpio);
        ESP_LOGE(TAG_INIT,
                 "FR_INPUT_REVERSE: FAILED: input_gpio=%d",
                 g_fr_pc817_cfg.reverse_gpio);
    }
#endif

#ifdef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
    ESP_LOGW(TAG_INIT,
             "STEPPER_STEERING: BYPASSED: actuator disabled (node_id=%u)",
             (unsigned)UIM2852_NODE_STEERING);
#else
    if (stepper_steering_ready) {
        ESP_LOGI(TAG_INIT,
                 "STEPPER_STEERING: OK: node_id=%u init_check=passed",
                 (unsigned)UIM2852_NODE_STEERING);
    } else {
        ESP_LOGE(TAG_INIT,
                 "STEPPER_STEERING: FAILED: node_id=%u init_check=failed",
                 (unsigned)UIM2852_NODE_STEERING);
    }
#endif
#ifdef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
    ESP_LOGW(TAG_INIT,
             "STEPPER_BRAKING: BYPASSED: actuator disabled (node_id=%u)",
             (unsigned)UIM2852_NODE_BRAKING);
#else
    if (stepper_braking_ready) {
        ESP_LOGI(TAG_INIT,
                 "STEPPER_BRAKING: OK: node_id=%u init_check=passed",
                 (unsigned)UIM2852_NODE_BRAKING);
    } else {
        ESP_LOGE(TAG_INIT,
                 "STEPPER_BRAKING: FAILED: node_id=%u init_check=failed",
                 (unsigned)UIM2852_NODE_BRAKING);
    }
#endif

    if (heartbeat_ready)
        ESP_LOGI(TAG_INIT, "HEARTBEAT_LED: OK: gpio=%d state=init", HEARTBEAT_LED_GPIO);
    else
        ESP_LOGW(TAG_INIT, "HEARTBEAT_LED: UNAVAILABLE: gpio=%d (non-critical)", HEARTBEAT_LED_GPIO);
}

[[maybe_unused]] static esp_err_t init_stepper_checked(stepper_motor_uim2852_t *motor, uint8_t node_id) {
    stepper_motor_uim2852_config_t cfg = STEPPER_MOTOR_UIM2852_CONFIG_DEFAULT();
    cfg.node_id = node_id;

    esp_err_t err = stepper_motor_uim2852_init(motor, &cfg);
    if (err != ESP_OK) return err;

    err = stepper_motor_uim2852_configure(motor);
    if (err != ESP_OK) return err;

    return stepper_motor_uim2852_disable(motor);
}

static bool all_required_components_ready(void) {
    bool ready = true;
    if (!g_twai_ready) ready = false;
#ifndef CONFIG_BYPASS_ACTUATOR_MULTIPLEXER
    if (!g_mux_ready) ready = false;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_PEDAL_RELAY
    if (!g_relay_ready) ready = false;
#endif
#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
    if (!g_pedal_input_ready) ready = false;
#endif
#ifndef CONFIG_BYPASS_INPUT_FR_SENSOR
    if (!g_fr_inputs_ready) ready = false;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
    if (!g_stepper_steering_ready) ready = false;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
    if (!g_stepper_braking_ready) ready = false;
#endif
    return ready;
}

static uint8_t primary_fault_from_component_health(void) {
    if (!g_twai_ready) return NODE_FAULT_CAN_TX;
#ifndef CONFIG_BYPASS_ACTUATOR_MULTIPLEXER
    if (!g_mux_ready) return NODE_FAULT_THROTTLE_INIT;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_PEDAL_RELAY
    if (!g_relay_ready) return NODE_FAULT_RELAY_INIT;
#endif
#if !defined(CONFIG_BYPASS_INPUT_PEDAL_ADC) || !defined(CONFIG_BYPASS_INPUT_FR_SENSOR)
    if (!g_pedal_input_ready || !g_fr_inputs_ready) return NODE_FAULT_SENSOR_INVALID;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
    if (!g_stepper_steering_ready) return NODE_FAULT_MOTOR_COMM;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
    if (!g_stepper_braking_ready) return NODE_FAULT_MOTOR_COMM;
#endif
    return NODE_FAULT_NONE;
}

static const char *sensor_fault_detail_string(fr_state_t fr_state, bool has_fr_sample) {
    if (has_fr_sample && fr_state == FR_STATE_INVALID) {
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

    if (pedal_unavailable && fr_unavailable) return "pedal_input+fr_input_unavailable";
    if (pedal_unavailable) return "pedal_input_unavailable";
    if (fr_unavailable) return "fr_input_unavailable";
    return "unspecified";
}

static const char *fault_detail_string(uint8_t fault_code, fr_state_t fr_state, bool has_fr_sample) {
    if (fault_code == NODE_FAULT_SENSOR_INVALID) {
        return sensor_fault_detail_string(fr_state, has_fr_sample);
    }
    return nullptr;
}

static void log_component_lost(const char *name, const char *detail) {
#ifdef CONFIG_LOG_COMPONENT_HEALTH_TRANSITIONS
    if (detail && detail[0] != '\0') {
        ESP_LOGE(TAG, "%s: LOST: %s", name, detail);
    } else {
        ESP_LOGE(TAG, "%s: LOST", name);
    }
#else
    (void)name;
    (void)detail;
#endif
}

static void log_component_regained(const char *name) {
#ifdef CONFIG_LOG_COMPONENT_HEALTH_TRANSITIONS
    ESP_LOGI(TAG, "%s: REGAINED", name);
#else
    (void)name;
#endif
}

static void mark_component_lost(bool *ready, const char *name, const char *detail) {
    if (!ready) return;
    if (*ready) {
        log_component_lost(name, detail);
    }
    *ready = false;
}

static void handle_mux_runtime_error(esp_err_t err, const char *detail) {
    if (err == ESP_OK) return;
    mark_component_lost(&g_mux_ready, "MULTIPLEXER", detail);
}

[[maybe_unused]] static void handle_relay_runtime_error(esp_err_t err, const char *detail) {
    if (err == ESP_OK) return;
    mark_component_lost(&g_relay_ready, "PEDAL_RELAY", detail);
}

static void update_control_state_from_component_health(void) {
    if (!all_required_components_ready()) {
        g_fault_code = primary_fault_from_component_health();
        g_control_state = NODE_STATE_FAULT;
        return;
    }

    if (g_control_state == NODE_STATE_FAULT) {
#ifdef CONFIG_LOG_COMPONENT_HEALTH_TRANSITIONS
        ESP_LOGI(TAG, "SYSTEM: RECOVERED: all required components healthy");
#endif
        g_fault_code = NODE_FAULT_NONE;
        g_control_state = NODE_STATE_READY;
    }
}

// Wait for can_rx_task (and any in-flight send_control_heartbeat) to exit
// TWAI API calls. Must be called AFTER setting g_twai_ready = false.
// can_rx_task's twai_receive() has a CAN_RX_TIMEOUT (10ms) bound, so a
// 20ms delay guarantees it has returned and entered its idle sleep.
// send_control_heartbeat() checks g_twai_ready at the top, so after this
// delay no task is calling twai_transmit() or twai_receive().
static void quiesce_can_rx() {
    vTaskDelay(pdMS_TO_TICKS(20));
}

// Rate-limit retry failure logs: log on 1st attempt, then every RETRY_LOG_EVERY_N
// attempts. At RETRY_INTERVAL_MS = 500ms, N=20 gives ~10 second intervals.
static const uint16_t RETRY_LOG_EVERY_N = 20;

static void retry_failed_components(void) {
    // Pace retries at RETRY_INTERVAL_MS to avoid hammering init calls every
    // control loop tick. Retries remain infinite (no cap).
    uint32_t now_ms = get_time_ms();
    if ((now_ms - g_last_retry_ms) < RETRY_INTERVAL_MS) return;
    g_last_retry_ms = now_ms;

    if (!g_twai_ready) {
        [[maybe_unused]] static uint16_t s_retry_count = 0;
        // Skip if another task is already performing CAN recovery.
        taskENTER_CRITICAL(&g_can_tx_lock);
        bool skip = g_can_recovery_in_progress;
        if (!skip) g_can_recovery_in_progress = true;
        taskEXIT_CRITICAL(&g_can_tx_lock);
        if (skip) return;

        // g_twai_ready is already false; quiesce before touching driver.
        quiesce_can_rx();

        // Suppress internal can_twai_recover_with_reinit() logs unless
        // CONFIG_LOG_CAN_RECOVERY is enabled; then rate-limit to 1st + every Nth.
#ifdef CONFIG_LOG_CAN_RECOVERY
        const char *verbose_tag = (s_retry_count == 0 ||
            ((s_retry_count + 1) % RETRY_LOG_EVERY_N) == 0) ? TAG : nullptr;
#else
        const char *verbose_tag = nullptr;
#endif
        esp_err_t err = can_twai_recover_with_reinit(TWAI_TX_GPIO, TWAI_RX_GPIO, verbose_tag);
        bool recovered = false;
        if (err == ESP_OK) {
            // Active probe: send one heartbeat frame to verify bus is truly connected.
            // After reinit the driver starts RUNNING with zero error counters, so a
            // passive bus_ok() check always passes. Sending a real frame forces TEC
            // accumulation on a disconnected bus (TEC += ~8 per failed TX).
            uint8_t probe_data[8] = {0};
            node_heartbeat_t probe_hb = {
                .sequence = 0,
                .state = g_control_state,
                .fault_code = g_fault_code,
                .flags = g_heartbeat_flags,
            };
            can_encode_heartbeat(probe_data, &probe_hb);
            (void)can_twai_send(CAN_ID_CONTROL_HEARTBEAT, probe_data, pdMS_TO_TICKS(50));
            vTaskDelay(pdMS_TO_TICKS(50));
            twai_status_info_t probe_status;
            recovered = (twai_get_status_info(&probe_status) == ESP_OK &&
                         probe_status.state == TWAI_STATE_RUNNING &&
                         probe_status.tx_error_counter == 0);
        }
#ifdef CONFIG_LOG_RETRY_TWAI
        if (!recovered) {
            s_retry_count++;
            if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0) {
                if (err != ESP_OK) {
                    ESP_LOGW(TAG, "TWAI retry failed (%u attempts): %s",
                             s_retry_count, esp_err_to_name(err));
                } else {
                    ESP_LOGW(TAG, "TWAI retry failed (%u attempts): bus unstable after reinit",
                             s_retry_count);
                }
            }
        }
#endif
        if (recovered) {
            s_retry_count = 0;
            g_twai_ready = true;
            log_component_regained("TWAI");
        }

        taskENTER_CRITICAL(&g_can_tx_lock);
        g_can_recovery_in_progress = false;
        taskEXIT_CRITICAL(&g_can_tx_lock);
    }

#ifndef CONFIG_BYPASS_ACTUATOR_MULTIPLEXER
    if (!g_mux_ready) {
        [[maybe_unused]] static uint16_t s_retry_count = 0;
        esp_err_t err = multiplexer_dg408djz_init(&g_mux_cfg);
        bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_MULTIPLEXER
        if (!recovered) {
            s_retry_count++;
            if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0) {
                ESP_LOGW(TAG, "MULTIPLEXER retry failed (%u attempts): %s",
                         s_retry_count, esp_err_to_name(err));
            }
        }
#endif
        if (recovered) {
            s_retry_count = 0;
            log_component_regained("MULTIPLEXER (driver-level)");
        }
        g_mux_ready = recovered;
    }
#endif

#ifndef CONFIG_BYPASS_ACTUATOR_PEDAL_RELAY
    if (!g_relay_ready) {
        [[maybe_unused]] static uint16_t s_retry_count = 0;
        esp_err_t err = relay_jd2912_init(&g_relay_cfg);
        bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_PEDAL_RELAY
        if (!recovered) {
            s_retry_count++;
            if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0) {
                ESP_LOGW(TAG, "PEDAL_RELAY retry failed (%u attempts): %s",
                         s_retry_count, esp_err_to_name(err));
            }
        }
#endif
        if (recovered) {
            s_retry_count = 0;
            log_component_regained("PEDAL_RELAY (driver-level)");
        }
        g_relay_ready = recovered;
    }
#endif

#ifndef CONFIG_BYPASS_INPUT_PEDAL_ADC
    if (!g_pedal_input_ready) {
        [[maybe_unused]] static uint16_t s_retry_count = 0;
        esp_err_t err = init_pedal_input();
        bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_PEDAL_INPUT
        if (!recovered) {
            s_retry_count++;
            if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0) {
                ESP_LOGW(TAG, "PEDAL_INPUT retry failed (%u attempts): %s",
                         s_retry_count, esp_err_to_name(err));
            }
        }
#endif
        if (recovered) {
            s_retry_count = 0;
            log_component_regained("PEDAL_INPUT");
        }
        g_pedal_input_ready = recovered;
    }
#endif

#ifndef CONFIG_BYPASS_INPUT_FR_SENSOR
    if (!g_fr_inputs_ready) {
        [[maybe_unused]] static uint16_t s_retry_count = 0;
        esp_err_t err = init_fr_inputs();
        bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_FR_INPUT
        if (!recovered) {
            s_retry_count++;
            if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0) {
                ESP_LOGW(TAG, "FR_INPUT_FORWARD retry failed (%u attempts): %s (input_gpio=%d)",
                         s_retry_count, esp_err_to_name(err), g_fr_pc817_cfg.forward_gpio);
                ESP_LOGW(TAG, "FR_INPUT_REVERSE retry failed (%u attempts): %s (input_gpio=%d)",
                         s_retry_count, esp_err_to_name(err), g_fr_pc817_cfg.reverse_gpio);
            }
        }
#endif
        if (recovered) {
            s_retry_count = 0;
            log_component_regained("FR_INPUT");
        }
        g_fr_inputs_ready = recovered;
    }
#endif

#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
    if (!g_stepper_steering_ready) {
        [[maybe_unused]] static uint16_t s_retry_count = 0;
        esp_err_t err = init_stepper_checked(&g_steering_stepper, UIM2852_NODE_STEERING);
        bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_STEPPER_STEERING
        if (!recovered) {
            s_retry_count++;
            if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0) {
                ESP_LOGW(TAG, "STEPPER_STEERING retry failed (%u attempts): %s",
                         s_retry_count, esp_err_to_name(err));
            }
        }
#endif
        if (recovered) {
            s_retry_count = 0;
            log_component_regained("STEPPER_STEERING");
        }
        g_stepper_steering_ready = recovered;
    }
#endif

#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
    if (!g_stepper_braking_ready) {
        [[maybe_unused]] static uint16_t s_retry_count = 0;
        esp_err_t err = init_stepper_checked(&g_braking_stepper, UIM2852_NODE_BRAKING);
        bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_STEPPER_BRAKING
        if (!recovered) {
            s_retry_count++;
            if (s_retry_count == 1 || (s_retry_count % RETRY_LOG_EVERY_N) == 0) {
                ESP_LOGW(TAG, "STEPPER_BRAKING retry failed (%u attempts): %s",
                         s_retry_count, esp_err_to_name(err));
            }
        }
#endif
        if (recovered) {
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

static void update_control_led_mode() {
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    const char *reason = "state_ready";

    if (g_control_state == NODE_STATE_FAULT) {
        red = LED_LEVEL;
        reason = node_fault_to_string(g_fault_code);
    } else if (g_control_state == NODE_STATE_OVERRIDE) {
        red = LED_LEVEL;
        reason = "state_override";
    } else if (g_control_state == NODE_STATE_ACTIVE) {
        blue = LED_LEVEL;
        reason = "state_active";
    } else if (g_control_state == NODE_STATE_ENABLING) {
        red = LED_LEVEL;
        green = LED_ORANGE_GREEN;
        reason = "state_enabling";
    } else if (g_control_state == NODE_STATE_READY) {
        green = LED_LEVEL;
        reason = "state_ready";
    } else {
        red = LED_LEVEL;
        green = LED_ORANGE_GREEN;
        reason = "state_other";
    }

    led_ws2812_set_manual_color(red, green, blue, reason);
}

// Track CAN TX result. On consecutive failures, mark TWAI lost.
// Protected by g_can_tx_lock to prevent concurrent recovery from control_task + heartbeat_task.
//
// IMPORTANT: can_twai_bus_ok() and ESP_LOG* must NOT be called inside
// taskENTER_CRITICAL sections. On single-core ESP32-C6, critical sections
// disable interrupts; calling into the TWAI driver or newlib fprintf with
// interrupts disabled causes abort() in lock_acquire_generic.
static void track_can_tx(esp_err_t err) {
    bool trigger_recovery = false;
    bool check_bus = false;

    taskENTER_CRITICAL(&g_can_tx_lock);
    can_tx_track_inputs_t t = {
        .fail_count = g_recovery.can_tx_fail_count,
        .threshold = CAN_TX_CONSEC_FAIL_THRESHOLD,
        .tx_ok = (err == ESP_OK),
    };
    can_tx_track_result_t r = control_track_can_tx(&t);
    g_recovery.can_tx_fail_count = r.new_fail_count;
    if (r.trigger_recovery) {
        g_recovery.can_tx_fail_count = 0;  // reset regardless
        check_bus = true;
    }
    taskEXIT_CRITICAL(&g_can_tx_lock);

    // Bus check and logging happen outside the critical section so that
    // twai_get_status_info() and ESP_LOG (which use locks/semaphores)
    // are called with interrupts enabled.
    if (check_bus) {
        bool bus_ok = can_twai_bus_ok();
        if (!bus_ok) {
            taskENTER_CRITICAL(&g_can_tx_lock);
            if (!g_can_recovery_in_progress) {
                g_can_recovery_in_progress = true;
                trigger_recovery = true;
            }
            taskEXIT_CRITICAL(&g_can_tx_lock);
        }
        // else: bus OK but TX still failing — no partner on bus, nothing to do.
    }

    if (trigger_recovery) {
#ifdef CONFIG_LOG_CAN_RECOVERY
        ESP_LOGE(TAG, "CAN bus unhealthy after %d TX failures", CAN_TX_CONSEC_FAIL_THRESHOLD);
#endif
        mark_component_lost(&g_twai_ready, "TWAI", "runtime bus unhealthy after tx failures");
        // Recovery handled by retry_failed_components() at 500ms pace.
        taskENTER_CRITICAL(&g_can_tx_lock);
        g_can_recovery_in_progress = false;
        taskEXIT_CRITICAL(&g_can_tx_lock);
    }
}

// ============================================================================
// Control Heartbeat Send Helper
// ============================================================================

// Send the control heartbeat on CAN. Called from heartbeat_task (periodic)
// and from control_task on state change (immediate).
static void send_control_heartbeat(void) {
    // Skip TX while driver is down or being recovered. Avoids calling
    // twai_transmit() on a torn-down driver, which can abort/crash.
    if (!g_twai_ready) return;

#ifdef CONFIG_LOG_CAN_RECOVERY
    // Heap integrity diagnostic — detect corruption early (before ESP_LOG triggers abort).
    // heap_caps_check_integrity_all returns true if heap is intact.
    if (!heap_caps_check_integrity_all(true)) {
        ESP_LOGE(TAG, "HEAP CORRUPTION detected in send_control_heartbeat! free=%lu",
                 (unsigned long)esp_get_free_heap_size());
    }
#endif

    uint8_t hb_data[8] = {0};

    taskENTER_CRITICAL(&g_hb_seq_lock);
    uint8_t seq = g_heartbeat_seq;
    g_heartbeat_seq = (uint8_t)(seq + 1);
    taskEXIT_CRITICAL(&g_hb_seq_lock);

    node_heartbeat_t hb_msg = {
        .sequence = seq,
        .state = g_control_state,
        .fault_code = g_fault_code,
        .flags = g_heartbeat_flags,
    };
    can_encode_heartbeat(hb_data, &hb_msg);

    esp_err_t err = can_twai_send(CAN_ID_CONTROL_HEARTBEAT, hb_data, pdMS_TO_TICKS(10));
#ifdef CONFIG_LOG_CAN_HEARTBEAT_TX
    if (err != ESP_OK) {
        if (!g_hb_tx_failing) {
            ESP_LOGE(TAG_TX, "Heartbeat TX failing: %s", esp_err_to_name(err));
            g_hb_tx_failing = true;
        }
    } else {
        if (g_hb_tx_failing) {
            ESP_LOGI(TAG_TX, "Heartbeat TX recovered");
            g_hb_tx_failing = false;
        }
        ESP_LOGI(TAG_TX, "HB: seq=%u state=%s fault=%s flags=0x%02X",
                 seq, node_state_to_string(g_control_state),
                 node_fault_to_string(g_fault_code), g_heartbeat_flags);
    }
#endif
    track_can_tx(err);
}

// ============================================================================
// State Machine Action Helpers
// ============================================================================

[[maybe_unused]] static const char *override_reason_to_string(uint8_t reason) {
    switch (reason) {
        case OVERRIDE_REASON_PEDAL: return "pedal";
        case OVERRIDE_REASON_FR_CHANGED: return "fr_changed";
        case OVERRIDE_REASON_STEERING: return "steering_position_error";
        case OVERRIDE_REASON_BRAKING: return "braking_position_error";
        case OVERRIDE_REASON_NONE:
        default: return "none";
    }
}

[[maybe_unused]] static const char *disable_reason_to_string(uint8_t reason) {
    switch (reason) {
        case CONTROL_DISABLE_REASON_SAFETY_RETREAT: return "safety_retreat";
        case CONTROL_DISABLE_REASON_MOTOR_FAULT: return "motor_fault";
        case CONTROL_DISABLE_REASON_SENSOR_INVALID: return "sensor_invalid";
        case CONTROL_DISABLE_REASON_NONE:
        default: return "none";
    }
}

// Immediately disable all autonomous actuators.
static void disable_autonomous_actuators(void) {
#ifndef CONFIG_BYPASS_ACTUATOR_MULTIPLEXER
    handle_mux_runtime_error(multiplexer_dg408djz_disable(), "disable command failed");
    handle_mux_runtime_error(multiplexer_dg408djz_emergency_stop(), "emergency stop command failed");
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_PEDAL_RELAY
    handle_relay_runtime_error(relay_jd2912_deenergize(), "de-energize command failed");
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
    stepper_motor_uim2852_emergency_stop(&g_steering_stepper);
    // Engage brake before disabling driver (spec: ST -> brake -> MO=0)
    {
        uint8_t bdata[8];
        uint8_t bdl = stepper_uim2852_build_brake(bdata, true);
        stepper_motor_uim2852_send_instruction(&g_steering_stepper, STEPPER_UIM2852_CW_MT, bdata, bdl);
    }
    stepper_motor_uim2852_disable(&g_steering_stepper);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
    stepper_motor_uim2852_emergency_stop(&g_braking_stepper);
    // Engage brake before disabling driver (spec: ST -> brake -> MO=0)
    {
        uint8_t bdata[8];
        uint8_t bdl = stepper_uim2852_build_brake(bdata, true);
        stepper_motor_uim2852_send_instruction(&g_braking_stepper, STEPPER_UIM2852_CW_MT, bdata, bdl);
    }
    stepper_motor_uim2852_disable(&g_braking_stepper);
#endif
}

// Immediately disable all autonomous actuators.
static void execute_trigger_override(uint8_t reason) {
#ifdef CONFIG_LOG_CONTROL_OVERRIDE
    ESP_LOGI(TAG, "OVERRIDE TRIGGERED (reason: %s)", override_reason_to_string(reason));
#else
    (void)reason;
#endif

    disable_autonomous_actuators();
}

// Disable autonomy without classifying event as driver override.
static void execute_disable_autonomy(uint8_t reason,
                                     uint8_t estop_fault_code,
                                     uint8_t control_fault_code) {
#if defined(CONFIG_LOG_CONTROL_STATE_CHANGES) || defined(CONFIG_LOG_CONTROL_ENABLE_SEQUENCE)
    if (reason == CONTROL_DISABLE_REASON_SAFETY_RETREAT) {
        ESP_LOGI(TAG, "AUTONOMY DISABLED (reason: %s, safety_fault: %s)",
                 disable_reason_to_string(reason),
                 node_fault_to_string(estop_fault_code));
    } else {
        ESP_LOGI(TAG, "AUTONOMY DISABLED (reason: %s, control_fault: %s)",
                 disable_reason_to_string(reason),
                 node_fault_to_string(control_fault_code));
    }
#else
    (void)reason;
    (void)estop_fault_code;
    (void)control_fault_code;
#endif

    disable_autonomous_actuators();
}

// Start the enable sequence (set mux to 0, energize relay)
static void execute_start_enable() {
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
    ESP_LOGI(TAG, "Starting autonomous enable sequence");
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MULTIPLEXER
    handle_mux_runtime_error(multiplexer_dg408djz_set_level(0), "set level command failed");
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_PEDAL_RELAY
    handle_relay_runtime_error(relay_jd2912_energize(), "energize command failed");
#endif
}

// Complete the enable sequence (enable steppers, switch mux to autonomous)
static void execute_complete_enable() {
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
    ESP_LOGI(TAG, "Completing autonomous enable sequence");
#endif
    esp_err_t steer_err = ESP_OK;
    esp_err_t brake_err = ESP_OK;
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
    steer_err = stepper_motor_uim2852_enable(&g_steering_stepper);
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
    if (steer_err != ESP_OK) ESP_LOGI(TAG, "Steering enable failed: %s", esp_err_to_name(steer_err));
#endif
    vTaskDelay(pdMS_TO_TICKS(5));
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
    brake_err = stepper_motor_uim2852_enable(&g_braking_stepper);
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
    if (brake_err != ESP_OK) ESP_LOGI(TAG, "Braking enable failed: %s", esp_err_to_name(brake_err));
#endif
#endif

    if (steer_err != ESP_OK || brake_err != ESP_OK) {
        ESP_LOGE(TAG, "Stepper enable failed, aborting autonomous enable");
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
        stepper_motor_uim2852_disable(&g_steering_stepper);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
        stepper_motor_uim2852_disable(&g_braking_stepper);
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_PEDAL_RELAY
        handle_relay_runtime_error(relay_jd2912_deenergize(), "de-energize command failed");
#endif
        handle_mux_runtime_error(multiplexer_dg408djz_disable(), "disable command failed");
        return;
    }

#ifndef CONFIG_BYPASS_ACTUATOR_MULTIPLEXER
    handle_mux_runtime_error(multiplexer_dg408djz_enable_autonomous(), "enable autonomous command failed");
#endif
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
    ESP_LOGI(TAG, "AUTONOMOUS MODE ACTIVE");
#endif
}

// Abort the enable sequence (de-energize relay, disable mux)
static void execute_abort_enable() {
#ifdef CONFIG_LOG_CONTROL_ENABLE_SEQUENCE
    ESP_LOGI(TAG, "Enable sequence aborted");
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_PEDAL_RELAY
    handle_relay_runtime_error(relay_jd2912_deenergize(), "de-energize command failed");
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_MULTIPLEXER
    handle_mux_runtime_error(multiplexer_dg408djz_disable(), "disable command failed");
#endif
}

// ============================================================================
// Fault Recovery
// ============================================================================

// Attempt to recover from the current fault by re-initializing the failed component.
// Returns true when required components currently report healthy.
// Fault->READY transition is applied in the control loop after state updates.
static bool attempt_fault_recovery() {
    return all_required_components_ready();
}

// ============================================================================
// CAN RX Task
// ============================================================================

void can_rx_task(void *param) {
    (void)param;
    twai_message_t msg{};

    while (true) {
        // Skip TWAI API calls while driver is down or being recovered.
        // Sleeping 100ms (not 5ms) prevents this priority-7 task from
        // starving lower-priority tasks when the bus is unavailable.
        if (!g_twai_ready) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (can_twai_receive(&msg, CAN_RX_TIMEOUT) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }

        if (msg.rtr) continue;

        TickType_t now = xTaskGetTickCount();

#if !defined(CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING) || !defined(CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING)
        // Handle extended frames from UIM2852CA stepper motors
        if (msg.extd) {
            bool matched = false;
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
            if (stepper_motor_uim2852_process_frame(&g_steering_stepper, &msg)) {
                matched = true;
                if (stepper_motor_uim2852_stall_detected(&g_steering_stepper) ||
                    stepper_motor_uim2852_has_error(&g_steering_stepper)) {
#ifdef CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX
                    ESP_LOGI(TAG_RX, "Steering stepper motor fault detected");
#endif
                    taskENTER_CRITICAL(&g_cmd_lock);
                    g_cmd.motor_fault_code = NODE_FAULT_MOTOR_COMM;
                    taskEXIT_CRITICAL(&g_cmd_lock);
                }
            }
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
            if (!matched && stepper_motor_uim2852_process_frame(&g_braking_stepper, &msg)) {
                matched = true;
                if (stepper_motor_uim2852_stall_detected(&g_braking_stepper) ||
                    stepper_motor_uim2852_has_error(&g_braking_stepper)) {
#ifdef CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX
                    ESP_LOGI(TAG_RX, "Braking stepper motor fault detected");
#endif
                    taskENTER_CRITICAL(&g_cmd_lock);
                    g_cmd.motor_fault_code = NODE_FAULT_MOTOR_COMM;
                    taskEXIT_CRITICAL(&g_cmd_lock);
                }
            }
#endif
            (void)matched;
            continue;
        }
#else
        if (msg.extd) continue;
#endif

        // Process Planner command (0x111)
        if (msg.identifier == CAN_ID_PLANNER_COMMAND && msg.data_length_code >= 6) {
            planner_command_t cmd;
            can_decode_planner_command(msg.data, &cmd);
            
            int8_t throttle_level = (int8_t)(cmd.throttle & 0x07);

            g_last_planner_cmd_tick = xTaskGetTickCount();

            // Stale command detection: track sequence changes
            if (cmd.sequence == g_planner_cmd_last_seq) {
                if (g_planner_cmd_stale_count < 255)
                    g_planner_cmd_stale_count = (uint8_t)(g_planner_cmd_stale_count + 1);
            } else {
                g_planner_cmd_last_seq = cmd.sequence;
                g_planner_cmd_stale_count = 0;
            }

            taskENTER_CRITICAL(&g_cmd_lock);
            g_cmd.throttle_target = throttle_level;
            g_cmd.steering_cmd = cmd.steering_position;
            g_cmd.braking_cmd = cmd.braking_position;
            taskEXIT_CRITICAL(&g_cmd_lock);
            
            led_ws2812_mark_activity(now);

#ifdef CONFIG_LOG_CAN_PLANNER_COMMAND_RX
            ESP_LOGI(TAG_RX, "CMD: thr=%d steer=%d brake=%d seq=%u",
                throttle_level, cmd.steering_position, cmd.braking_position, cmd.sequence);
#endif
        }

        // Process Safety heartbeat (0x100)
        else if (msg.identifier == CAN_ID_SAFETY_HEARTBEAT && msg.data_length_code >= 4) {
            node_heartbeat_t hb;
            can_decode_heartbeat(msg.data, &hb);

            uint8_t new_target = sanitize_safety_target_state(hb.state);  // Safety system target
            uint8_t fault_code = hb.fault_code;     // Safety's fault = estop reason

            taskENTER_CRITICAL(&g_cmd_lock);
            g_cmd.target_state = new_target;
            g_cmd.estop_fault_code = fault_code;
            taskEXIT_CRITICAL(&g_cmd_lock);

            heartbeat_monitor_update(&g_hb_monitor, g_node_safety, hb.sequence, new_target);
            
            led_ws2812_mark_activity(now);

#ifdef CONFIG_LOG_CAN_HEARTBEAT_RX
            static uint8_t prev_safety_target = 0xFF;
            static uint8_t prev_safety_fault = 0xFF;
            if (fault_code != 0 && prev_safety_fault == 0) {
                ESP_LOGI(TAG_RX, "Safety FAULT: %s",
                         node_fault_to_string(fault_code));
            } else if (fault_code == 0 && prev_safety_fault != 0) {
                ESP_LOGI(TAG_RX, "Safety fault cleared");
            }
            if (new_target != prev_safety_target) {
                ESP_LOGI(TAG_RX, "Safety target: %s -> %s",
                         node_state_to_string(prev_safety_target),
                         node_state_to_string(new_target));
            } else {
                ESP_LOGI(TAG_RX, "Safety HB: seq=%u target=%s fault=%s",
                         hb.sequence, node_state_to_string(new_target),
                         node_fault_to_string(fault_code));
            }
            prev_safety_target = new_target;
            prev_safety_fault = fault_code;
#endif
        }
    }
}

// ============================================================================
// Control Task
// ============================================================================

void control_task(void *param) {
    (void)param;

    // Declare loop-state variables before any goto to satisfy C++ scoping rules
#ifdef CONFIG_LOG_CONTROL_STATE_CHANGES
    uint8_t prev_state = 0xFF;
#endif
#ifdef CONFIG_LOG_CONTROL_FAULT_CHANGES
    uint8_t prev_fault_code = 0xFF;
#endif
#ifdef CONFIG_LOG_CONTROL_SAFETY_MIRROR_CHANGES
    uint8_t prev_target_state = 0xFF;
    uint8_t prev_estop_fault = 0xFF;
#endif
    int16_t last_steering_sent = INT16_MIN;
    int16_t last_braking_sent = INT16_MIN;

    g_throttle_current = 0;

#ifdef CONFIG_LOG_CONTROL_STATE_CHANGES
    prev_state = g_control_state;
#endif
#ifdef CONFIG_LOG_CONTROL_FAULT_CHANGES
    prev_fault_code = g_fault_code;
#endif
#ifdef CONFIG_LOG_CONTROL_SAFETY_MIRROR_CHANGES
    prev_target_state = g_cmd.target_state;
    prev_estop_fault = g_cmd.estop_fault_code;
#endif

    while (true) {
        uint32_t now_ms = get_time_ms();

        retry_failed_components();

        // Take a thread-safe snapshot of CAN RX state
        command_snapshot_t cmd_local;
        taskENTER_CRITICAL(&g_cmd_lock);
        cmd_local = g_cmd;
        // Clear one-shot motor fault flag after reading
        g_cmd.motor_fault_code = NODE_FAULT_NONE;
        taskEXIT_CRITICAL(&g_cmd_lock);

        if (cmd_local.motor_fault_code == NODE_FAULT_MOTOR_COMM) {
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
        cmd_local.estop_fault_code = NODE_FAULT_NONE;
#endif
#ifdef CONFIG_BYPASS_PLANNER_COMMAND_INPUTS
        cmd_local.throttle_target = 0;
        cmd_local.steering_cmd = 0;
        cmd_local.braking_cmd = 0;
#endif

        // Check Safety heartbeat timeout — retreat to READY if Safety is gone.
        heartbeat_monitor_check_timeouts(&g_hb_monitor);
        bool safety_alive = heartbeat_monitor_is_alive(&g_hb_monitor, g_node_safety);
#ifdef CONFIG_BYPASS_SAFETY_LIVENESS_CHECKS
        safety_alive = true;
#endif
        if (!safety_alive) {
            cmd_local.target_state = NODE_STATE_READY;  // retreat to safe state
        }

        // Check Planner command freshness - zero throttle if stale
#ifndef CONFIG_BYPASS_PLANNER_COMMAND_STALE_CHECKS
        TickType_t planner_age = xTaskGetTickCount() - g_last_planner_cmd_tick;
        bool planner_cmd_stale = false;

        if (g_control_state == NODE_STATE_ACTIVE && g_last_planner_cmd_tick > 0) {
            // Timeout: no command received for PLANNER_CMD_TIMEOUT
            if (planner_age > PLANNER_CMD_TIMEOUT) {
                planner_cmd_stale = true;
#ifdef CONFIG_LOG_CAN_PLANNER_COMMAND_RX
                ESP_LOGI(TAG, "Planner command timeout (%lu ms), zeroing throttle",
                         (unsigned long)(planner_age * portTICK_PERIOD_MS));
#endif
            }
            // Stale sequence: same sequence seen PLANNER_CMD_STALE_COUNT times
            else if (g_planner_cmd_stale_count >= PLANNER_CMD_STALE_COUNT) {
                planner_cmd_stale = true;
#ifdef CONFIG_LOG_CAN_PLANNER_COMMAND_RX
                ESP_LOGI(TAG, "Planner command stale (seq=%u repeated %u times), zeroing throttle",
                         g_planner_cmd_last_seq, g_planner_cmd_stale_count);
#endif
            }
        }

        if (planner_cmd_stale) {
            cmd_local.throttle_target = 0;
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
        if (g_stepper_steering_ready && !g_steering_stepper.motion_in_progress) {
            steering_pos_err = (stepper_motor_uim2852_position_error(&g_steering_stepper)
                                > STEPPER_POSITION_ERROR_THRESHOLD);
        }
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
        if (g_stepper_braking_ready && !g_braking_stepper.motion_in_progress) {
            braking_pos_err = (stepper_motor_uim2852_position_error(&g_braking_stepper)
                                > STEPPER_POSITION_ERROR_THRESHOLD);
        }
#endif

        control_inputs_t step_in = {
            .target_state = cmd_local.target_state,
            .throttle_target = cmd_local.throttle_target,
            .steering_cmd = cmd_local.steering_cmd,
            .braking_cmd = cmd_local.braking_cmd,
            .motor_fault_code = cmd_local.motor_fault_code,
            .fr_state = fr_state,
            .pedal_pressed = pedal_pressed_now,
            .pedal_rearmed = pedal_rearmed_now,
            .fr_is_invalid = (fr_state == FR_STATE_INVALID),
            .steering_position_error = steering_pos_err,
            .braking_position_error = braking_pos_err,
            .now_ms = now_ms,
            .enable_start_ms = g_enable_start_ms,
            .enable_sequence_ms = ENABLE_SEQUENCE_MS,
            .enable_work_done = g_enable_work_done,
            .throttle_current = g_throttle_current,
            .last_throttle_change_ms = g_last_throttle_change_ms,
            .throttle_slew_interval_ms = THROTTLE_SLEW_INTERVAL_MS,
            .last_steering_sent = last_steering_sent,
            .last_braking_sent = last_braking_sent,
        };

        // Compute next state (pure function — no side effects)
        control_step_result_t step = control_compute_step(g_control_state, g_fault_code, &step_in);

#ifdef CONFIG_LOG_CONTROL_STATE_TICK
        ESP_LOGI(TAG, "tick state=%s target=%s estop=%s fr=%d pedal=%d/%d thr=%d/%d actions=0x%02X -> %s",
                 node_state_to_string(g_control_state),
                 node_state_to_string(cmd_local.target_state),
                 node_fault_to_string(cmd_local.estop_fault_code),
                 fr_state, pedal_pressed_now, pedal_rearmed_now,
                 g_throttle_current, cmd_local.throttle_target,
                 step.actions, node_state_to_string(step.new_state));
#endif

        // Execute hardware actions indicated by the step result
        if (step.actions & CONTROL_ACTION_TRIGGER_OVERRIDE) {
            execute_trigger_override(step.override_reason);
        }
        if (step.actions & CONTROL_ACTION_DISABLE_AUTONOMY) {
            execute_disable_autonomy(step.disable_reason,
                                     cmd_local.estop_fault_code,
                                     step.new_fault_code);
        }
        if (step.actions & CONTROL_ACTION_START_ENABLE) {
            execute_start_enable();
        }
        if (step.actions & CONTROL_ACTION_COMPLETE_ENABLE) {
            execute_complete_enable();
#if (!defined(CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING) || !defined(CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING)) && !defined(CONFIG_BYPASS_ACTUATOR_MULTIPLEXER)
            // If stepper enable failed, execute_complete_enable leaves mux disabled
            if (!multiplexer_dg408djz_is_autonomous()) {
                step.new_state = NODE_STATE_FAULT;
                step.new_fault_code = NODE_FAULT_MOTOR_COMM;
            }
#endif
        }
        if (step.actions & CONTROL_ACTION_ABORT_ENABLE) {
            execute_abort_enable();
        }
        if (step.actions & CONTROL_ACTION_APPLY_THROTTLE) {
#ifndef CONFIG_BYPASS_ACTUATOR_MULTIPLEXER
            handle_mux_runtime_error(multiplexer_dg408djz_set_level(step.throttle_level),
                                     "set level command failed");
#endif
#ifdef CONFIG_LOG_CONTROL_THROTTLE_CHANGES
            {
                static int8_t prev_level = -1;
                if (step.throttle_level != prev_level) {
                    ESP_LOGI(TAG, "Throttle level: %d -> %d", prev_level, step.throttle_level);
                    prev_level = step.throttle_level;
                }
            }
#endif
        }

#ifdef CONFIG_LOG_CONTROL_THROTTLE_TICK
        ESP_LOGI(TAG, "Throttle tick: current=%d target=%d",
                 step.throttle_level, cmd_local.throttle_target);
#endif

        if (step.actions & CONTROL_ACTION_ATTEMPT_RECOVERY) {
            (void)attempt_fault_recovery();
        }

        // Send stepper commands when position changed
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
        if (step.send_steering) {
            esp_err_t err = stepper_motor_uim2852_go_absolute(&g_steering_stepper, step.steering_position);
            if (err != ESP_OK) {
                step.new_last_steering = last_steering_sent;  // keep old value on failure
            }
#ifdef CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX
            if (err != ESP_OK) {
                ESP_LOGI(TAG_TX, "Steering cmd failed: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG_TX, "Steering -> %d", step.steering_position);
            }
#endif
            track_can_tx(err);
        }
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
        if (step.send_braking) {
            esp_err_t err = stepper_motor_uim2852_go_absolute(&g_braking_stepper, step.braking_position);
            if (err != ESP_OK) {
                step.new_last_braking = last_braking_sent;  // keep old value on failure
            }
#ifdef CONFIG_LOG_ACTUATOR_STEPPER_COMMAND_TX
            if (err != ESP_OK) {
                ESP_LOGI(TAG_TX, "Braking cmd failed: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG_TX, "Braking -> %d", step.braking_position);
            }
#endif
            track_can_tx(err);
        }
#endif

        // Detect fault entry (before overwriting g_fault_code)
        if (step.new_fault_code != NODE_FAULT_NONE && step.new_fault_code != g_fault_code) {
            const char *detail = fault_detail_string(step.new_fault_code, fr_state, true);
            if (detail) {
                ESP_LOGE(TAG, "Entering FAULT state: %s (detail=%s)",
                         node_fault_to_string(step.new_fault_code), detail);
            } else {
                ESP_LOGE(TAG, "Entering FAULT state: %s", node_fault_to_string(step.new_fault_code));
            }
        }

        // Detect state change for immediate heartbeat send
        uint8_t old_state = g_control_state;

        // Apply state updates
        g_control_state = step.new_state;
        g_fault_code = step.new_fault_code;
        g_heartbeat_flags = step.heartbeat_flags;
        g_throttle_current = step.throttle_level;
        g_last_throttle_change_ms = step.throttle_change_ms;
        g_enable_start_ms = step.enable_start_ms;
        g_enable_work_done = step.enable_work_done;
        last_steering_sent = step.new_last_steering;
        last_braking_sent = step.new_last_braking;

        if (step.new_fault_code == NODE_FAULT_THROTTLE_INIT && g_mux_ready) {
            mark_component_lost(&g_mux_ready, "MULTIPLEXER", "runtime init fault");
        }
        if (step.new_fault_code == NODE_FAULT_RELAY_INIT && g_relay_ready) {
            mark_component_lost(&g_relay_ready, "PEDAL_RELAY", "runtime init fault");
        }
        if (step.new_fault_code == NODE_FAULT_SENSOR_INVALID && g_fr_inputs_ready) {
            mark_component_lost(&g_fr_inputs_ready, "FR_INPUT", "runtime sensor invalid");
        }
        if (step.new_fault_code == NODE_FAULT_MOTOR_COMM) {
            mark_component_lost(&g_stepper_steering_ready, "STEPPER_STEERING", "runtime motor communication fault");
            mark_component_lost(&g_stepper_braking_ready, "STEPPER_BRAKING", "runtime motor communication fault");
        }

        if (!all_required_components_ready()) {
            g_control_state = NODE_STATE_FAULT;
            g_fault_code = primary_fault_from_component_health();
        } else if (g_control_state == NODE_STATE_FAULT) {
            g_control_state = NODE_STATE_READY;
            g_fault_code = NODE_FAULT_NONE;
        }

        // Send immediate heartbeat on state change
        if (g_control_state != old_state) {
            send_control_heartbeat();
        }

        // Log control-local transitions only
#ifdef CONFIG_LOG_CONTROL_STATE_CHANGES
        if (g_control_state != prev_state) {
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
            else if (step.actions & CONTROL_ACTION_ATTEMPT_RECOVERY)
                reason = "attempt_recovery";

            ESP_LOGI(TAG, "State: %s -> %s (reason=%s)",
                     node_state_to_string(prev_state),
                     node_state_to_string(g_control_state),
                     reason);
        }
        prev_state = g_control_state;
#endif

#ifdef CONFIG_LOG_CONTROL_FAULT_CHANGES
        if (g_fault_code != prev_fault_code) {
            const char *detail = fault_detail_string(g_fault_code, fr_state, true);
            if (detail) {
                ESP_LOGI(TAG, "Fault: %s -> %s (detail=%s)",
                         node_fault_to_string(prev_fault_code),
                         node_fault_to_string(g_fault_code),
                         detail);
            } else {
                ESP_LOGI(TAG, "Fault: %s -> %s",
                         node_fault_to_string(prev_fault_code),
                         node_fault_to_string(g_fault_code));
            }
        }
        prev_fault_code = g_fault_code;
#endif

        // Log Safety mirrored state/fault changes separately
#ifdef CONFIG_LOG_CONTROL_SAFETY_MIRROR_CHANGES
        if (cmd_local.target_state != prev_target_state) {
            ESP_LOGI(TAG, "Safety target: %s -> %s",
                     node_state_to_string(prev_target_state),
                     node_state_to_string(cmd_local.target_state));
        }
        if (cmd_local.estop_fault_code != prev_estop_fault) {
            ESP_LOGI(TAG, "Safety estop fault: %s -> %s",
                     node_fault_to_string(prev_estop_fault),
                     node_fault_to_string(cmd_local.estop_fault_code));
        }

        prev_target_state = cmd_local.target_state;
        prev_estop_fault = cmd_local.estop_fault_code;
#endif

        vTaskDelay(CONTROL_LOOP_INTERVAL);
    }
}

// ============================================================================
// Heartbeat Task
// ============================================================================

void heartbeat_task(void *param) {
    led_ws2812_config_t *cfg = static_cast<led_ws2812_config_t *>(param);

    while (true) {
        TickType_t now = xTaskGetTickCount();

        update_control_led_mode();
        led_ws2812_tick(cfg, now);

        // Send heartbeat (periodic 100ms)
        send_control_heartbeat();

        vTaskDelay(HEARTBEAT_SEND_INTERVAL);
    }
}

// ============================================================================
// Main Task
// ============================================================================

void main_task(void *param) {
    (void)param;

    g_control_state = NODE_STATE_INIT;

    // Initialize TWAI once during startup. If it fails, mark FAULT and let
    // background per-component retries handle recovery.
    esp_err_t err = ESP_FAIL;
    err = can_twai_init_default(TWAI_TX_GPIO, TWAI_RX_GPIO);
    g_twai_ready = (err == ESP_OK);

    err = led_ws2812_init(&g_heartbeat_led_cfg);
    bool heartbeat_ready = (err == ESP_OK);
    if (heartbeat_ready) {
        led_ws2812_set_manual_mode(true);
        led_ws2812_set_manual_color(LED_LEVEL, LED_ORANGE_GREEN, 0, "state_init");
    }

    // Initialize heartbeat monitor to detect Safety heartbeat timeout.
    heartbeat_monitor_config_t hb_cfg = { .name = "CONTROL" };
    heartbeat_monitor_init(&g_hb_monitor, &hb_cfg);
    g_node_safety = heartbeat_monitor_register(&g_hb_monitor, "Safety", HEARTBEAT_TIMEOUT_MS);

    // Start CAN RX first so stepper init checks can receive responses.
    if (xTaskCreate(can_rx_task, "can_rx", CAN_RX_TASK_STACK, nullptr, CAN_RX_TASK_PRIO, &g_can_rx_task_handle) != pdPASS) {
        ESP_LOGE(TAG_INIT, "Failed to create CAN RX task, restarting");
        esp_restart();
    }

    // One startup attempt per component; retries happen later while faulted.
#ifndef CONFIG_BYPASS_ACTUATOR_MULTIPLEXER
    g_mux_ready = (multiplexer_dg408djz_init(&g_mux_cfg) == ESP_OK);
#else
    g_mux_ready = true;
#endif

#ifndef CONFIG_BYPASS_ACTUATOR_PEDAL_RELAY
    g_relay_ready = (relay_jd2912_init(&g_relay_cfg) == ESP_OK);
#else
    g_relay_ready = true;
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

#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_STEERING
    g_stepper_steering_ready =
        (init_stepper_checked(&g_steering_stepper, UIM2852_NODE_STEERING) == ESP_OK);
#else
    g_stepper_steering_ready = true;
#endif
#ifndef CONFIG_BYPASS_ACTUATOR_STEPPER_BRAKING
    g_stepper_braking_ready =
        (init_stepper_checked(&g_braking_stepper, UIM2852_NODE_BRAKING) == ESP_OK);
#else
    g_stepper_braking_ready = true;
#endif

    log_startup_device_status(g_twai_ready,
                              g_mux_ready,
                              g_relay_ready,
                              g_pedal_input_ready,
                              g_fr_inputs_ready,
                              g_stepper_steering_ready,
                              g_stepper_braking_ready,
                              heartbeat_ready);

    update_control_state_from_component_health();

    if (g_control_state == NODE_STATE_FAULT) {
        const char *detail = fault_detail_string(g_fault_code, fr_state_debounced(), true);
        if (detail) {
            ESP_LOGE(TAG_INIT,
                     "State: INIT -> FAULT (fault=%s detail=%s)",
                     node_fault_to_string(g_fault_code),
                     detail);
        } else {
            ESP_LOGE(TAG_INIT,
                     "State: INIT -> FAULT (fault=%s)",
                     node_fault_to_string(g_fault_code));
        }
    } else {
        // Set state to READY so control_task starts in READY (not INIT).
        // control_task owns all subsequent state transition logging.
        g_control_state = NODE_STATE_READY;
    }

    if (xTaskCreate(control_task, "control", CONTROL_TASK_STACK, nullptr, CONTROL_TASK_PRIO, nullptr) != pdPASS) {
        ESP_LOGE(TAG_INIT, "Failed to create control task, restarting");
        esp_restart();
    }
    if (xTaskCreate(heartbeat_task, "heartbeat", HEARTBEAT_TASK_STACK, &g_heartbeat_led_cfg, HEARTBEAT_TASK_PRIO, nullptr) != pdPASS) {
        ESP_LOGE(TAG_INIT, "Failed to create heartbeat task, restarting");
        esp_restart();
    }

    vTaskDelete(nullptr);
}
}

extern "C" void app_main(void) {
    if (xTaskCreate(main_task, "main_task", 8192, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE("CONTROL_INIT", "Failed to create main_task, restarting");
        esp_restart();
    }
}
