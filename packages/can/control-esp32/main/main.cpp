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
#include "esp_adc/adc_oneshot.h"

#include "can_protocol.hh"
#include "can_twai.hh"
#include "heartbeat.hh"
#include "multiplexer_dg408djz.hh"
#include "stepper_motor_uim2852.h"
#include "relay_jd2912.hh"
#include "override_sensors.hh"
#include "control_logic.h"



// Control ESP32 - Autonomous control execution
//
// Receives commands from Planner via CAN and actuates throttle, steering, and braking.
// Safety ESP32 commands system state via its heartbeat (0x100). Control reacts
// to the target_state: starts enable sequence when target >= ENABLING, transitions to
// ACTIVE when target == ACTIVE (after enable work is done), and retreats to READY when
// target drops below its current state.
//
// State machine:
//   INIT -> READY -> ENABLING -> ACTIVE
//                        |          |
//                        v          v
//                      FAULT     OVERRIDE
//                        |          |
//                        +-> READY <+
//
// Recovery: FAULT state is now recoverable. After a cooldown, the system attempts
// component-level re-init. If recovery fails after MAX_RECOVERY_ATTEMPTS, esp_restart().

namespace {

static const char *TAG = "CONTROL";
static const char *TAG_TX = "CONTROL_TX";
static const char *TAG_RX __attribute__((unused)) = "CONTROL_RX";

// ============================================================================
// Task Configuration
// ============================================================================

constexpr int CAN_RX_TASK_STACK = 4096;
constexpr int CONTROL_TASK_STACK = 4096;
constexpr int HEARTBEAT_TASK_STACK = 4096;
constexpr UBaseType_t CAN_RX_TASK_PRIO = 7;
constexpr UBaseType_t CONTROL_TASK_PRIO = 6;
constexpr UBaseType_t HEARTBEAT_TASK_PRIO = 4;

// ============================================================================
// Timing Constants
// ============================================================================

constexpr TickType_t CAN_RX_TIMEOUT = pdMS_TO_TICKS(10);
constexpr TickType_t CONTROL_LOOP_INTERVAL = pdMS_TO_TICKS(20);
constexpr TickType_t HEARTBEAT_SEND_INTERVAL = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS);
constexpr uint32_t ENABLE_SEQUENCE_MS = 200;
constexpr uint32_t THROTTLE_SLEW_INTERVAL_MS = 100;  // max 1 level change per 100ms
constexpr TickType_t PLANNER_CMD_TIMEOUT = pdMS_TO_TICKS(500);

// ============================================================================
// Init Retry & Recovery Constants
// ============================================================================

constexpr uint8_t INIT_MAX_RETRIES = 3;
constexpr uint32_t INIT_RETRY_DELAY_MS = 1000;
constexpr uint8_t CAN_TX_FAIL_THRESHOLD = 5;
constexpr uint8_t MAX_RECOVERY_ATTEMPTS = 3;
constexpr uint32_t RECOVERY_COOLDOWN_MS = 2000;
constexpr uint32_t RESTART_DELAY_MS = 3000;  // delay before esp_restart to flush logs

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

// F/R optocouplers (PC817) - detect gear selector position
constexpr gpio_num_t FR_DIR_GPIO = GPIO_NUM_14;   // direction enable optocoupler
constexpr gpio_num_t FR_REV_GPIO = GPIO_NUM_15;   // reverse optocoupler

// Status LED (WS2812)
constexpr gpio_num_t HEARTBEAT_LED_GPIO = GPIO_NUM_8;
constexpr uint8_t LED_LEVEL = 16;
constexpr uint8_t LED_ORANGE_GREEN = 8;

// Throttle limits
constexpr int8_t THROTTLE_LEVEL_MAX = 7;

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

static override_sensors_config_t g_override_sensors_cfg = {
    .adc_unit = ADC_UNIT_1,
    .pedal_adc_channel = PEDAL_ADC_CHANNEL,
    .fr_dir_gpio = FR_DIR_GPIO,
    .fr_rev_gpio = FR_REV_GPIO,
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

// Timing state
static uint32_t g_enable_start_ms = 0;
static uint32_t g_last_throttle_change_ms = 0;
static bool g_enable_work_done = false;

// Heartbeat
static volatile uint8_t g_heartbeat_seq = 0;

// Spinlock for heartbeat sequence (shared by control_task immediate send + heartbeat_task)
static portMUX_TYPE g_hb_seq_lock = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// Recovery State
// ============================================================================

struct recovery_state_t {
    uint8_t can_tx_fail_count;        // consecutive CAN TX failures
    uint8_t recovery_attempts;        // fault recovery attempts
    uint32_t last_recovery_ms;        // cooldown tracking

};

static recovery_state_t g_recovery = {};
static bool g_can_recovery_in_progress = false;

// Spinlock for CAN TX tracking / recovery (used by control_task + heartbeat_task)
static portMUX_TYPE g_can_tx_lock = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// Stepper Motor Instances (UIM2852CA)
// ============================================================================

static stepper_motor_uim2852_t g_steering_stepper = {};
static stepper_motor_uim2852_t g_braking_stepper = {};

static uint32_t get_time_ms() {
    return (uint32_t)(esp_timer_get_time() / 1000);
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

    heartbeat_set_manual_color(red, green, blue, reason);
}

// ============================================================================
// Init Retry Helper
// ============================================================================

// Retry an init function up to INIT_MAX_RETRIES times with delay between attempts.
// Returns ESP_OK on success, or last error on exhaustion.
template <typename ConfigT>
static esp_err_t retry_init(const char *name,
                            esp_err_t (*fn)(const ConfigT *),
                            const ConfigT *config) {
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= INIT_MAX_RETRIES; attempt++) {
        err = fn(config);
        if (err == ESP_OK) {
            if (attempt > 1)
                ESP_LOGI(TAG, "%s init succeeded on attempt %d", name, attempt);
            return ESP_OK;
        }
        ESP_LOGI(TAG, "%s init failed (attempt %d/%d): %s",
                 name, attempt, INIT_MAX_RETRIES, esp_err_to_name(err));
        if (attempt < INIT_MAX_RETRIES)
            vTaskDelay(pdMS_TO_TICKS(INIT_RETRY_DELAY_MS));
    }
    ESP_LOGE(TAG, "%s init FAILED after %d attempts", name, INIT_MAX_RETRIES);
    return err;
}

// ============================================================================
// CAN Recovery Helper
// ============================================================================

// Attempt to recover CAN bus communication.
// First tries stop/start cycle, then full driver reinstall.
// Calls esp_restart() if all recovery fails.
static void attempt_can_recovery() {
#ifdef CONFIG_LOG_CAN_RECOVERY
    ESP_LOGI(TAG, "Attempting CAN recovery (stop/start cycle)");
#endif
    // Safe to call while CAN RX task is blocked in twai_receive() —
    // twai_stop() causes twai_receive() to return ESP_ERR_INVALID_STATE,
    // which the RX loop handles gracefully (continues to next iteration).
    twai_stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_err_t err = twai_start();
    if (err == ESP_OK) {
#ifdef CONFIG_LOG_CAN_RECOVERY
        ESP_LOGI(TAG, "CAN recovery succeeded (stop/start)");
#endif
        g_recovery.can_tx_fail_count = 0;
        return;
    }

    // Escalate: full driver reinstall
#ifdef CONFIG_LOG_CAN_RECOVERY
    ESP_LOGI(TAG, "CAN stop/start failed, attempting full re-init");
#endif
    twai_driver_uninstall();
    vTaskDelay(pdMS_TO_TICKS(200));
    err = can_twai_init_default(TWAI_TX_GPIO, TWAI_RX_GPIO);
    if (err == ESP_OK) {
#ifdef CONFIG_LOG_CAN_RECOVERY
        ESP_LOGI(TAG, "CAN full re-init succeeded");
#endif
        g_recovery.can_tx_fail_count = 0;
        return;
    }

    ESP_LOGE(TAG, "CAN recovery FAILED, restarting ESP in %lu ms", (unsigned long)RESTART_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
    esp_restart();
}

// Track CAN TX result. On consecutive failures, trigger recovery.
// Protected by g_can_tx_lock to prevent concurrent recovery from control_task + heartbeat_task.
static void track_can_tx(esp_err_t err) {
    bool trigger_recovery = false;

    taskENTER_CRITICAL(&g_can_tx_lock);
    can_tx_track_inputs_t t = {
        .fail_count = g_recovery.can_tx_fail_count,
        .threshold = CAN_TX_FAIL_THRESHOLD,
        .tx_ok = (err == ESP_OK),
    };
    can_tx_track_result_t r = control_track_can_tx(&t);
    g_recovery.can_tx_fail_count = r.new_fail_count;
    if (r.trigger_recovery) {
        bool bus_ok = can_twai_bus_ok();
        g_recovery.can_tx_fail_count = 0;  // reset regardless
        if (!bus_ok) {
            if (!g_can_recovery_in_progress) {
                g_can_recovery_in_progress = true;
                trigger_recovery = true;
            }
        } else {
#ifdef CONFIG_LOG_CAN_RECOVERY
            ESP_LOGI(TAG, "CAN TX failed %d times (bus OK, no partner?)",
                     CAN_TX_FAIL_THRESHOLD);
#endif
        }
    }
    taskEXIT_CRITICAL(&g_can_tx_lock);

    if (trigger_recovery) {
        ESP_LOGE(TAG, "CAN bus unhealthy after %d TX failures, initiating recovery",
                 CAN_TX_FAIL_THRESHOLD);
        attempt_can_recovery();
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
    if (err != ESP_OK) {
        ESP_LOGE(TAG_TX, "Heartbeat failed: %s", esp_err_to_name(err));
    }
#ifdef CONFIG_LOG_HEARTBEAT_TX
    else {
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
#ifndef CONFIG_BYPASS_MULTIPLEXER
    multiplexer_dg408djz_disable();
    multiplexer_dg408djz_emergency_stop();
#endif
#ifndef CONFIG_BYPASS_ENABLE_RELAY
    relay_jd2912_deenergize();
#endif
#ifndef CONFIG_BYPASS_STEPPER_MOTORS
    stepper_motor_uim2852_emergency_stop(&g_steering_stepper);
    stepper_motor_uim2852_emergency_stop(&g_braking_stepper);
    stepper_motor_uim2852_disable(&g_steering_stepper);
    stepper_motor_uim2852_disable(&g_braking_stepper);
#endif
}

// Immediately disable all autonomous actuators.
static void execute_trigger_override(uint8_t reason) {
#ifdef CONFIG_LOG_OVERRIDE
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
#if defined(CONFIG_LOG_STATE_CHANGES) || defined(CONFIG_LOG_ENABLE_SEQUENCE)
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
#ifdef CONFIG_LOG_ENABLE_SEQUENCE
    ESP_LOGI(TAG, "Starting autonomous enable sequence");
#endif
#ifndef CONFIG_BYPASS_MULTIPLEXER
    multiplexer_dg408djz_set_level(0);
#endif
#ifndef CONFIG_BYPASS_ENABLE_RELAY
    relay_jd2912_energize();
#endif
}

// Complete the enable sequence (enable steppers, switch mux to autonomous)
static void execute_complete_enable() {
#ifdef CONFIG_LOG_ENABLE_SEQUENCE
    ESP_LOGI(TAG, "Completing autonomous enable sequence");
#endif
#ifndef CONFIG_BYPASS_STEPPER_MOTORS
    esp_err_t steer_err = stepper_motor_uim2852_enable(&g_steering_stepper);
#ifdef CONFIG_LOG_ENABLE_SEQUENCE
    if (steer_err != ESP_OK) ESP_LOGI(TAG, "Steering enable failed: %s", esp_err_to_name(steer_err));
#endif
    vTaskDelay(pdMS_TO_TICKS(5));
    esp_err_t brake_err = stepper_motor_uim2852_enable(&g_braking_stepper);
#ifdef CONFIG_LOG_ENABLE_SEQUENCE
    if (brake_err != ESP_OK) ESP_LOGI(TAG, "Braking enable failed: %s", esp_err_to_name(brake_err));
#endif
    
    if (steer_err != ESP_OK || brake_err != ESP_OK) {
        ESP_LOGE(TAG, "Stepper enable failed, aborting autonomous enable");
        stepper_motor_uim2852_disable(&g_steering_stepper);
        stepper_motor_uim2852_disable(&g_braking_stepper);
#ifndef CONFIG_BYPASS_ENABLE_RELAY
        relay_jd2912_deenergize();
#endif
        multiplexer_dg408djz_disable();
        return;
    }
#endif

#ifndef CONFIG_BYPASS_MULTIPLEXER
    multiplexer_dg408djz_enable_autonomous();
#endif
#ifdef CONFIG_LOG_ENABLE_SEQUENCE
    ESP_LOGI(TAG, "AUTONOMOUS MODE ACTIVE");
#endif
}

// Abort the enable sequence (de-energize relay, disable mux)
static void execute_abort_enable() {
#ifdef CONFIG_LOG_ENABLE_SEQUENCE
    ESP_LOGI(TAG, "Enable sequence aborted");
#endif
#ifndef CONFIG_BYPASS_ENABLE_RELAY
    relay_jd2912_deenergize();
#endif
#ifndef CONFIG_BYPASS_MULTIPLEXER
    multiplexer_dg408djz_disable();
#endif
}

// ============================================================================
// Fault Recovery
// ============================================================================

// Attempt to recover from the current fault by re-initializing the failed component.
// Returns true if recovery succeeded and fault was cleared.
static bool attempt_fault_recovery() {
    uint32_t now_ms = get_time_ms();

    // Enforce cooldown between recovery attempts
    if ((now_ms - g_recovery.last_recovery_ms) < RECOVERY_COOLDOWN_MS) return false;

    g_recovery.last_recovery_ms = now_ms;
    g_recovery.recovery_attempts++;

#ifdef CONFIG_LOG_RECOVERY
    ESP_LOGI(TAG, "Recovery attempt %d/%d for fault: %s",
             g_recovery.recovery_attempts, MAX_RECOVERY_ATTEMPTS,
             node_fault_to_string(g_fault_code));
#endif

    esp_err_t err = ESP_FAIL;

    switch (g_fault_code) {
        case NODE_FAULT_MOTOR_COMM:
            // Clear motor error flags and re-configure
            g_steering_stepper.status.error_detected = false;
            g_steering_stepper.status.stall_detected = false;
            g_braking_stepper.status.error_detected = false;
            g_braking_stepper.status.stall_detected = false;
            stepper_motor_uim2852_clear_status(&g_steering_stepper);
            vTaskDelay(pdMS_TO_TICKS(10));
            stepper_motor_uim2852_clear_status(&g_braking_stepper);
            vTaskDelay(pdMS_TO_TICKS(10));
            err = stepper_motor_uim2852_configure(&g_steering_stepper);
            if (err == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(10));
                err = stepper_motor_uim2852_configure(&g_braking_stepper);
            }
            break;

        case NODE_FAULT_THROTTLE_INIT:
            err = multiplexer_dg408djz_init(&g_mux_cfg);
            break;

        case NODE_FAULT_RELAY_INIT:
            err = relay_jd2912_init(&g_relay_cfg);
            break;

        case NODE_FAULT_SENSOR_INVALID:
            // Sensor invalid may be transient (wiring glitch) - try re-init
            err = override_sensors_init(&g_override_sensors_cfg);
            break;

        case NODE_FAULT_CAN_TX:
            // CAN recovery is handled separately by attempt_can_recovery()
            attempt_can_recovery();
            err = ESP_OK;  // if we get here, recovery succeeded (otherwise esp_restart was called)
            break;

        default:
#ifdef CONFIG_LOG_RECOVERY
            ESP_LOGI(TAG, "No recovery handler for fault 0x%02X", g_fault_code);
#endif
            break;
    }

    if (err == ESP_OK) {
#ifdef CONFIG_LOG_RECOVERY
        ESP_LOGI(TAG, "Recovery SUCCEEDED for fault: %s", node_fault_to_string(g_fault_code));
#endif
        g_fault_code = NODE_FAULT_NONE;
        g_control_state = NODE_STATE_READY;
        g_recovery.recovery_attempts = 0;
        return true;
    }

#ifdef CONFIG_LOG_RECOVERY
    ESP_LOGI(TAG, "Recovery attempt %d FAILED: %s", g_recovery.recovery_attempts, esp_err_to_name(err));
#endif

    if (g_recovery.recovery_attempts >= MAX_RECOVERY_ATTEMPTS) {
        ESP_LOGE(TAG, "Recovery exhausted (%d attempts), restarting ESP in %lu ms",
                 MAX_RECOVERY_ATTEMPTS, (unsigned long)RESTART_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
        esp_restart();
    }

    return false;
}

// ============================================================================
// CAN RX Task
// ============================================================================

void can_rx_task(void *param) {
    (void)param;
    twai_message_t msg{};

    ESP_LOGI(TAG, "CAN RX task started");

    while (true) {
        if (can_twai_receive(&msg, CAN_RX_TIMEOUT) != ESP_OK) continue;

        if (msg.rtr) continue;

        TickType_t now = xTaskGetTickCount();

#ifndef CONFIG_BYPASS_STEPPER_MOTORS
        // Handle extended frames from UIM2852CA stepper motors
        if (msg.extd) {
            bool steering_match = stepper_motor_uim2852_process_frame(&g_steering_stepper, &msg);
            if (steering_match) {
                if (stepper_motor_uim2852_stall_detected(&g_steering_stepper) ||
                    stepper_motor_uim2852_has_error(&g_steering_stepper)) {
#ifdef CONFIG_LOG_STEPPER_COMMANDS
                    ESP_LOGI(TAG_RX, "Steering stepper motor fault detected");
#endif
                    taskENTER_CRITICAL(&g_cmd_lock);
                    g_cmd.motor_fault_code = NODE_FAULT_MOTOR_COMM;
                    taskEXIT_CRITICAL(&g_cmd_lock);
                }
            } else {
                bool braking_match = stepper_motor_uim2852_process_frame(&g_braking_stepper, &msg);
                if (braking_match) {
                    if (stepper_motor_uim2852_stall_detected(&g_braking_stepper) ||
                        stepper_motor_uim2852_has_error(&g_braking_stepper)) {
#ifdef CONFIG_LOG_STEPPER_COMMANDS
                        ESP_LOGI(TAG_RX, "Braking stepper motor fault detected");
#endif
                        taskENTER_CRITICAL(&g_cmd_lock);
                        g_cmd.motor_fault_code = NODE_FAULT_MOTOR_COMM;
                        taskEXIT_CRITICAL(&g_cmd_lock);
                    }
                }
            }
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
            if (throttle_level > THROTTLE_LEVEL_MAX) throttle_level = THROTTLE_LEVEL_MAX;

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
            
            heartbeat_mark_activity(now);

#ifdef CONFIG_LOG_PLANNER_COMMANDS
            ESP_LOGI(TAG_RX, "CMD: thr=%d steer=%d brake=%d seq=%u",
                throttle_level, cmd.steering_position, cmd.braking_position, cmd.sequence);
#endif
        }

        // Process Safety heartbeat (0x100)
        else if (msg.identifier == CAN_ID_SAFETY_HEARTBEAT && msg.data_length_code >= 4) {
            node_heartbeat_t hb;
            can_decode_heartbeat(msg.data, &hb);

            uint8_t new_target = hb.state;          // Safety's state = system target
            uint8_t fault_code = hb.fault_code;     // Safety's fault = estop reason

            taskENTER_CRITICAL(&g_cmd_lock);
            g_cmd.target_state = new_target;
            g_cmd.estop_fault_code = fault_code;
            taskEXIT_CRITICAL(&g_cmd_lock);
            
            heartbeat_mark_activity(now);

#ifdef CONFIG_LOG_HEARTBEAT_RX
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

    ESP_LOGI(TAG, "Control task started");

    // Declare loop-state variables before any goto to satisfy C++ scoping rules
#ifdef CONFIG_LOG_STATE_CHANGES
    uint8_t prev_state = 0xFF;
    uint8_t prev_fault_code = 0xFF;
#endif
#ifdef CONFIG_LOG_SAFETY_MIRROR_CHANGES
    uint8_t prev_target_state = 0xFF;
    uint8_t prev_estop_fault = 0xFF;
#endif
    int16_t last_steering_sent = INT16_MIN;
    int16_t last_braking_sent = INT16_MIN;

#ifndef CONFIG_BYPASS_STEPPER_MOTORS
    // Initialize UIM2852CA stepper motors
    stepper_motor_uim2852_config_t steer_cfg = STEPPER_MOTOR_UIM2852_CONFIG_DEFAULT();
    steer_cfg.node_id = UIM2852_NODE_STEERING;
    esp_err_t steer_init_err = stepper_motor_uim2852_init(&g_steering_stepper, &steer_cfg);
    if (steer_init_err != ESP_OK) {
        ESP_LOGE(TAG, "Steering stepper init failed");
    }

    stepper_motor_uim2852_config_t brake_cfg = STEPPER_MOTOR_UIM2852_CONFIG_DEFAULT();
    brake_cfg.node_id = UIM2852_NODE_BRAKING;
    esp_err_t brake_init_err = stepper_motor_uim2852_init(&g_braking_stepper, &brake_cfg);
    if (brake_init_err != ESP_OK) {
        ESP_LOGE(TAG, "Braking stepper init failed");
    }

    // If either init failed, go straight to FAULT
    if (steer_init_err != ESP_OK || brake_init_err != ESP_OK) {
        g_fault_code = NODE_FAULT_MOTOR_COMM;
        g_control_state = NODE_STATE_FAULT;
        ESP_LOGE(TAG, "Stepper init failed, entering FAULT state");
        // Still enter main loop for recovery attempts
        goto control_loop;
    }

    esp_err_t cfg_err;
    cfg_err = stepper_motor_uim2852_configure(&g_steering_stepper);
    if (cfg_err != ESP_OK) {
        ESP_LOGE(TAG, "Steering stepper configure failed");
        g_fault_code = NODE_FAULT_MOTOR_COMM;
        g_control_state = NODE_STATE_FAULT;
        goto control_loop;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    cfg_err = stepper_motor_uim2852_configure(&g_braking_stepper);
    if (cfg_err != ESP_OK) {
        ESP_LOGE(TAG, "Braking stepper configure failed");
        g_fault_code = NODE_FAULT_MOTOR_COMM;
        g_control_state = NODE_STATE_FAULT;
        goto control_loop;
    }
    vTaskDelay(pdMS_TO_TICKS(10));

    stepper_motor_uim2852_disable(&g_steering_stepper);
    stepper_motor_uim2852_disable(&g_braking_stepper);
#else
    ESP_LOGI(TAG, "Stepper motors BYPASSED (CONFIG_BYPASS_STEPPER_MOTORS)");
#endif

    g_throttle_current = 0;

    g_control_state = NODE_STATE_READY;
    ESP_LOGI(TAG, "Control ready, waiting for commands");

#ifdef CONFIG_LOG_STATE_CHANGES
    prev_state = g_control_state;
    prev_fault_code = g_fault_code;
#endif
#ifdef CONFIG_LOG_SAFETY_MIRROR_CHANGES
    prev_target_state = g_cmd.target_state;
    prev_estop_fault = g_cmd.estop_fault_code;
#endif

#ifndef CONFIG_BYPASS_STEPPER_MOTORS
control_loop:
#endif
    while (true) {
        uint32_t now_ms = get_time_ms();

        // Take a thread-safe snapshot of CAN RX state
        command_snapshot_t cmd_local;
        taskENTER_CRITICAL(&g_cmd_lock);
        cmd_local = g_cmd;
        // Clear one-shot motor fault flag after reading
        g_cmd.motor_fault_code = NODE_FAULT_NONE;
        taskEXIT_CRITICAL(&g_cmd_lock);

        override_sensors_update(now_ms);
        fr_state_t fr_state = override_sensors_get_fr_state();

        // Apply test bypasses
#ifdef CONFIG_BYPASS_FR_SENSOR
        fr_state = FR_STATE_FORWARD;
#endif
#ifdef CONFIG_BYPASS_SAFETY_TARGET_STATE
        cmd_local.target_state = NODE_STATE_ACTIVE;
#endif
#ifdef CONFIG_BYPASS_SAFETY_ESTOP_FAULT
        cmd_local.estop_fault_code = NODE_FAULT_NONE;
#endif
#ifdef CONFIG_BYPASS_PLANNER_CMD_INPUTS
        cmd_local.throttle_target = 0;
        cmd_local.steering_cmd = 0;
        cmd_local.braking_cmd = 0;
#endif

        // Check Planner command freshness - zero throttle if stale
#ifndef CONFIG_BYPASS_PLANNER_CMD_STALE_CHECK
        TickType_t planner_age = xTaskGetTickCount() - g_last_planner_cmd_tick;
        bool planner_cmd_stale = false;

        if (g_control_state == NODE_STATE_ACTIVE && g_last_planner_cmd_tick > 0) {
            // Timeout: no command received for PLANNER_CMD_TIMEOUT
            if (planner_age > PLANNER_CMD_TIMEOUT) {
                planner_cmd_stale = true;
#ifdef CONFIG_LOG_PLANNER_COMMANDS
                ESP_LOGI(TAG, "Planner command timeout (%lu ms), zeroing throttle",
                         (unsigned long)(planner_age * portTICK_PERIOD_MS));
#endif
            }
            // Stale sequence: same sequence seen PLANNER_CMD_STALE_COUNT times
            else if (g_planner_cmd_stale_count >= PLANNER_CMD_STALE_COUNT) {
                planner_cmd_stale = true;
#ifdef CONFIG_LOG_PLANNER_COMMANDS
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
        bool pedal_pressed = override_sensors_pedal_pressed();
        bool pedal_rearmed = override_sensors_pedal_rearmed();
#ifdef CONFIG_BYPASS_PEDAL_OVERRIDE
        pedal_pressed = false;
        pedal_rearmed = true;
#endif

        control_inputs_t step_in = {
            .target_state = cmd_local.target_state,
            .throttle_target = cmd_local.throttle_target,
            .steering_cmd = cmd_local.steering_cmd,
            .braking_cmd = cmd_local.braking_cmd,
            .motor_fault_code = cmd_local.motor_fault_code,
            .fr_state = fr_state,
            .pedal_pressed = pedal_pressed,
            .pedal_rearmed = pedal_rearmed,
            .fr_is_invalid = (fr_state == FR_STATE_INVALID),
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

#ifdef CONFIG_LOG_STATE_TICK
        ESP_LOGI(TAG, "tick state=%s target=%s estop=%s fr=%d pedal=%d/%d thr=%d/%d actions=0x%02X -> %s",
                 node_state_to_string(g_control_state),
                 node_state_to_string(cmd_local.target_state),
                 node_fault_to_string(cmd_local.estop_fault_code),
                 fr_state, pedal_pressed, pedal_rearmed,
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
#if !defined(CONFIG_BYPASS_STEPPER_MOTORS) && !defined(CONFIG_BYPASS_MULTIPLEXER)
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
#ifndef CONFIG_BYPASS_MULTIPLEXER
            multiplexer_dg408djz_set_level(step.throttle_level);
#endif
#ifdef CONFIG_LOG_THROTTLE
            {
                static int8_t prev_level = -1;
                if (step.throttle_level != prev_level) {
                    ESP_LOGI(TAG, "Throttle level: %d -> %d", prev_level, step.throttle_level);
                    prev_level = step.throttle_level;
                }
            }
#endif
        }

#ifdef CONFIG_LOG_THROTTLE_TICK
        ESP_LOGI(TAG, "Throttle tick: current=%d target=%d",
                 step.throttle_level, cmd_local.throttle_target);
#endif

        if (step.actions & CONTROL_ACTION_ATTEMPT_RECOVERY) {
            if (attempt_fault_recovery()) {
                // Recovery succeeded — override step result so the state update
                // block below applies the recovered state instead of re-entering FAULT.
                step.new_state = g_control_state;       // READY
                step.new_fault_code = g_fault_code;     // NONE
            }
        }

        // Send stepper commands when position changed
#ifndef CONFIG_BYPASS_STEPPER_MOTORS
        if (step.send_steering) {
            esp_err_t err = stepper_motor_uim2852_go_absolute(&g_steering_stepper, step.steering_position);
            if (err != ESP_OK) {
                step.new_last_steering = last_steering_sent;  // keep old value on failure
            }
#ifdef CONFIG_LOG_STEPPER_COMMANDS
            if (err != ESP_OK) {
                ESP_LOGI(TAG_TX, "Steering cmd failed: %s", esp_err_to_name(err));
            } else {
                ESP_LOGI(TAG_TX, "Steering -> %d", step.steering_position);
            }
#endif
            track_can_tx(err);
        }
        if (step.send_braking) {
            esp_err_t err = stepper_motor_uim2852_go_absolute(&g_braking_stepper, step.braking_position);
            if (err != ESP_OK) {
                step.new_last_braking = last_braking_sent;  // keep old value on failure
            }
#ifdef CONFIG_LOG_STEPPER_COMMANDS
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
            g_recovery.recovery_attempts = 0;
            ESP_LOGE(TAG, "Entering FAULT state: %s", node_fault_to_string(step.new_fault_code));
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

        // Send immediate heartbeat on state change
        if (g_control_state != old_state) {
            send_control_heartbeat();
        }

        // Log control-local transitions only
#ifdef CONFIG_LOG_STATE_CHANGES
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
        if (g_fault_code != prev_fault_code) {
            ESP_LOGI(TAG, "Fault: %s -> %s",
                     node_fault_to_string(prev_fault_code),
                     node_fault_to_string(g_fault_code));
        }

        prev_state = g_control_state;
        prev_fault_code = g_fault_code;
#endif

        // Log Safety mirrored state/fault changes separately
#ifdef CONFIG_LOG_SAFETY_MIRROR_CHANGES
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
    heartbeat_config_t *cfg = static_cast<heartbeat_config_t *>(param);

    ESP_LOGI(TAG, "Heartbeat task started");

    while (true) {
        TickType_t now = xTaskGetTickCount();

        heartbeat_set_manual_mode(true);
        update_control_led_mode();
        heartbeat_tick(cfg, now);

        // Send heartbeat (periodic 100ms)
        send_control_heartbeat();

        // Check CAN bus health — recovery must happen OUTSIDE critical section
        // because can_twai_recover_bus_off() calls vTaskDelay() internally.
        taskENTER_CRITICAL(&g_can_tx_lock);
        bool bus_unhealthy = !can_twai_bus_ok();
        bool trigger_recovery = bus_unhealthy && !g_can_recovery_in_progress;
        if (trigger_recovery) g_can_recovery_in_progress = true;
        taskEXIT_CRITICAL(&g_can_tx_lock);
        if (trigger_recovery) {
#ifdef CONFIG_LOG_CAN_RECOVERY
            ESP_LOGI(TAG, "CAN bus unhealthy, attempting recovery");
#endif
            can_twai_recover_bus_off();
            taskENTER_CRITICAL(&g_can_tx_lock);
            g_can_recovery_in_progress = false;
            taskEXIT_CRITICAL(&g_can_tx_lock);
        }

        vTaskDelay(HEARTBEAT_SEND_INTERVAL);
    }
}

// ============================================================================
// Main Task
// ============================================================================

void main_task(void *param) {
    (void)param;

    ESP_LOGI(TAG, "Control ESP32 startup");
#ifdef CONFIG_IDF_TARGET
    ESP_LOGI(TAG, "Target: %s", CONFIG_IDF_TARGET);
#endif

    // Initialize TWAI (via WAVESHARE SN65HVD230 transceiver)
    // TWAI is critical - retry, then restart on failure.
    // Can't use retry_init helper because can_twai_init_default takes 2 params.
    esp_err_t err = ESP_FAIL;
    for (int i = 0; i < INIT_MAX_RETRIES; i++) {
        err = can_twai_init_default(TWAI_TX_GPIO, TWAI_RX_GPIO);
        if (err == ESP_OK) {
            if (i > 0) ESP_LOGI(TAG, "TWAI init succeeded on attempt %d", i + 1);
            break;
        }
        ESP_LOGI(TAG, "TWAI init failed (attempt %d/%d): %s", i + 1, INIT_MAX_RETRIES, esp_err_to_name(err));
        if (i < INIT_MAX_RETRIES - 1) vTaskDelay(pdMS_TO_TICKS(INIT_RETRY_DELAY_MS));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI init FAILED after %d retries, restarting ESP", INIT_MAX_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
        esp_restart();
    }

    // Initialize components with retry
    bool critical_failure = false;

    err = retry_init("Multiplexer", multiplexer_dg408djz_init, &g_mux_cfg);
    if (err != ESP_OK) {
        g_fault_code = NODE_FAULT_THROTTLE_INIT;
        critical_failure = true;
    }

    err = retry_init("Pedal bypass relay", relay_jd2912_init, &g_relay_cfg);
    if (err != ESP_OK) {
        g_fault_code = NODE_FAULT_RELAY_INIT;
        critical_failure = true;
    }

    err = retry_init("Override sensors", override_sensors_init, &g_override_sensors_cfg);
    if (err != ESP_OK) {
        g_fault_code = NODE_FAULT_SENSOR_INVALID;
        critical_failure = true;
    }

    // If any critical component failed after retries, restart
    if (critical_failure) {
        ESP_LOGE(TAG, "Critical component init failed (fault: %s), restarting ESP in %lu ms",
                 node_fault_to_string(g_fault_code), (unsigned long)RESTART_DELAY_MS);
        vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
        esp_restart();
    }

    static heartbeat_config_t heartbeat_cfg = {
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

    err = heartbeat_init(&heartbeat_cfg);
    if (err != ESP_OK)
        ESP_LOGI(TAG, "Heartbeat LED init failed: %s", esp_err_to_name(err));
    else
        heartbeat_set_manual_mode(true);

    // Log active bypasses
#ifdef CONFIG_BYPASS_SAFETY_TARGET_STATE
    ESP_LOGW(TAG, "BYPASS: Safety target state DISABLED (forcing ACTIVE)");
#endif
#ifdef CONFIG_BYPASS_SAFETY_ESTOP_FAULT
    ESP_LOGW(TAG, "BYPASS: Safety e-stop fault DISABLED (forcing clear)");
#endif
#ifdef CONFIG_BYPASS_FR_SENSOR
    ESP_LOGW(TAG, "BYPASS: F/R sensor DISABLED (forcing Forward)");
#endif
#ifdef CONFIG_BYPASS_PLANNER_CMD_INPUTS
    ESP_LOGW(TAG, "BYPASS: Planner command inputs DISABLED (forcing zero cmds)");
#endif
#ifdef CONFIG_BYPASS_PLANNER_CMD_STALE_CHECK
    ESP_LOGW(TAG, "BYPASS: Planner command stale checks DISABLED");
#endif
#ifdef CONFIG_BYPASS_STEPPER_MOTORS
    ESP_LOGW(TAG, "BYPASS: Stepper motors DISABLED");
#endif
#ifdef CONFIG_BYPASS_PEDAL_OVERRIDE
    ESP_LOGW(TAG, "BYPASS: Pedal override detection DISABLED");
#endif
#ifdef CONFIG_BYPASS_ENABLE_RELAY
    ESP_LOGW(TAG, "BYPASS: Pedal bypass relay DISABLED");
#endif
#ifdef CONFIG_BYPASS_MULTIPLEXER
    ESP_LOGW(TAG, "BYPASS: Throttle mux DISABLED");
#endif

    ESP_LOGI(TAG, "Initial F/R state: %d", override_sensors_get_fr_state());
    ESP_LOGI(TAG, "Initial pedal: %u mV (%s)", 
        override_sensors_get_pedal_mv(),
        override_sensors_pedal_pressed() ? "PRESSED" : "released");

    ESP_LOGI(TAG, "Starting control tasks");

    if (xTaskCreate(can_rx_task, "can_rx", CAN_RX_TASK_STACK, nullptr, CAN_RX_TASK_PRIO, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN RX task, restarting");
        esp_restart();
    }
    if (xTaskCreate(control_task, "control", CONTROL_TASK_STACK, nullptr, CONTROL_TASK_PRIO, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create control task, restarting");
        esp_restart();
    }
    if (xTaskCreate(heartbeat_task, "heartbeat", HEARTBEAT_TASK_STACK, &heartbeat_cfg, HEARTBEAT_TASK_PRIO, nullptr) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create heartbeat task, restarting");
        esp_restart();
    }

    vTaskDelete(nullptr);
}
}

extern "C" void app_main(void) {
    if (xTaskCreate(main_task, "main_task", 4096, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE("CONTROL", "Failed to create main_task, restarting");
        esp_restart();
    }
}
