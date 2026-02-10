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
#include "throttle_mux.hh"
#include "stepper_motor_uim2852.h"
#include "enable_relay.hh"
#include "override_sensors.hh"
#include "control_logic.h"

// Control ESP32 - Autonomous control execution
//
// Receives commands from Orin via CAN and actuates throttle, steering, and braking.
// Safety ESP32 must grant autonomous permission (CAN_ID_SAFETY_AUTO_ALLOWED) before
// this node will enable actuators. Human override via pedal or F/R switch immediately
// disables autonomous mode and returns control to manual.
//
// State machine: INIT -> READY -> ENABLING -> ACTIVE -> OVERRIDE -> READY
//                                    |                      ^
//                                    +-------(fault)--------+
//
// Recovery: FAULT state is now recoverable. After a cooldown, the system attempts
// component-level re-init. If recovery fails after MAX_RECOVERY_ATTEMPTS, esp_restart().

namespace {

static const char *TAG = "CONTROL";

// ============================================================================
// Task Configuration
// ============================================================================

constexpr int CAN_RX_TASK_STACK = 4096;
constexpr int CONTROL_TASK_STACK = 4096;
constexpr int HEARTBEAT_TASK_STACK = 2048;
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

// Throttle limits
constexpr int8_t THROTTLE_LEVEL_MIN = 0;
constexpr int8_t THROTTLE_LEVEL_MAX = 7;

// ============================================================================
// Component Configurations
// ============================================================================

static throttle_mux_config_t g_throttle_mux_cfg = {
    .a0 = MUX_A0_GPIO,
    .a1 = MUX_A1_GPIO,
    .a2 = MUX_A2_GPIO,
    .en = MUX_EN_GPIO,
    .relay = THROTTLE_RELAY_GPIO,
};

static enable_relay_config_t g_enable_relay_cfg = {
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
    bool auto_allowed;
    uint8_t auto_estop_reason;
    int8_t throttle_target;
    int16_t steering_cmd;
    int16_t braking_cmd;
    uint8_t motor_fault_code;  // set by CAN RX on stepper error
};

static portMUX_TYPE g_cmd_lock = portMUX_INITIALIZER_UNLOCKED;
static command_snapshot_t g_cmd = {};  // protected by g_cmd_lock

// ============================================================================
// Local State (Control task only - no lock needed)
// ============================================================================

// State machine
static uint8_t g_control_state = CONTROL_STATE_INIT;
static uint8_t g_fault_code = CONTROL_FAULT_NONE;
static uint8_t g_override_reason = OVERRIDE_REASON_NONE;

// Throttle
static int8_t g_throttle_current = 0;

// Timing state
static uint32_t g_enable_start_ms = 0;
static uint32_t g_last_throttle_change_ms = 0;

// Heartbeat
static uint8_t g_heartbeat_seq = 0;

// ============================================================================
// Recovery State
// ============================================================================

struct recovery_state_t {
    uint8_t can_tx_fail_count;        // consecutive CAN TX failures
    uint8_t recovery_attempts;        // fault recovery attempts
    uint32_t last_recovery_ms;        // cooldown tracking
    uint32_t fault_entered_ms;        // when fault state was entered
};

static recovery_state_t g_recovery = {};

// ============================================================================
// Stepper Motor Instances (UIM2852CA)
// ============================================================================

static stepper_motor_uim2852_t g_steering_stepper = {};
static stepper_motor_uim2852_t g_braking_stepper = {};
static constexpr TickType_t MOTOR_TIMEOUT_TICKS = pdMS_TO_TICKS(1000);

// ============================================================================
// Debug Helpers
// ============================================================================

// String helpers are now in can_protocol.hh (shared between control & safety)

static uint32_t get_time_ms() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// ============================================================================
// Init Retry Helper
// ============================================================================

// Retry an init function up to INIT_MAX_RETRIES times with delay between attempts.
// Returns ESP_OK on success, or last error on exhaustion.
typedef esp_err_t (*init_fn_t)(const void *config);

static esp_err_t retry_init(const char *name, init_fn_t fn, const void *config) {
    esp_err_t err = ESP_FAIL;
    for (int attempt = 1; attempt <= INIT_MAX_RETRIES; attempt++) {
        err = fn(config);
        if (err == ESP_OK) {
            if (attempt > 1)
                ESP_LOGI(TAG, "%s init succeeded on attempt %d", name, attempt);
            return ESP_OK;
        }
        ESP_LOGW(TAG, "%s init failed (attempt %d/%d): %s",
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
    ESP_LOGW(TAG, "Attempting CAN recovery (stop/start cycle)");
    twai_stop();
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_err_t err = twai_start();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "CAN recovery succeeded (stop/start)");
        g_recovery.can_tx_fail_count = 0;
        return;
    }

    // Escalate: full driver reinstall
    ESP_LOGW(TAG, "CAN stop/start failed, attempting full re-init");
    twai_driver_uninstall();
    vTaskDelay(pdMS_TO_TICKS(200));
    err = can_twai_init_default(TWAI_TX_GPIO, TWAI_RX_GPIO);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "CAN full re-init succeeded");
        g_recovery.can_tx_fail_count = 0;
        return;
    }

    ESP_LOGE(TAG, "CAN recovery FAILED, restarting ESP in %lu ms", (unsigned long)RESTART_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
    esp_restart();
}

// Track CAN TX result. On consecutive failures, trigger recovery.
static void track_can_tx(esp_err_t err) {
    if (err == ESP_OK) {
        g_recovery.can_tx_fail_count = 0;
        return;
    }
    g_recovery.can_tx_fail_count++;
    if (g_recovery.can_tx_fail_count >= CAN_TX_FAIL_THRESHOLD) {
        ESP_LOGE(TAG, "CAN TX failed %d consecutive times, initiating recovery",
                 g_recovery.can_tx_fail_count);
        attempt_can_recovery();
    }
}

// ============================================================================
// State Machine Action Helpers
// ============================================================================

// Immediately disable all autonomous actuators.
static void execute_trigger_override(uint8_t reason) {
    ESP_LOGW(TAG, "OVERRIDE TRIGGERED: %s", override_reason_to_string(reason));

    throttle_mux_disable();
    throttle_mux_emergency_stop();
    enable_relay_deenergize();
    stepper_motor_uim2852_emergency_stop(&g_steering_stepper);
    stepper_motor_uim2852_emergency_stop(&g_braking_stepper);
}

// Start the enable sequence (set mux to 0, energize relay)
static void execute_start_enable() {
    ESP_LOGI(TAG, "Starting autonomous enable sequence");
    throttle_mux_set_level(0);
    enable_relay_energize();
}

// Complete the enable sequence (enable steppers, switch mux to autonomous)
static void execute_complete_enable() {
    ESP_LOGI(TAG, "Completing autonomous enable sequence");
    stepper_motor_uim2852_enable(&g_steering_stepper);
    vTaskDelay(pdMS_TO_TICKS(5));
    stepper_motor_uim2852_enable(&g_braking_stepper);
    throttle_mux_enable_autonomous();
    ESP_LOGI(TAG, "AUTONOMOUS MODE ACTIVE");
}

// Abort the enable sequence (de-energize relay, disable mux)
static void execute_abort_enable() {
    ESP_LOGW(TAG, "Enable sequence aborted");
    enable_relay_deenergize();
    throttle_mux_disable();
}

// Map FR sensor state to control_logic FR constants
static uint8_t map_fr_state(fr_state_t fr) {
    switch (fr) {
        case FR_STATE_NEUTRAL: return CONTROL_FR_NEUTRAL;
        case FR_STATE_FORWARD: return CONTROL_FR_FORWARD;
        case FR_STATE_REVERSE: return CONTROL_FR_REVERSE;
        default:               return CONTROL_FR_INVALID;
    }
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

    ESP_LOGW(TAG, "Recovery attempt %d/%d for fault: %s",
             g_recovery.recovery_attempts, MAX_RECOVERY_ATTEMPTS,
             control_fault_to_string(g_fault_code));

    esp_err_t err = ESP_FAIL;

    switch (g_fault_code) {
        case CONTROL_FAULT_MOTOR_COMM:
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

        case CONTROL_FAULT_THROTTLE_INIT:
            err = throttle_mux_init(&g_throttle_mux_cfg);
            break;

        case CONTROL_FAULT_RELAY_INIT:
            err = enable_relay_init(&g_enable_relay_cfg);
            break;

        case CONTROL_FAULT_SENSOR_INVALID:
            // Sensor invalid may be transient (wiring glitch) - try re-init
            err = override_sensors_init(&g_override_sensors_cfg);
            break;

        case CONTROL_FAULT_CAN_TX:
            // CAN recovery is handled separately by attempt_can_recovery()
            attempt_can_recovery();
            err = ESP_OK;  // if we get here, recovery succeeded (otherwise esp_restart was called)
            break;

        default:
            ESP_LOGW(TAG, "No recovery handler for fault 0x%02X", g_fault_code);
            break;
    }

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Recovery SUCCEEDED for fault: %s", control_fault_to_string(g_fault_code));
        g_fault_code = CONTROL_FAULT_NONE;
        g_control_state = CONTROL_STATE_READY;
        g_recovery.recovery_attempts = 0;
        return true;
    }

    ESP_LOGW(TAG, "Recovery attempt %d FAILED: %s", g_recovery.recovery_attempts, esp_err_to_name(err));

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

        // Handle extended frames from UIM2852CA stepper motors
        if (msg.extd) {
            bool steering_match = stepper_motor_uim2852_process_frame(&g_steering_stepper, &msg);
            if (steering_match) {
                if (stepper_motor_uim2852_stall_detected(&g_steering_stepper) ||
                    stepper_motor_uim2852_has_error(&g_steering_stepper)) {
                    ESP_LOGW(TAG, "Steering stepper motor fault detected");
                    taskENTER_CRITICAL(&g_cmd_lock);
                    g_cmd.motor_fault_code = CONTROL_FAULT_MOTOR_COMM;
                    taskEXIT_CRITICAL(&g_cmd_lock);
                }
            } else {
                bool braking_match = stepper_motor_uim2852_process_frame(&g_braking_stepper, &msg);
                if (braking_match) {
                    if (stepper_motor_uim2852_stall_detected(&g_braking_stepper) ||
                        stepper_motor_uim2852_has_error(&g_braking_stepper)) {
                        ESP_LOGW(TAG, "Braking stepper motor fault detected");
                        taskENTER_CRITICAL(&g_cmd_lock);
                        g_cmd.motor_fault_code = CONTROL_FAULT_MOTOR_COMM;
                        taskEXIT_CRITICAL(&g_cmd_lock);
                    }
                }
            }
            continue;
        }

        // Process Orin command (0x111)
        if (msg.identifier == CAN_ID_ORIN_COMMAND && msg.data_length_code >= 8) {
            orin_command_t cmd;
            can_decode_orin_command(msg.data, &cmd);
            
            int8_t throttle_level = (int8_t)(cmd.throttle & 0x07);
            if (throttle_level > THROTTLE_LEVEL_MAX) throttle_level = THROTTLE_LEVEL_MAX;

            taskENTER_CRITICAL(&g_cmd_lock);
            g_cmd.throttle_target = throttle_level;
            g_cmd.steering_cmd = cmd.steering_position;
            g_cmd.braking_cmd = cmd.braking_position;
            taskEXIT_CRITICAL(&g_cmd_lock);
            
            heartbeat_mark_activity(now);

            ESP_LOGD(TAG, "[CAN RX] CMD: thr=%d steer=%d brake=%d seq=%u",
                throttle_level, cmd.steering_position, cmd.braking_position, cmd.sequence);
        }

        // Process Safety auto allowed (0x101)
        else if (msg.identifier == CAN_ID_SAFETY_AUTO_ALLOWED && msg.data_length_code >= 3) {
            safety_auto_allowed_t auto_msg;
            can_decode_safety_auto_allowed(msg.data, &auto_msg);

            bool new_allowed = (auto_msg.allowed != 0);
            bool was_allowed;

            taskENTER_CRITICAL(&g_cmd_lock);
            was_allowed = g_cmd.auto_allowed;
            g_cmd.auto_allowed = new_allowed;
            g_cmd.auto_estop_reason = auto_msg.estop_reason;
            taskEXIT_CRITICAL(&g_cmd_lock);
            
            heartbeat_mark_activity(now);
            
            if (new_allowed != was_allowed) {
                if (new_allowed) ESP_LOGW(TAG, "[CAN RX] Auto: ALLOWED");
                else ESP_LOGW(TAG, "[CAN RX] Auto: BLOCKED (estop:%s)", 
                    	estop_reason_to_string(auto_msg.estop_reason));
            }
        }
    }
}

// ============================================================================
// Control Task
// ============================================================================

void control_task(void *param) {
    (void)param;

    ESP_LOGI(TAG, "Control task started");

    // Initialize UIM2852CA stepper motors
    stepper_motor_uim2852_config_t steer_cfg = STEPPER_MOTOR_UIM2852_CONFIG_DEFAULT();
    steer_cfg.node_id = UIM2852_NODE_STEERING;
    stepper_motor_uim2852_init(&g_steering_stepper, &steer_cfg);

    stepper_motor_uim2852_config_t brake_cfg = STEPPER_MOTOR_UIM2852_CONFIG_DEFAULT();
    brake_cfg.node_id = UIM2852_NODE_BRAKING;
    stepper_motor_uim2852_init(&g_braking_stepper, &brake_cfg);

    stepper_motor_uim2852_configure(&g_steering_stepper);
    vTaskDelay(pdMS_TO_TICKS(10));
    stepper_motor_uim2852_configure(&g_braking_stepper);
    vTaskDelay(pdMS_TO_TICKS(10));

    stepper_motor_uim2852_disable(&g_steering_stepper);
    stepper_motor_uim2852_disable(&g_braking_stepper);

    g_throttle_current = 0;

    g_control_state = CONTROL_STATE_READY;
    ESP_LOGI(TAG, "Control ready, waiting for commands");

    uint8_t prev_state = 0xFF;
    fr_state_t prev_fr = FR_STATE_INVALID;
    int8_t prev_thr_target = -1;
    int8_t prev_thr_current = -1;
    bool prev_auto_allowed = false;

    // Stepper command change detection - avoid sending same position every loop
    int16_t last_steering_sent = INT16_MIN;
    int16_t last_braking_sent = INT16_MIN;

    vTaskDelay(pdMS_TO_TICKS(2000));

    while (true) {
        uint32_t now_ms = get_time_ms();

        // Take a thread-safe snapshot of CAN RX state
        command_snapshot_t cmd_local;
        taskENTER_CRITICAL(&g_cmd_lock);
        cmd_local = g_cmd;
        // Clear one-shot motor fault flag after reading
        g_cmd.motor_fault_code = CONTROL_FAULT_NONE;
        taskEXIT_CRITICAL(&g_cmd_lock);

        override_sensors_update(now_ms);
        fr_state_t fr_state = override_sensors_get_fr_state();

        // Build inputs for pure state machine step
        control_inputs_t step_in = {
            .auto_allowed = cmd_local.auto_allowed,
            .auto_estop_reason = cmd_local.auto_estop_reason,
            .throttle_target = cmd_local.throttle_target,
            .steering_cmd = cmd_local.steering_cmd,
            .braking_cmd = cmd_local.braking_cmd,
            .motor_fault_code = cmd_local.motor_fault_code,
            .fr_state = map_fr_state(fr_state),
            .pedal_pressed = override_sensors_pedal_pressed(),
            .pedal_rearmed = override_sensors_pedal_rearmed(),
            .fr_is_invalid = (fr_state == FR_STATE_INVALID),
            .now_ms = now_ms,
            .enable_start_ms = g_enable_start_ms,
            .enable_sequence_ms = ENABLE_SEQUENCE_MS,
            .throttle_current = g_throttle_current,
            .last_throttle_change_ms = g_last_throttle_change_ms,
            .throttle_slew_interval_ms = THROTTLE_SLEW_INTERVAL_MS,
            .last_steering_sent = last_steering_sent,
            .last_braking_sent = last_braking_sent,
        };

        // Compute next state (pure function — no side effects)
        control_step_result_t step = control_compute_step(g_control_state, g_fault_code, &step_in);

        // Execute hardware actions indicated by the step result
        if (step.actions & CONTROL_ACTION_TRIGGER_OVERRIDE) {
            execute_trigger_override(step.override_reason);
        }
        if (step.actions & CONTROL_ACTION_START_ENABLE) {
            execute_start_enable();
        }
        if (step.actions & CONTROL_ACTION_COMPLETE_ENABLE) {
            execute_complete_enable();
        }
        if (step.actions & CONTROL_ACTION_ABORT_ENABLE) {
            execute_abort_enable();
        }
        if (step.actions & CONTROL_ACTION_APPLY_THROTTLE) {
            throttle_mux_set_level(step.throttle_level);
        }
        if (step.actions & CONTROL_ACTION_ATTEMPT_RECOVERY) {
            attempt_fault_recovery();
        }

        // Send stepper commands when position changed
        if (step.send_steering) {
            esp_err_t err = stepper_motor_uim2852_go_absolute(&g_steering_stepper, step.steering_position);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "[CAN TX] Steering cmd failed: %s", esp_err_to_name(err));
                step.new_last_steering = last_steering_sent;  // keep old value on failure
            }
        }
        if (step.send_braking) {
            esp_err_t err = stepper_motor_uim2852_go_absolute(&g_braking_stepper, step.braking_position);
            if (err != ESP_OK) {
                ESP_LOGW(TAG, "[CAN TX] Braking cmd failed: %s", esp_err_to_name(err));
                step.new_last_braking = last_braking_sent;  // keep old value on failure
            }
        }

        // Detect fault entry (before overwriting g_fault_code)
        if (step.new_fault_code != CONTROL_FAULT_NONE && step.new_fault_code != g_fault_code) {
            g_recovery.fault_entered_ms = now_ms;
            g_recovery.recovery_attempts = 0;
            ESP_LOGE(TAG, "Entering FAULT state: %s", control_fault_to_string(step.new_fault_code));
        }

        // Apply state updates
        g_control_state = step.new_state;
        g_fault_code = step.new_fault_code;
        g_override_reason = step.override_reason;
        g_throttle_current = step.throttle_level;
        g_last_throttle_change_ms = step.throttle_change_ms;
        g_enable_start_ms = step.enable_start_ms;
        last_steering_sent = step.new_last_steering;
        last_braking_sent = step.new_last_braking;

        // Log on state change
        if (g_control_state != prev_state || fr_state != prev_fr ||
            cmd_local.throttle_target != prev_thr_target || g_throttle_current != prev_thr_current ||
            cmd_local.auto_allowed != prev_auto_allowed) {
            ESP_LOGI(TAG, "State: %s | FR=%d Pedal=%umV Thr=%d/%d Auto:%s (estop:%s)",
                control_state_to_string(g_control_state), 
                fr_state, override_sensors_get_pedal_mv(),
                g_throttle_current, cmd_local.throttle_target,
                cmd_local.auto_allowed ? "ALLOWED" : "BLOCKED",
                estop_reason_to_string(cmd_local.auto_estop_reason));
            prev_state = g_control_state;
            prev_fr = fr_state;
            prev_thr_target = cmd_local.throttle_target;
            prev_thr_current = g_throttle_current;
            prev_auto_allowed = cmd_local.auto_allowed;
        }

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

        if (g_control_state == CONTROL_STATE_FAULT || 
            g_control_state == CONTROL_STATE_OVERRIDE)
            heartbeat_set_error(true);
        else heartbeat_set_error(false);
        heartbeat_tick(cfg, now);

        // Read shared state for CAN broadcast
        command_snapshot_t cmd_local;
        taskENTER_CRITICAL(&g_cmd_lock);
        cmd_local = g_cmd;
        taskEXIT_CRITICAL(&g_cmd_lock);

        uint8_t hb_data[8] = {0};
        hb_data[0] = g_heartbeat_seq++;
        hb_data[1] = g_control_state;
        hb_data[2] = g_fault_code;

        esp_err_t err = can_twai_send(CAN_ID_CONTROL_HEARTBEAT, hb_data, pdMS_TO_TICKS(10));
        if (err != ESP_OK)
            ESP_LOGW(TAG, "[CAN TX] Heartbeat failed: %s", esp_err_to_name(err));
        track_can_tx(err);

        uint8_t status_data[8] = {0};
        
        status_data[0] = (g_control_state == CONTROL_STATE_ACTIVE) 
                         ? (uint8_t)g_throttle_current : 0xFF;
        
        status_data[1] = (uint8_t)override_sensors_get_fr_state();
        uint8_t sensor_flags = override_sensors_get_flags();

        if (enable_relay_is_energized()) sensor_flags |= SENSOR_FLAG_ENABLE_RELAY;
        if (throttle_mux_is_autonomous()) sensor_flags |= SENSOR_FLAG_THROTTLE_RELAY;
        status_data[2] = sensor_flags;
        
        status_data[3] = g_override_reason;
        can_pack_le16s(&status_data[4], cmd_local.steering_cmd);
        can_pack_le16s(&status_data[6], cmd_local.braking_cmd);

        err = can_twai_send(CAN_ID_CONTROL_STATUS, status_data, pdMS_TO_TICKS(10));
        if (err != ESP_OK)
            ESP_LOGW(TAG, "[CAN TX] Status failed: %s", esp_err_to_name(err));
        track_can_tx(err);

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
        ESP_LOGW(TAG, "TWAI init failed (attempt %d/%d): %s", i + 1, INIT_MAX_RETRIES, esp_err_to_name(err));
        if (i < INIT_MAX_RETRIES - 1) vTaskDelay(pdMS_TO_TICKS(INIT_RETRY_DELAY_MS));
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI init FAILED after %d retries, restarting ESP", INIT_MAX_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
        esp_restart();
    }

    // Initialize components with retry
    bool critical_failure = false;

    err = retry_init("Throttle mux", (init_fn_t)throttle_mux_init, &g_throttle_mux_cfg);
    if (err != ESP_OK) {
        g_fault_code = CONTROL_FAULT_THROTTLE_INIT;
        critical_failure = true;
    }

    err = retry_init("Enable relay", (init_fn_t)enable_relay_init, &g_enable_relay_cfg);
    if (err != ESP_OK) {
        g_fault_code = CONTROL_FAULT_RELAY_INIT;
        critical_failure = true;
    }

    err = retry_init("Override sensors", (init_fn_t)override_sensors_init, &g_override_sensors_cfg);
    if (err != ESP_OK) {
        g_fault_code = CONTROL_FAULT_SENSOR_INVALID;
        critical_failure = true;
    }

    // If any critical component failed after retries, restart
    if (critical_failure) {
        ESP_LOGE(TAG, "Critical component init failed (fault: %s), restarting ESP in %lu ms",
                 control_fault_to_string(g_fault_code), (unsigned long)RESTART_DELAY_MS);
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
        ESP_LOGW(TAG, "Heartbeat LED init failed: %s", esp_err_to_name(err));

    ESP_LOGI(TAG, "Initial F/R state: %d", override_sensors_get_fr_state());
    ESP_LOGI(TAG, "Initial pedal: %u mV (%s)", 
        override_sensors_get_pedal_mv(),
        override_sensors_pedal_pressed() ? "PRESSED" : "released");

    ESP_LOGI(TAG, "Starting control tasks");

    xTaskCreate(can_rx_task, "can_rx", CAN_RX_TASK_STACK, nullptr, CAN_RX_TASK_PRIO, nullptr);
    xTaskCreate(control_task, "control", CONTROL_TASK_STACK, nullptr, CONTROL_TASK_PRIO, nullptr);
    xTaskCreate(heartbeat_task, "heartbeat", HEARTBEAT_TASK_STACK, &heartbeat_cfg, HEARTBEAT_TASK_PRIO, nullptr);

    vTaskDelete(nullptr);
}
}

extern "C" void app_main(void) {
    xTaskCreate(main_task, "main_task", 4096, nullptr, 5, nullptr);
}
