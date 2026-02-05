#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"

#include "can_protocol.hh"
#include "can_twai.hh"
#include "heartbeat.hh"
#include "throttle_mux.hh"
#include "stepper_motor_uim2852.h"
#include "enable_relay.hh"
#include "override_sensors.hh"

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

namespace {

static const char *TAG = "CONTROL";

// =============================================================================
// Task Configuration
// =============================================================================

constexpr int CAN_RX_TASK_STACK = 4096;
constexpr int CONTROL_TASK_STACK = 4096;
constexpr int HEARTBEAT_TASK_STACK = 2048;
constexpr UBaseType_t CAN_RX_TASK_PRIO = 7;
constexpr UBaseType_t CONTROL_TASK_PRIO = 6;
constexpr UBaseType_t HEARTBEAT_TASK_PRIO = 4;

// =============================================================================
// Timing Constants
// =============================================================================

constexpr TickType_t CAN_RX_TIMEOUT = pdMS_TO_TICKS(10);
constexpr TickType_t CONTROL_LOOP_INTERVAL = pdMS_TO_TICKS(20);
constexpr TickType_t HEARTBEAT_SEND_INTERVAL = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS);
constexpr uint32_t ENABLE_SEQUENCE_MS = 200;
constexpr uint32_t THROTTLE_SLEW_INTERVAL_MS = 100;  // max 1 level change per 100ms

// =============================================================================
// GPIO Pin Assignments
// =============================================================================

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

// =============================================================================
// Component Configurations
// =============================================================================

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

// =============================================================================
// Global State
// =============================================================================

// State machine
static uint8_t g_control_state = CONTROL_STATE_INIT;
static uint8_t g_fault_code = CONTROL_FAULT_NONE;
static uint8_t g_override_reason = OVERRIDE_REASON_NONE;

// Safety permission (set by Safety via CAN_ID_SAFETY_AUTO_ALLOWED)
static bool g_auto_allowed = false;
static uint8_t g_auto_estop_reason = 0;

// Command state from Orin
static int8_t g_throttle_target = 0;
static int8_t g_throttle_current = 0;
static int16_t g_steering_cmd = 0;
static int16_t g_braking_cmd = 0;

// Timing state
static uint32_t g_enable_start_ms = 0;
static uint32_t g_last_throttle_change_ms = 0;

// Heartbeat
static uint8_t g_heartbeat_seq = 0;

// =============================================================================
// Stepper Motor Instances (UIM2852CA)
// =============================================================================

static stepper_motor_uim2852_t g_steering_stepper = {};
static stepper_motor_uim2852_t g_braking_stepper = {};
static constexpr TickType_t MOTOR_TIMEOUT_TICKS = pdMS_TO_TICKS(1000);

// =============================================================================
// Debug Helpers
// =============================================================================

static const char* control_state_to_string(uint8_t state) {
    switch (state) {
        case CONTROL_STATE_INIT:     return "INIT";
        case CONTROL_STATE_READY:    return "READY";
        case CONTROL_STATE_ENABLING: return "ENABLING";
        case CONTROL_STATE_ACTIVE:   return "ACTIVE";
        case CONTROL_STATE_OVERRIDE: return "OVERRIDE";
        case CONTROL_STATE_FAULT:    return "FAULT";
        default:                     return "UNKNOWN";
    }
}

static const char* estop_reason_to_string(uint8_t reason) {
    switch (reason) {
        case ESTOP_REASON_NONE:            return "none";
        case ESTOP_REASON_MUSHROOM:        return "push_button";
        case ESTOP_REASON_REMOTE:          return "rf_remote";
        case ESTOP_REASON_ULTRASONIC:      return "ultrasonic";
        case ESTOP_REASON_ORIN_ERROR:      return "orin_error";
        case ESTOP_REASON_ORIN_TIMEOUT:    return "orin_timeout";
        case ESTOP_REASON_CONTROL_TIMEOUT: return "control_timeout";
        case ESTOP_REASON_CONTROL_ERROR:   return "control_error";
        default:                           return "unknown";
    }
}

static const char* override_reason_to_string(uint8_t reason) {
    switch (reason) {
        case OVERRIDE_REASON_NONE:       return "none";
        case OVERRIDE_REASON_PEDAL:      return "pedal";
        case OVERRIDE_REASON_FR_CHANGED: return "fr_changed";
        default:                         return "unknown";
    }
}

static uint32_t get_time_ms() {
    return (uint32_t)(esp_timer_get_time() / 1000);
}

// =============================================================================
// State Machine Helpers
// =============================================================================

// Immediately disable all autonomous actuators and transition to OVERRIDE state.
static void trigger_override(uint8_t reason) {
    ESP_LOGW(TAG, "OVERRIDE TRIGGERED: %s", override_reason_to_string(reason));

    throttle_mux_disable();
    throttle_mux_emergency_stop();
    enable_relay_deenergize();
    stepper_motor_uim2852_emergency_stop(&g_steering_stepper);
    stepper_motor_uim2852_emergency_stop(&g_braking_stepper);

    g_override_reason = reason;
    g_throttle_current = 0;
    g_throttle_target = 0;
    g_control_state = CONTROL_STATE_OVERRIDE;
}

// Check preconditions for enabling autonomous mode
static bool check_enable_preconditions() {
    // Must be in Forward
    fr_state_t fr = override_sensors_get_fr_state();
    if (fr != FR_STATE_FORWARD) {
        ESP_LOGW(TAG, "Cannot enable: F/R not in Forward (state=%d)", fr);
        return false;
    }

    // Pedal must NOT be pressed
    if (override_sensors_pedal_pressed()) {
        ESP_LOGW(TAG, "Cannot enable: Pedal is pressed");
        return false;
    }

    // Pedal must be re-armed (below threshold for 500ms)
    if (!override_sensors_pedal_rearmed()) {
        ESP_LOGW(TAG, "Cannot enable: Pedal not re-armed (release for 500ms)");
        return false;
    }

    // No active faults
    if (g_fault_code != CONTROL_FAULT_NONE) {
        ESP_LOGW(TAG, "Cannot enable: Active fault code=%u", g_fault_code);
        return false;
    }

    return true;
}

// Start the enable sequence
static void start_enable_sequence() {
    ESP_LOGI(TAG, "Starting autonomous enable sequence");

    // Set DG408 to level 0
    throttle_mux_set_level(0);
    g_throttle_current = 0;
    g_last_throttle_change_ms = get_time_ms();

    // Energize MOSFET relay
    enable_relay_energize();

    g_enable_start_ms = get_time_ms();
    g_control_state = CONTROL_STATE_ENABLING;
}

// Complete the enable sequence
static void complete_enable_sequence() {
    ESP_LOGI(TAG, "Completing autonomous enable sequence");

    stepper_motor_uim2852_enable(&g_steering_stepper);
    vTaskDelay(pdMS_TO_TICKS(5));
    stepper_motor_uim2852_enable(&g_braking_stepper);

    throttle_mux_enable_autonomous();

    g_control_state = CONTROL_STATE_ACTIVE;
    g_override_reason = OVERRIDE_REASON_NONE;

    ESP_LOGI(TAG, "AUTONOMOUS MODE ACTIVE");
}

// =============================================================================
// CAN RX Task
// =============================================================================

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
            if (stepper_motor_uim2852_process_frame(&g_steering_stepper, &msg)) {
                if (stepper_motor_uim2852_stall_detected(&g_steering_stepper) ||
                    stepper_motor_uim2852_has_error(&g_steering_stepper)) {
                    ESP_LOGW(TAG, "Steering stepper motor fault detected");
                    g_fault_code = CONTROL_FAULT_MOTOR_COMM;
                    if (g_control_state == CONTROL_STATE_ACTIVE)
                        trigger_override(OVERRIDE_REASON_NONE);
                }
            } else if (stepper_motor_uim2852_process_frame(&g_braking_stepper, &msg)) {
                if (stepper_motor_uim2852_stall_detected(&g_braking_stepper) ||
                    stepper_motor_uim2852_has_error(&g_braking_stepper)) {
                    ESP_LOGW(TAG, "Braking stepper motor fault detected");
                    g_fault_code = CONTROL_FAULT_MOTOR_COMM;
                    if (g_control_state == CONTROL_STATE_ACTIVE)
                        trigger_override(OVERRIDE_REASON_NONE);
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
            g_throttle_target = throttle_level;
            g_steering_cmd = cmd.steering_position;
            g_braking_cmd = cmd.braking_position;
            
            heartbeat_mark_activity(now);

            ESP_LOGD(TAG, "[CAN RX] CMD: thr=%d steer=%d brake=%d seq=%u",
                throttle_level, cmd.steering_position, cmd.braking_position, cmd.sequence);
        }

        // Process Safety auto allowed (0x101)
        else if (msg.identifier == CAN_ID_SAFETY_AUTO_ALLOWED && msg.data_length_code >= 3) {
            safety_auto_allowed_t auto_msg;
            can_decode_safety_auto_allowed(msg.data, &auto_msg);
            
            bool was_allowed = g_auto_allowed;
            g_auto_allowed = (auto_msg.allowed != 0);
            g_auto_estop_reason = auto_msg.estop_reason;
            
            heartbeat_mark_activity(now);
            
            if (g_auto_allowed != was_allowed) {
                if (g_auto_allowed) ESP_LOGW(TAG, "[CAN RX] Auto: ALLOWED");
                else ESP_LOGW(TAG, "[CAN RX] Auto: BLOCKED (estop:%s)", 
                    	estop_reason_to_string(auto_msg.estop_reason));
            }
        }
    }
}

// =============================================================================
// Control Task
// =============================================================================

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
    g_throttle_target = 0;

    g_control_state = CONTROL_STATE_READY;
    ESP_LOGI(TAG, "Control ready, waiting for commands");

    uint8_t prev_state = 0xFF;
    fr_state_t prev_fr = FR_STATE_INVALID;
    int8_t prev_thr_target = -1;
    int8_t prev_thr_current = -1;
    bool prev_auto_allowed = false;

    vTaskDelay(pdMS_TO_TICKS(2000));

    while (true) {
        uint32_t now_ms = get_time_ms();

        override_sensors_update(now_ms);
        fr_state_t fr_state = override_sensors_get_fr_state();

        if (fr_state == FR_STATE_INVALID) {
            if (g_fault_code != CONTROL_FAULT_SENSOR_INVALID) {
                ESP_LOGE(TAG, "F/R sensor INVALID state detected!");
                g_fault_code = CONTROL_FAULT_SENSOR_INVALID;
                if (g_control_state == CONTROL_STATE_ACTIVE ||
                    g_control_state == CONTROL_STATE_ENABLING) {
                    trigger_override(OVERRIDE_REASON_FR_CHANGED);
                }
            }
        }

        switch (g_control_state) {
            case CONTROL_STATE_INIT:
                g_control_state = CONTROL_STATE_READY;
                break;

            case CONTROL_STATE_READY:
                if (g_auto_allowed && check_enable_preconditions())
                	start_enable_sequence();
                break;

            case CONTROL_STATE_ENABLING:
                if (!g_auto_allowed) {
                    ESP_LOGW(TAG, "Enable sequence aborted (auto blocked)");
                    enable_relay_deenergize();
                    throttle_mux_disable();
                    g_control_state = CONTROL_STATE_READY;
                    break;
                }

                if (override_sensors_pedal_pressed()) {
                    ESP_LOGW(TAG, "Enable aborted: pedal pressed");
                    enable_relay_deenergize();
                    throttle_mux_disable();
                    g_control_state = CONTROL_STATE_READY;
                    break;
                }

                if (fr_state != FR_STATE_FORWARD) {
                    ESP_LOGW(TAG, "Enable aborted: F/R not forward");
                    enable_relay_deenergize();
                    throttle_mux_disable();
                    g_control_state = CONTROL_STATE_READY;
                    break;
                }

                if ((now_ms - g_enable_start_ms) >= ENABLE_SEQUENCE_MS)
                    complete_enable_sequence();
                break;

            case CONTROL_STATE_ACTIVE:
                if (override_sensors_pedal_pressed()) {
                    trigger_override(OVERRIDE_REASON_PEDAL);
                    break;
                }

                if (fr_state != FR_STATE_FORWARD) {
                    trigger_override(OVERRIDE_REASON_FR_CHANGED);
                    break;
                }

                if (!g_auto_allowed) {
                    ESP_LOGW(TAG, "Safety blocked auto (estop:%s)", 
                        estop_reason_to_string(g_auto_estop_reason));
                    trigger_override(OVERRIDE_REASON_NONE);
                    break;
                }

                // Execute throttle commands with slew rate limiting
                if (g_throttle_current != g_throttle_target) {
                    if ((now_ms - g_last_throttle_change_ms) >= THROTTLE_SLEW_INTERVAL_MS) {
                        if (g_throttle_current < g_throttle_target) g_throttle_current++;
                    	else g_throttle_current--;
            
                        throttle_mux_set_level(g_throttle_current);
                        g_last_throttle_change_ms = now_ms;
                    }
                }

                {
                    esp_err_t err = stepper_motor_uim2852_go_absolute(&g_steering_stepper, g_steering_cmd);
                    if (err != ESP_OK)
                        ESP_LOGW(TAG, "[CAN TX] Steering cmd failed: %s", esp_err_to_name(err));

                    err = stepper_motor_uim2852_go_absolute(&g_braking_stepper, g_braking_cmd);
                    if (err != ESP_OK)
                        ESP_LOGW(TAG, "[CAN TX] Braking cmd failed: %s", esp_err_to_name(err));
                }
                break;

            case CONTROL_STATE_OVERRIDE:
                g_throttle_current = 0;
                
                if (g_auto_allowed && 
                    fr_state == FR_STATE_FORWARD &&
                    override_sensors_pedal_rearmed()) {
                    ESP_LOGI(TAG, "Override cleared - returning to READY");
                    g_override_reason = OVERRIDE_REASON_NONE;
                    g_control_state = CONTROL_STATE_READY;
                }
                break;

            case CONTROL_STATE_FAULT:
                g_throttle_current = 0;
                break;
        }

        // Log on state change
        if (g_control_state != prev_state || fr_state != prev_fr ||
            g_throttle_target != prev_thr_target || g_throttle_current != prev_thr_current ||
            g_auto_allowed != prev_auto_allowed) {
            ESP_LOGI(TAG, "State: %s | FR=%d Pedal=%umV Thr=%d/%d Auto:%s (estop:%s)",
                control_state_to_string(g_control_state), 
                fr_state, override_sensors_get_pedal_mv(),
                g_throttle_current, g_throttle_target,
                g_auto_allowed ? "ALLOWED" : "BLOCKED",
                estop_reason_to_string(g_auto_estop_reason));
            prev_state = g_control_state;
            prev_fr = fr_state;
            prev_thr_target = g_throttle_target;
            prev_thr_current = g_throttle_current;
            prev_auto_allowed = g_auto_allowed;
        }

        vTaskDelay(CONTROL_LOOP_INTERVAL);
    }
}

// =============================================================================
// Heartbeat Task
// =============================================================================

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

        uint8_t hb_data[8] = {0};
        hb_data[0] = g_heartbeat_seq++;
        hb_data[1] = g_control_state;
        hb_data[2] = g_fault_code;

        esp_err_t err = can_twai_send(CAN_ID_CONTROL_HEARTBEAT, hb_data, pdMS_TO_TICKS(10));
        if (err != ESP_OK)
            ESP_LOGW(TAG, "[CAN TX] Heartbeat failed: %s", esp_err_to_name(err));

        uint8_t status_data[8] = {0};
        
        status_data[0] = (g_control_state == CONTROL_STATE_ACTIVE) 
                         ? (uint8_t)g_throttle_current : 0xFF;
        
        status_data[1] = (uint8_t)override_sensors_get_fr_state();
        uint8_t sensor_flags = override_sensors_get_flags();

        if (enable_relay_is_energized()) sensor_flags |= SENSOR_FLAG_ENABLE_RELAY;
        if (throttle_mux_is_autonomous()) sensor_flags |= SENSOR_FLAG_THROTTLE_RELAY;
        status_data[2] = sensor_flags;
        
        status_data[3] = g_override_reason;
        can_pack_le16s(&status_data[4], g_steering_cmd);
        can_pack_le16s(&status_data[6], g_braking_cmd);

        err = can_twai_send(CAN_ID_CONTROL_STATUS, status_data, pdMS_TO_TICKS(10));
        if (err != ESP_OK)
            ESP_LOGW(TAG, "[CAN TX] Status failed: %s", esp_err_to_name(err));

        vTaskDelay(HEARTBEAT_SEND_INTERVAL);
    }
}

// =============================================================================
// Main Task
// =============================================================================

void main_task(void *param) {
    (void)param;

    ESP_LOGI(TAG, "Control ESP32 startup");
#ifdef CONFIG_IDF_TARGET
    ESP_LOGI(TAG, "Target: %s", CONFIG_IDF_TARGET);
#endif

    // Initialize TWAI (via WAVESHARE SN65HVD230 transceiver)
    esp_err_t err = can_twai_init_default(TWAI_TX_GPIO, TWAI_RX_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI init failed: %s", esp_err_to_name(err));
        g_fault_code = CONTROL_FAULT_CAN_TX;
        g_control_state = CONTROL_STATE_FAULT;
        vTaskDelete(nullptr);
        return;
    }

    err = throttle_mux_init(&g_throttle_mux_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Throttle mux init failed: %s", esp_err_to_name(err));
        g_fault_code = CONTROL_FAULT_THROTTLE_INIT;
        g_control_state = CONTROL_STATE_FAULT;
    }

    err = enable_relay_init(&g_enable_relay_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Enable relay init failed: %s", esp_err_to_name(err));
        g_fault_code = CONTROL_FAULT_THROTTLE_INIT;
        g_control_state = CONTROL_STATE_FAULT;
    }

    err = override_sensors_init(&g_override_sensors_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Override sensors init failed: %s", esp_err_to_name(err));
        g_fault_code = CONTROL_FAULT_SENSOR_INVALID;
        g_control_state = CONTROL_STATE_FAULT;
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

    if (g_fault_code != CONTROL_FAULT_NONE) heartbeat_set_error(true);

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
