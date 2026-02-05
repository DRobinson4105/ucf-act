#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "can_protocol.hh"
#include "can_twai.hh"
#include "heartbeat.hh"
#include "heartbeat_monitor.hh"
#include "push_button_hb2es544.hh"
#include "power_relay.hh"
#include "ultrasonic_a02yyuw.hh"
#include "rf_remote_ev1527.hh"

// Safety ESP32 - E-stop and autonomous permission control
//
// Monitors physical e-stop inputs (push button HB2-ES544, RF remote EV1527, ultrasonic A02YYUW)
// and node heartbeats (Orin, Control). When any e-stop condition is active,
// disables power relay and broadcasts CAN_ID_SAFETY_AUTO_ALLOWED = 0 to block
// autonomous mode. Control ESP32 must receive allowed=1 before enabling actuators.
//
// E-stop priority: push_button > rf_remote > ultrasonic > orin_error > orin_timeout
//                  > control_timeout > control_error

namespace {

static const char *TAG = "SAFETY";

// =============================================================================
// Task Configuration
// =============================================================================

constexpr int CAN_RX_TASK_STACK = 4096;
constexpr int SAFETY_TASK_STACK = 4096;
constexpr int HEARTBEAT_TASK_STACK = 2048;
constexpr UBaseType_t CAN_RX_TASK_PRIO = 7;
constexpr UBaseType_t SAFETY_TASK_PRIO = 6;
constexpr UBaseType_t HEARTBEAT_TASK_PRIO = 4;

// =============================================================================
// Timing Constants
// =============================================================================

constexpr TickType_t CAN_RX_TIMEOUT = pdMS_TO_TICKS(10);
constexpr TickType_t SAFETY_LOOP_INTERVAL = pdMS_TO_TICKS(50);
constexpr TickType_t HEARTBEAT_SEND_INTERVAL = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS);

// =============================================================================
// GPIO Pin Assignments
// =============================================================================

// E-stop inputs
constexpr gpio_num_t PUSH_BUTTON_HB2ES544_GPIO = GPIO_NUM_6;
constexpr int PUSH_BUTTON_HB2ES544_ACTIVE_LEVEL = 1;

constexpr gpio_num_t RF_REMOTE_EV1527_GPIO = GPIO_NUM_7;
constexpr int RF_REMOTE_EV1527_ACTIVE_LEVEL = 1;

// Power relay output (cuts power to actuators when e-stop active)
constexpr gpio_num_t POWER_RELAY_GPIO = GPIO_NUM_2;
constexpr bool POWER_RELAY_ACTIVE_HIGH = true;

// CAN bus (WAVESHARE SN65HVD230 transceiver)
constexpr gpio_num_t TWAI_TX_GPIO = GPIO_NUM_4;
constexpr gpio_num_t TWAI_RX_GPIO = GPIO_NUM_5;

// Ultrasonic sensor (A02YYUW)
constexpr uart_port_t ULTRASONIC_A02YYUW_UART = UART_NUM_1;
constexpr int ULTRASONIC_A02YYUW_TX_GPIO = GPIO_NUM_10;  // Sensor RX (mode select)
constexpr int ULTRASONIC_A02YYUW_RX_GPIO = GPIO_NUM_11;  // Sensor TX (data output)
constexpr int ULTRASONIC_A02YYUW_BAUD_RATE = 9600;
constexpr uint16_t ULTRASONIC_STOP_DISTANCE_MM = 1000;

// Status LED (WS2812)
constexpr gpio_num_t HEARTBEAT_LED_GPIO = GPIO_NUM_8;

// =============================================================================
// Global State
// =============================================================================

// E-stop input states
static bool g_push_button_active = false;
static bool g_rf_remote_active = false;
static bool g_ultrasonic_triggered = false;  // obstacle detected OR sensor fault
static bool g_orin_error = false;
static bool g_control_error = false;

// Aggregated e-stop state
static bool g_estop_active = false;
static uint8_t g_estop_reason = ESTOP_REASON_NONE;

// Heartbeat monitor (tracks Orin and Control timeouts)
static heartbeat_monitor_t g_hb_monitor;
static int g_node_orin = -1;
static int g_node_control = -1;

// =============================================================================
// Component Configurations
// =============================================================================

static push_button_hb2es544_config_t g_push_button_cfg;
static rf_remote_ev1527_config_t g_rf_remote_cfg;
static power_relay_config_t g_relay_cfg;
static ultrasonic_a02yyuw_config_t g_ultrasonic_cfg;
static heartbeat_config_t g_heartbeat_led_cfg;

// =============================================================================
// Debug Helpers
// =============================================================================

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

// =============================================================================
// CAN RX Task
// =============================================================================

// Receives CAN heartbeats from Orin and Control ESP32.
// Updates heartbeat monitor timestamps and detects fault states.
void can_rx_task(void *param) {
    (void)param;
    twai_message_t msg{};
    ESP_LOGI(TAG, "CAN RX task started");

    while (true) {
        if (can_twai_receive(&msg, CAN_RX_TIMEOUT) != ESP_OK) continue;
        if (msg.extd || msg.rtr) continue;
        heartbeat_mark_activity(xTaskGetTickCount());

        // Process Orin heartbeat - check state for fault condition
        if (msg.identifier == CAN_ID_ORIN_HEARTBEAT && msg.data_length_code >= 2) {
            uint8_t seq = msg.data[0];
            uint8_t state = msg.data[1];
            heartbeat_monitor_update(&g_hb_monitor, g_node_orin, seq, state);
            
            // Check if Orin is in fault state
            bool was_error = g_orin_error;
            g_orin_error = (state == ORIN_STATE_FAULT);
            if (g_orin_error && !was_error) {
                ESP_LOGW(TAG, "[CAN RX] Orin in FAULT state");
            } else if (!g_orin_error && was_error) {
                ESP_LOGI(TAG, "[CAN RX] Orin fault cleared");
            }
        }

        // Process Control ESP32 heartbeat - check state for fault condition
        else if (msg.identifier == CAN_ID_CONTROL_HEARTBEAT && msg.data_length_code >= 2) {
            uint8_t seq = msg.data[0];
            uint8_t state = msg.data[1];
            heartbeat_monitor_update(&g_hb_monitor, g_node_control, seq, state);
            
            // Check if Control is in fault state
            bool was_error = g_control_error;
            g_control_error = (state == CONTROL_STATE_FAULT);
            if (g_control_error && !was_error) {
                ESP_LOGW(TAG, "[CAN RX] Control in FAULT state");
            } else if (!g_control_error && was_error) {
                ESP_LOGI(TAG, "[CAN RX] Control fault cleared");
            }
        }
    }
}

// =============================================================================
// Safety Task
// =============================================================================

// Main safety monitoring loop. Runs at 20Hz.
// Reads e-stop inputs, checks heartbeat timeouts, controls power relay,
// and broadcasts auto-allowed state on CAN.
void safety_task(void *param) {
    (void)param;
    ESP_LOGI(TAG, "Safety task started");

    bool last_estop = false;
    uint8_t last_estop_reason = 0xFF;

    // Initial delay to let nodes report in
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (true) {
        // Read hardware inputs
        g_push_button_active = push_button_hb2es544_read_active(&g_push_button_cfg);
        g_rf_remote_active = rf_remote_ev1527_is_active(&g_rf_remote_cfg);

        // Check ultrasonic sensor - triggered if obstacle detected OR sensor not responding
        // (fail-safe: if we can't confirm path is clear, assume it's blocked)
        uint16_t distance_mm = 0;
        bool obstacle_detected = ultrasonic_a02yyuw_is_too_close(ULTRASONIC_STOP_DISTANCE_MM, &distance_mm);
        bool sensor_healthy = ultrasonic_a02yyuw_is_healthy();
        g_ultrasonic_triggered = obstacle_detected || !sensor_healthy;

        // Check heartbeat timeouts
        heartbeat_monitor_check_timeouts(&g_hb_monitor);
        bool orin_timeout = !heartbeat_monitor_is_alive(&g_hb_monitor, g_node_orin);
        bool control_timeout = !heartbeat_monitor_is_alive(&g_hb_monitor, g_node_control);

        // Determine E-stop state and reason (priority order)
        bool estop_active = false;
        uint8_t estop_reason = ESTOP_REASON_NONE;

        if (g_push_button_active) {
            estop_active = true;
            estop_reason = ESTOP_REASON_MUSHROOM;
        } else if (g_rf_remote_active) {
            estop_active = true;
            estop_reason = ESTOP_REASON_REMOTE;
        } else if (g_ultrasonic_triggered) {
            estop_active = true;
            estop_reason = ESTOP_REASON_ULTRASONIC;
        } else if (g_orin_error) {
            estop_active = true;
            estop_reason = ESTOP_REASON_ORIN_ERROR;
        } else if (orin_timeout) {
            estop_active = true;
            estop_reason = ESTOP_REASON_ORIN_TIMEOUT;
        } else if (control_timeout) {
            estop_active = true;
            estop_reason = ESTOP_REASON_CONTROL_TIMEOUT;
        } else if (g_control_error) {
            estop_active = true;
            estop_reason = ESTOP_REASON_CONTROL_ERROR;
        }

        g_estop_active = estop_active;
        g_estop_reason = estop_reason;

        // Control power relay based on safety state
        if (estop_active) {
            power_relay_disable(&g_relay_cfg);
            heartbeat_set_error(true);
        } else {
            power_relay_enable(&g_relay_cfg);
            heartbeat_set_error(false);
        }

        // Broadcast auto allowed state on change or periodically while blocked
        static TickType_t last_auto_broadcast = 0;
        TickType_t now = xTaskGetTickCount();
        bool should_broadcast = (estop_active != last_estop) || 
                                (estop_active && (now - last_auto_broadcast) >= pdMS_TO_TICKS(500));
        
        if (should_broadcast) {
            uint8_t data[8] = {0};
            safety_auto_allowed_t auto_msg = {
                .allowed = estop_active ? (uint8_t)0 : (uint8_t)1,
                .block_reason = (uint8_t)(estop_active ? AUTO_BLOCKED_REASON_ESTOP : AUTO_BLOCKED_REASON_NONE),
                .estop_reason = estop_reason
            };
            can_encode_safety_auto_allowed(data, &auto_msg);
            esp_err_t err = can_twai_send(CAN_ID_SAFETY_AUTO_ALLOWED, data, pdMS_TO_TICKS(10));
            
            if (err == ESP_OK) {
                if (estop_active != last_estop) {
                    ESP_LOGW(TAG, "[CAN TX] Auto: %s (estop:%s)", 
                             auto_msg.allowed ? "ALLOWED" : "BLOCKED", 
                             estop_reason_to_string(estop_reason));
                } else {
                    ESP_LOGW(TAG, "[CAN TX] Auto: BLOCKED periodic (estop:%s)", 
                             estop_reason_to_string(estop_reason));
                }
            } else {
                ESP_LOGE(TAG, "[CAN TX] Auto allowed failed: %s", esp_err_to_name(err));
            }
            
            last_estop = estop_active;
            last_auto_broadcast = now;
        }

        // Autonomous mode allowed when e-stop clear and all nodes alive
        bool auto_allowed = !estop_active && !orin_timeout && !control_timeout;

        // Log on state change
        if (estop_reason != last_estop_reason) {
            ESP_LOGI(TAG, "Auto: %s (estop:%s) | btn=%d remote=%d ultra=%d | orin=%d ctrl=%d",
                     auto_allowed ? "ALLOWED" : "BLOCKED",
                     estop_reason_to_string(estop_reason),
                     g_push_button_active, g_rf_remote_active, g_ultrasonic_triggered, 
                     (g_orin_error || orin_timeout) ? 1 : 0,
                     (g_control_error || control_timeout) ? 1 : 0);
            last_estop_reason = estop_reason;
        }

        vTaskDelay(SAFETY_LOOP_INTERVAL);
    }
}

// =============================================================================
// Heartbeat Task
// =============================================================================

// Updates WS2812 LED state indicator. Shows green when OK, red when e-stop active.
void heartbeat_task(void *param) {
    (void)param;

    ESP_LOGI(TAG, "Heartbeat task started");

    while (true) {
        TickType_t now = xTaskGetTickCount();

        // Update LED heartbeat
        heartbeat_tick(&g_heartbeat_led_cfg, now);

        vTaskDelay(HEARTBEAT_SEND_INTERVAL);
    }
}

// =============================================================================
// Main Task
// =============================================================================

// Initialization and task startup. Configures all peripherals and e-stop inputs,
// registers heartbeat monitor nodes, then spawns CAN RX, Safety, and Heartbeat tasks.
void main_task(void *param) {
    (void)param;

    ESP_LOGI(TAG, "Safety ESP32 startup");
#ifdef CONFIG_IDF_TARGET
    ESP_LOGI(TAG, "Target: %s", CONFIG_IDF_TARGET);
#endif

    // Initialize heartbeat monitor
    heartbeat_monitor_config_t hb_cfg = { .name = "SAFETY" };
    heartbeat_monitor_init(&g_hb_monitor, &hb_cfg);
    
    // Register nodes to monitor (Safety only cares about Orin and Control)
    g_node_orin = heartbeat_monitor_register(&g_hb_monitor, "Orin", HEARTBEAT_TIMEOUT_MS);
    g_node_control = heartbeat_monitor_register(&g_hb_monitor, "Control", HEARTBEAT_TIMEOUT_MS);

    // Configure push button e-stop (mxuteek HB2-ES544)
    g_push_button_cfg = {
        .gpio = PUSH_BUTTON_HB2ES544_GPIO,
        .active_level = PUSH_BUTTON_HB2ES544_ACTIVE_LEVEL,
        .enable_pullup = true,
        .enable_pulldown = false,
    };

    esp_err_t err = push_button_hb2es544_init(&g_push_button_cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Push button HB2-ES544 init failed: %s", esp_err_to_name(err));

    // Configure RF remote e-stop (DieseRC EV1527)
    g_rf_remote_cfg = {
        .gpio = RF_REMOTE_EV1527_GPIO,
        .active_level = RF_REMOTE_EV1527_ACTIVE_LEVEL,
        .enable_pullup = true,
        .enable_pulldown = false,
    };

    err = rf_remote_ev1527_init(&g_rf_remote_cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "RF remote EV1527 init failed: %s", esp_err_to_name(err));

    // Configure power relay (starts disabled = safe)
    g_relay_cfg = {
        .gpio = POWER_RELAY_GPIO,
        .active_high = POWER_RELAY_ACTIVE_HIGH,
        .enable_pullup = false,
        .enable_pulldown = true,  // Pull down to ensure OFF on reset
    };

    err = power_relay_init(&g_relay_cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Power relay init failed: %s", esp_err_to_name(err));

    // Configure ultrasonic sensor (A02YYUW)
    g_ultrasonic_cfg = {
        .uart_num = ULTRASONIC_A02YYUW_UART,
        .tx_gpio = ULTRASONIC_A02YYUW_TX_GPIO,
        .rx_gpio = ULTRASONIC_A02YYUW_RX_GPIO,
        .baud_rate = ULTRASONIC_A02YYUW_BAUD_RATE,
    };

    err = ultrasonic_a02yyuw_init(&g_ultrasonic_cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Ultrasonic A02YYUW init failed: %s", esp_err_to_name(err));

    // Initialize TWAI for CAN communication (via WAVESHARE SN65HVD230 transceiver)
    err = can_twai_init_default(TWAI_TX_GPIO, TWAI_RX_GPIO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI init failed: %s", esp_err_to_name(err));
        // This is fatal - we can't communicate safety state
        vTaskDelete(nullptr);
        return;
    }

    // Configure heartbeat LED (WS2812 on GPIO8)
    g_heartbeat_led_cfg = {
        .gpio = HEARTBEAT_LED_GPIO,
        .interval_ticks = pdMS_TO_TICKS(500),
        .activity_window_ticks = pdMS_TO_TICKS(250),
        .label = "safety",
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

    err = heartbeat_init(&g_heartbeat_led_cfg);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "Heartbeat LED init failed: %s", esp_err_to_name(err));

    ESP_LOGI(TAG, "Starting safety tasks");

    // Start tasks
    xTaskCreate(can_rx_task, "can_rx", CAN_RX_TASK_STACK, nullptr, CAN_RX_TASK_PRIO, nullptr);
    xTaskCreate(safety_task, "safety", SAFETY_TASK_STACK, nullptr, SAFETY_TASK_PRIO, nullptr);
    xTaskCreate(heartbeat_task, "heartbeat", HEARTBEAT_TASK_STACK, nullptr, HEARTBEAT_TASK_PRIO, nullptr);

    // Main task complete - delete self
    vTaskDelete(nullptr);
}
}

extern "C" void app_main(void) {
    xTaskCreate(main_task, "main_task", 4096, nullptr, 5, nullptr);
}
