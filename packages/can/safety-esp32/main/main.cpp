/**
 * @file main.cpp
 * @brief Safety ESP32 main application — e-stop monitoring and autonomous permission control.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/twai.h"

#include "can_protocol.hh"
#include "can_twai.hh"
#include "heartbeat.hh"
#include "heartbeat_monitor.hh"
#include "push_button_hb2es544.hh"
#include "power_relay.hh"
#include "ultrasonic_a02yyuw.hh"
#include "rf_remote_ev1527.hh"
#include "safety_logic.h"

// Safety ESP32 - E-stop and autonomous permission control
//
// Monitors physical e-stop inputs (push button HB2-ES544, RF remote EV1527, ultrasonic A02YYUW)
// and node heartbeats (Orin, Control). When any e-stop condition is active,
// disables power relay and broadcasts CAN_ID_SAFETY_AUTO_ALLOWED = 0 to block
// autonomous mode. Control ESP32 must receive allowed=1 before enabling actuators.
//
// E-stop priority: push_button > rf_remote > ultrasonic > orin_error > orin_timeout
//                  > control_timeout > control_error
//
// Recovery: Non-TWAI component failures are logged but do not prevent operation
// (safety defaults to blocking everything). TWAI failures trigger retry with
// escalation to esp_restart().

namespace {

static const char *TAG = "SAFETY";

// ============================================================================
// Task Configuration
// ============================================================================

constexpr int CAN_RX_TASK_STACK = 4096;
constexpr int SAFETY_TASK_STACK = 4096;
constexpr int HEARTBEAT_TASK_STACK = 2048;
constexpr UBaseType_t CAN_RX_TASK_PRIO = 7;
constexpr UBaseType_t SAFETY_TASK_PRIO = 6;
constexpr UBaseType_t HEARTBEAT_TASK_PRIO = 4;

// ============================================================================
// Timing Constants
// ============================================================================

constexpr TickType_t CAN_RX_TIMEOUT = pdMS_TO_TICKS(10);
constexpr TickType_t SAFETY_LOOP_INTERVAL = pdMS_TO_TICKS(50);
constexpr TickType_t HEARTBEAT_SEND_INTERVAL = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS);

// ============================================================================
// Init Retry & Recovery Constants
// ============================================================================

constexpr uint8_t INIT_MAX_RETRIES = 3;
constexpr uint32_t INIT_RETRY_DELAY_MS = 1000;
constexpr uint8_t CAN_TX_FAIL_THRESHOLD = 5;
constexpr uint32_t RESTART_DELAY_MS = 3000;  // delay before esp_restart to flush logs

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

// ============================================================================
// Global State
// ============================================================================

// Fault states set by CAN RX task (read by safety_task)
static bool g_orin_error = false;
static bool g_control_error = false;

// Heartbeat monitor (tracks Orin and Control timeouts)
static heartbeat_monitor_t g_hb_monitor;
static int g_node_orin = -1;
static int g_node_control = -1;

// ============================================================================
// Component Configurations
// ============================================================================

static push_button_hb2es544_config_t g_push_button_cfg;
static rf_remote_ev1527_config_t g_rf_remote_cfg;
static power_relay_config_t g_relay_cfg;
static ultrasonic_a02yyuw_config_t g_ultrasonic_cfg;
static heartbeat_config_t g_heartbeat_led_cfg;

// ============================================================================
// Recovery State
// ============================================================================

static uint8_t g_can_tx_fail_count = 0;

// ============================================================================
// Debug Helpers (string helpers are now in can_protocol.hh)
// ============================================================================

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
        g_can_tx_fail_count = 0;
        return;
    }

    // Escalate: full driver reinstall
    ESP_LOGW(TAG, "CAN stop/start failed, attempting full re-init");
    twai_driver_uninstall();
    vTaskDelay(pdMS_TO_TICKS(200));
    err = can_twai_init_default(TWAI_TX_GPIO, TWAI_RX_GPIO);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "CAN full re-init succeeded");
        g_can_tx_fail_count = 0;
        return;
    }

    ESP_LOGE(TAG, "CAN recovery FAILED, restarting ESP in %lu ms", (unsigned long)RESTART_DELAY_MS);
    vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
    esp_restart();
}

// Track CAN TX result. On consecutive failures, trigger recovery.
static void track_can_tx(esp_err_t err) {
    if (err == ESP_OK) {
        g_can_tx_fail_count = 0;
        return;
    }
    g_can_tx_fail_count++;
    if (g_can_tx_fail_count >= CAN_TX_FAIL_THRESHOLD) {
        ESP_LOGE(TAG, "CAN TX failed %d consecutive times, initiating recovery",
                 g_can_tx_fail_count);
        attempt_can_recovery();
    }
}

// ============================================================================
// CAN RX Task
// ============================================================================

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

// ============================================================================
// Safety Task
// ============================================================================

// Main safety monitoring loop. Runs at 20Hz.
// Reads e-stop inputs, checks heartbeat timeouts, controls power relay,
// and broadcasts auto-allowed state on CAN.
//
// Decision logic (e-stop priority chain, relay, auto-allowed) is delegated to
// the pure safety_logic module, making the core algorithm unit-testable on host.
void safety_task(void *param) {
    (void)param;
    ESP_LOGI(TAG, "Safety task started");

    bool last_estop = false;
    uint8_t last_estop_reason = 0xFF;
    TickType_t last_auto_broadcast = 0;

    // Initial delay to let nodes report in
    vTaskDelay(pdMS_TO_TICKS(2000));

    while (true) {
        // Read hardware inputs into the pure-function input struct
        uint16_t distance_mm = 0;

        safety_inputs_t inputs = {
            .push_button_active = push_button_hb2es544_read_active(&g_push_button_cfg),
            .rf_remote_active   = rf_remote_ev1527_is_active(&g_rf_remote_cfg),
            .ultrasonic_too_close = ultrasonic_a02yyuw_is_too_close(ULTRASONIC_STOP_DISTANCE_MM, &distance_mm),
            .ultrasonic_healthy = ultrasonic_a02yyuw_is_healthy(),
            .orin_alive   = true,   // updated below from heartbeat monitor
            .control_alive = true,  // updated below from heartbeat monitor
            .orin_error   = g_orin_error,
            .control_error = g_control_error,
        };

        // Check heartbeat timeouts
        heartbeat_monitor_check_timeouts(&g_hb_monitor);
        inputs.orin_alive = heartbeat_monitor_is_alive(&g_hb_monitor, g_node_orin);
        inputs.control_alive = heartbeat_monitor_is_alive(&g_hb_monitor, g_node_control);

        // Evaluate all safety conditions (pure function — no side effects)
        safety_decision_t decision = safety_evaluate(&inputs);

        // Control power relay based on safety decision
        if (decision.relay_enable) {
            power_relay_enable(&g_relay_cfg);
            heartbeat_set_error(false);
        } else {
            power_relay_disable(&g_relay_cfg);
            heartbeat_set_error(true);
        }

        // Determine whether to broadcast (pure function)
        TickType_t now = xTaskGetTickCount();
        safety_broadcast_t bcast = safety_should_broadcast(
            decision.estop_active, last_estop, now, last_auto_broadcast, pdMS_TO_TICKS(500));

        if (bcast.should_broadcast) {
            uint8_t data[8] = {0};
            safety_auto_allowed_t auto_msg = {
                .allowed = decision.estop_active ? (uint8_t)0 : (uint8_t)1,
                .block_reason = (uint8_t)(decision.estop_active ? AUTO_BLOCKED_REASON_ESTOP : AUTO_BLOCKED_REASON_NONE),
                .estop_reason = decision.estop_reason
            };
            can_encode_safety_auto_allowed(data, &auto_msg);
            esp_err_t err = can_twai_send(CAN_ID_SAFETY_AUTO_ALLOWED, data, pdMS_TO_TICKS(10));
            track_can_tx(err);
            
            if (err == ESP_OK) {
                if (bcast.state_changed) {
                    ESP_LOGW(TAG, "[CAN TX] Auto: %s (estop:%s)", 
                             auto_msg.allowed ? "ALLOWED" : "BLOCKED", 
                             estop_reason_to_string(decision.estop_reason));
                } else {
                    ESP_LOGW(TAG, "[CAN TX] Auto: BLOCKED periodic (estop:%s)", 
                             estop_reason_to_string(decision.estop_reason));
                }
            } else {
                ESP_LOGE(TAG, "[CAN TX] Auto allowed failed: %s", esp_err_to_name(err));
            }
            
            last_estop = decision.estop_active;
            last_auto_broadcast = now;
        }

        // Log on state change
        if (decision.estop_reason != last_estop_reason) {
            ESP_LOGI(TAG, "Auto: %s (estop:%s) | btn=%d remote=%d ultra=%d | orin=%d ctrl=%d",
                     decision.auto_allowed ? "ALLOWED" : "BLOCKED",
                     estop_reason_to_string(decision.estop_reason),
                     inputs.push_button_active, inputs.rf_remote_active,
                     safety_compute_ultrasonic_trigger(inputs.ultrasonic_too_close, inputs.ultrasonic_healthy),
                     (inputs.orin_error || !inputs.orin_alive) ? 1 : 0,
                     (inputs.control_error || !inputs.control_alive) ? 1 : 0);
            last_estop_reason = decision.estop_reason;
        }

        vTaskDelay(SAFETY_LOOP_INTERVAL);
    }
}

// ============================================================================
// Heartbeat Task
// ============================================================================

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

// ============================================================================
// Main Task
// ============================================================================

// Initialization and task startup. Configures all peripherals and e-stop inputs,
// registers heartbeat monitor nodes, then spawns CAN RX, Safety, and Heartbeat tasks.
// Non-TWAI component failures are logged but don't prevent startup (safety defaults
// to blocking everything, which is the correct fail-safe behavior).
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

    esp_err_t err = retry_init("Push button HB2-ES544",
        (init_fn_t)push_button_hb2es544_init, &g_push_button_cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Push button init failed after retries - safety will default to blocking");

    // Configure RF remote e-stop (DieseRC EV1527)
    g_rf_remote_cfg = {
        .gpio = RF_REMOTE_EV1527_GPIO,
        .active_level = RF_REMOTE_EV1527_ACTIVE_LEVEL,
        .enable_pullup = true,
        .enable_pulldown = false,
    };

    err = retry_init("RF remote EV1527",
        (init_fn_t)rf_remote_ev1527_init, &g_rf_remote_cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "RF remote init failed after retries - safety will default to blocking");

    // Configure power relay (starts disabled = safe)
    g_relay_cfg = {
        .gpio = POWER_RELAY_GPIO,
        .active_high = POWER_RELAY_ACTIVE_HIGH,
        .enable_pullup = false,
        .enable_pulldown = true,  // Pull down to ensure OFF on reset
    };

    err = retry_init("Power relay", (init_fn_t)power_relay_init, &g_relay_cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Power relay init failed after retries - relay stays OFF (safe)");

    // Configure ultrasonic sensor (A02YYUW)
    g_ultrasonic_cfg = {
        .uart_num = ULTRASONIC_A02YYUW_UART,
        .tx_gpio = ULTRASONIC_A02YYUW_TX_GPIO,
        .rx_gpio = ULTRASONIC_A02YYUW_RX_GPIO,
        .baud_rate = ULTRASONIC_A02YYUW_BAUD_RATE,
    };

    err = retry_init("Ultrasonic A02YYUW",
        (init_fn_t)ultrasonic_a02yyuw_init, &g_ultrasonic_cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "Ultrasonic init failed after retries - health check will block auto");

    // Initialize TWAI for CAN communication (via WAVESHARE SN65HVD230 transceiver)
    // TWAI is critical - retry, then restart on failure.
    // Can't use retry_init helper because can_twai_init_default takes 2 params.
    err = ESP_FAIL;
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
        // TWAI is fatal for safety node - we can't communicate safety state
        ESP_LOGE(TAG, "TWAI init FAILED after %d retries, restarting ESP", INIT_MAX_RETRIES);
        vTaskDelay(pdMS_TO_TICKS(RESTART_DELAY_MS));
        esp_restart();
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
