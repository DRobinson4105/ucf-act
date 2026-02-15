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

#include "can_protocol.hh"
#include "can_twai.hh"
#include "led_ws2812.hh"
#include "heartbeat_monitor.hh"
#include "push_button_hb2es544.hh"
#include "relay_srd05vdc.hh"
#include "ultrasonic_a02yyuw.hh"
#include "rf_remote_ev1527.hh"
#include "safety_logic.h"
#include "system_state.h"



// Safety ESP32 - System state authority and e-stop monitoring
//
// Monitors physical e-stop inputs (push button HB2-ES544, RF remote EV1527, ultrasonic A02YYUW)
// and node heartbeats (Planner, Control). When any e-stop condition is active, disables power
// relay and retreats the system target state to READY.
//
// Safety is the ONLY node that can advance state forward (READY -> ENABLING -> ACTIVE).
// It broadcasts a heartbeat (0x100) with target_state in the state field and the estop
// fault_code. All three nodes use the same heartbeat format (node_heartbeat_t).
//
// State advancement:
//   READY -> ENABLING:  both Planner and Control report READY, no e-stop,
//                       Planner/Orin autonomy request edge latched
//   ENABLING -> ACTIVE: both report ENABLING + enable_complete flag set
//   ENABLING/ACTIVE -> READY: Planner/Orin autonomy request dropped (halt)
//   ANY -> READY:       e-stop, fault, override, timeout
//
// E-stop faults are OR'd into a bitmask — all active faults are reported simultaneously.
// Fault bits: button | remote | ultrasonic | planner | planner_timeout | control | control_timeout

namespace {

static const char *TAG = "SAFETY";
static const char *TAG_INIT = "SAFETY_INIT";
static const char *TAG_TX __attribute__((unused)) = "SAFETY_TX";
static const char *TAG_RX __attribute__((unused)) = "SAFETY_RX";

// ============================================================================
// Task Configuration
// ============================================================================

constexpr int CAN_RX_TASK_STACK = 4096;
constexpr int SAFETY_TASK_STACK = 4096;
constexpr int HEARTBEAT_TASK_STACK = 4096;
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
// Recovery Constants
// ============================================================================

constexpr uint8_t CAN_TX_FAIL_THRESHOLD = 5;
constexpr uint32_t RETRY_INTERVAL_MS = 500;  // component retry cadence (infinite, no cap)

// Debounce: require N consecutive "clear" reads before declaring push-button
// or RF remote disengaged. Engage is always immediate (safety-critical).
// At 50ms loop cadence, 3 samples = 150ms disengage hold.
constexpr uint8_t ESTOP_DISENGAGE_COUNT = 3;

// Ultrasonic health hysteresis: require N consecutive agreeing health samples
// before changing health state. Prevents flap around timeout boundary.
// At 50ms loop cadence, 3 samples = 150ms hysteresis window.
constexpr uint8_t ULTRASONIC_HEALTH_HYSTERESIS = 3;

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
constexpr uint8_t LED_LEVEL = 16;
constexpr uint8_t LED_ORANGE_GREEN = 8;

// ============================================================================
// Global State
// ============================================================================

// Heartbeat monitor (tracks Planner and Control timeouts)
static heartbeat_monitor_t g_hb_monitor;
static int g_node_planner = -1;
static int g_node_control = -1;

// Node state tracking (set by CAN RX task, read by safety_task)
// These are volatile single-byte reads — atomically safe on ESP32.
static volatile uint8_t g_planner_state = NODE_STATE_INIT;
static volatile uint8_t g_planner_flags = 0;

static volatile uint8_t g_control_node_state = NODE_STATE_INIT;
static volatile uint8_t g_control_flags = 0;

// System target state (Safety is the authority)
// Starts as READY (not INIT) so early heartbeats never advertise INIT.
static volatile uint8_t g_target_state = NODE_STATE_READY;

// Safety heartbeat sequence counter
static volatile uint8_t g_safety_hb_seq = 0;
[[maybe_unused]] static bool g_hb_tx_failing = false;  // edge-trigger for heartbeat TX failure logging

// Spinlock for heartbeat sequence (shared by safety_task immediate send + heartbeat_task)
static portMUX_TYPE g_safety_hb_seq_lock = portMUX_INITIALIZER_UNLOCKED;

// Fault code for heartbeat (NODE_FAULT_ESTOP_* or NODE_FAULT_NONE)
static volatile uint8_t g_estop_fault_code = NODE_FAULT_NONE;

// Component health tracking.
static bool g_push_button_init_ok = false;
static bool g_rf_remote_init_ok = false;
static bool g_ultrasonic_init_ok = false;
static bool g_ultrasonic_health_prev = false;
static bool g_ultrasonic_health_seen = false;
static uint8_t g_ultrasonic_health_counter = 0;  // hysteresis counter
static bool g_relay_init_ok = false;
static bool g_twai_ready = false;

// Debounce counters for push-button and RF disengage (engage is immediate).
static uint8_t g_push_button_clear_count = 0;
static bool g_push_button_debounced = false;  // debounced "active" state
static uint8_t g_rf_remote_clear_count = 0;
static bool g_rf_remote_debounced = false;    // debounced "active" state

// ============================================================================
// Component Configurations
// ============================================================================

static push_button_hb2es544_config_t g_push_button_cfg;
static rf_remote_ev1527_config_t g_rf_remote_cfg;
static relay_srd05vdc_config_t g_relay_cfg;
static ultrasonic_a02yyuw_config_t g_ultrasonic_cfg;
static led_ws2812_config_t g_heartbeat_led_cfg;

// Retry pacing: last timestamp (ms) when retry_failed_components ran.
// Retries are infinite (no cap) but paced at RETRY_INTERVAL_MS to avoid
// hammering init calls on every safety loop tick.
static uint32_t g_last_retry_ms = 0;

static uint32_t get_time_ms() {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

static void log_startup_device_status(bool twai_ready, bool relay_init_ok, bool heartbeat_ready) {
    if (twai_ready) {
        ESP_LOGI(TAG_INIT, "TWAI: OK: tx_gpio=%d rx_gpio=%d", TWAI_TX_GPIO, TWAI_RX_GPIO);
    } else {
        ESP_LOGE(TAG_INIT, "TWAI: FAILED: tx_gpio=%d rx_gpio=%d", TWAI_TX_GPIO, TWAI_RX_GPIO);
    }

#ifdef CONFIG_BYPASS_INPUT_PUSH_BUTTON
    ESP_LOGW(TAG_INIT,
             "PUSH_BUTTON: BYPASSED: disabled (input_gpio=%d active_level=%s)",
             g_push_button_cfg.gpio,
             g_push_button_cfg.active_level ? "HIGH" : "LOW");
#else
    if (g_push_button_init_ok)
        ESP_LOGI(TAG_INIT,
                 "PUSH_BUTTON: OK: input_gpio=%d active_level=%s",
                 g_push_button_cfg.gpio,
                 g_push_button_cfg.active_level ? "HIGH" : "LOW");
    else
        ESP_LOGE(TAG_INIT, "PUSH_BUTTON: FAILED: input_gpio=%d", g_push_button_cfg.gpio);
#endif

#ifdef CONFIG_BYPASS_INPUT_RF_REMOTE
    ESP_LOGW(TAG_INIT,
             "RF_REMOTE: BYPASSED: disabled (input_gpio=%d active_level=%s)",
             g_rf_remote_cfg.gpio,
             g_rf_remote_cfg.active_level ? "HIGH" : "LOW");
#else
    if (g_rf_remote_init_ok)
        ESP_LOGI(TAG_INIT,
                 "RF_REMOTE: OK: input_gpio=%d active_level=%s",
                 g_rf_remote_cfg.gpio,
                 g_rf_remote_cfg.active_level ? "HIGH" : "LOW");
    else
        ESP_LOGE(TAG_INIT, "RF_REMOTE: FAILED: input_gpio=%d", g_rf_remote_cfg.gpio);
#endif

#ifdef CONFIG_BYPASS_INPUT_ULTRASONIC
    ESP_LOGW(TAG_INIT,
             "ULTRASONIC: BYPASSED: disabled (uart=%d tx_gpio=%d rx_gpio=%d)",
             g_ultrasonic_cfg.uart_num,
             g_ultrasonic_cfg.tx_gpio,
             g_ultrasonic_cfg.rx_gpio);
#else
    if (g_ultrasonic_init_ok)
        ESP_LOGI(TAG_INIT,
                 "ULTRASONIC: OK: uart=%d tx_gpio=%d rx_gpio=%d baud=%d",
                 g_ultrasonic_cfg.uart_num,
                 g_ultrasonic_cfg.tx_gpio,
                 g_ultrasonic_cfg.rx_gpio,
                 g_ultrasonic_cfg.baud_rate);
    else
        ESP_LOGE(TAG_INIT, "ULTRASONIC: FAILED: uart=%d", g_ultrasonic_cfg.uart_num);
#endif

    if (relay_init_ok)
        ESP_LOGI(TAG_INIT,
                 "RELAY: OK: gpio=%d active_level=%s start=DISABLED",
                 g_relay_cfg.gpio,
                 g_relay_cfg.active_high ? "HIGH" : "LOW");
    else
        ESP_LOGE(TAG_INIT, "RELAY: FAILED: gpio=%d", g_relay_cfg.gpio);

#ifdef CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS
    ESP_LOGW(TAG_INIT, "PLANNER: BYPASSED: liveness/state/autonomy checks disabled");
#endif

#ifdef CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS
    ESP_LOGW(TAG_INIT, "CONTROL: BYPASSED: liveness/state checks disabled");
#endif

    if (heartbeat_ready)
        ESP_LOGI(TAG_INIT, "HEARTBEAT_LED: OK: gpio=%d state=init", HEARTBEAT_LED_GPIO);
    else
        ESP_LOGW(TAG_INIT, "HEARTBEAT_LED: UNAVAILABLE: gpio=%d (non-critical)", HEARTBEAT_LED_GPIO);
}

static void log_component_lost(const char *name, const char *detail) {
#ifdef CONFIG_LOG_COMPONENT_HEALTH_TRANSITIONS
    if (detail && detail[0] != '\0') {
        ESP_LOGE(TAG_INIT, "%s: LOST: %s", name, detail);
    } else {
        ESP_LOGE(TAG_INIT, "%s: LOST", name);
    }
#else
    (void)name;
    (void)detail;
#endif
}

static void log_component_regained(const char *name) {
#ifdef CONFIG_LOG_COMPONENT_HEALTH_TRANSITIONS
    ESP_LOGI(TAG_INIT, "%s: REGAINED", name);
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

static void retry_failed_components(void) {
    // Pace retries at RETRY_INTERVAL_MS to avoid hammering init calls every
    // safety loop tick. Retries remain infinite (no cap).
    uint32_t now_ms = get_time_ms();
    if ((now_ms - g_last_retry_ms) < RETRY_INTERVAL_MS) return;
    g_last_retry_ms = now_ms;

#ifndef CONFIG_BYPASS_INPUT_PUSH_BUTTON
    if (!g_push_button_init_ok) {
        esp_err_t err = push_button_hb2es544_init(&g_push_button_cfg);
        bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_PUSH_BUTTON
        if (!recovered) {
            ESP_LOGI(TAG_INIT, "PUSH_BUTTON retry failed: %s", esp_err_to_name(err));
        }
#endif
        if (recovered) {
            g_push_button_init_ok = true;
            log_component_regained("PUSH_BUTTON");
        }
    }
#endif

#ifndef CONFIG_BYPASS_INPUT_RF_REMOTE
    if (!g_rf_remote_init_ok) {
        esp_err_t err = rf_remote_ev1527_init(&g_rf_remote_cfg);
        bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_RF_REMOTE
        if (!recovered) {
            ESP_LOGI(TAG_INIT, "RF_REMOTE retry failed: %s", esp_err_to_name(err));
        }
#endif
        if (recovered) {
            g_rf_remote_init_ok = true;
            log_component_regained("RF_REMOTE");
        }
    }
#endif

    if (!g_relay_init_ok) {
        esp_err_t err = relay_srd05vdc_init(&g_relay_cfg);
        bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_POWER_RELAY
        if (!recovered) {
            ESP_LOGI(TAG_INIT, "RELAY retry failed: %s", esp_err_to_name(err));
        }
#endif
        if (recovered) {
            g_relay_init_ok = true;
            log_component_regained("RELAY (driver-level)");
        }
    }

#ifndef CONFIG_BYPASS_INPUT_ULTRASONIC
    if (!g_ultrasonic_init_ok) {
        esp_err_t err = ultrasonic_a02yyuw_init(&g_ultrasonic_cfg);
        bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_ULTRASONIC
        if (!recovered) {
            ESP_LOGI(TAG_INIT, "ULTRASONIC retry failed: %s", esp_err_to_name(err));
        }
#endif
        if (recovered) {
            g_ultrasonic_init_ok = true;
            g_ultrasonic_health_prev = false;
            g_ultrasonic_health_seen = false;
        }
    }
#endif

    if (!g_twai_ready) {
        esp_err_t err = can_twai_recover_with_reinit(TWAI_TX_GPIO, TWAI_RX_GPIO, TAG_INIT);
        bool recovered = (err == ESP_OK);
#ifdef CONFIG_LOG_RETRY_TWAI
        if (!recovered) {
            ESP_LOGI(TAG_INIT, "TWAI retry failed: %s", esp_err_to_name(err));
        }
#endif
        if (recovered) {
            g_twai_ready = true;
            log_component_regained("TWAI");
        }
    }
}

// ============================================================================
// LED State Mapping
// ============================================================================

static void update_safety_led_mode() {
    uint8_t red = 0;
    uint8_t green = 0;
    uint8_t blue = 0;
    const char *reason = "target_ready";

    if (g_estop_fault_code != NODE_FAULT_NONE) {
        red = LED_LEVEL;
        reason = node_fault_to_string(g_estop_fault_code);
    } else {
        switch (g_target_state) {
            case NODE_STATE_READY:
                green = LED_LEVEL;
                reason = "target_ready";
                break;
            case NODE_STATE_ENABLING:
                red = LED_LEVEL;
                green = LED_ORANGE_GREEN;
                reason = "target_enabling";
                break;
            case NODE_STATE_ACTIVE:
                blue = LED_LEVEL;
                reason = "target_active";
                break;
            default:
                red = LED_LEVEL;
                green = LED_ORANGE_GREEN;
                reason = "target_other";
                break;
        }
    }

    led_ws2812_set_manual_color(red, green, blue, reason);
}

// ============================================================================
// Recovery State
// ============================================================================

static uint8_t g_can_tx_fail_count = 0;
static bool g_can_recovery_in_progress = false;

// Spinlock for CAN TX tracking / recovery (used by safety_task + heartbeat_task)
static portMUX_TYPE g_can_tx_lock = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// CAN Recovery Helper
// ============================================================================

// Attempt to recover CAN bus communication.
// Returns true on success, false on failure (no reset).
static bool attempt_can_recovery() {
    bool was_ready = g_twai_ready;
#ifdef CONFIG_LOG_CAN_RECOVERY
    ESP_LOGI(TAG, "Attempting CAN recovery (stop/start, then re-init if needed)");
#endif
    esp_err_t err = can_twai_recover_with_reinit(TWAI_TX_GPIO, TWAI_RX_GPIO, TAG);
    if (err == ESP_OK) {
        g_twai_ready = true;
        if (!was_ready) {
            log_component_regained("TWAI");
        }
#ifdef CONFIG_LOG_CAN_RECOVERY
        ESP_LOGI(TAG, "CAN recovery succeeded");
#endif
        g_can_tx_fail_count = 0;
        return true;
    }

    mark_component_lost(&g_twai_ready, "TWAI", "CAN recovery failed");
#ifdef CONFIG_LOG_CAN_RECOVERY
    ESP_LOGE(TAG, "CAN recovery FAILED: %s (will retry)", esp_err_to_name(err));
#endif
    return false;
}

// Track CAN TX result. On consecutive failures, trigger recovery.
// Protected by g_can_tx_lock to prevent concurrent recovery from safety_task + heartbeat_task.
static void track_can_tx(esp_err_t err) {
    bool trigger_recovery = false;

    taskENTER_CRITICAL(&g_can_tx_lock);
    if (err == ESP_OK) {
        g_can_tx_fail_count = 0;
        taskEXIT_CRITICAL(&g_can_tx_lock);
        return;
    }
    g_can_tx_fail_count++;
    if (g_can_tx_fail_count >= CAN_TX_FAIL_THRESHOLD) {
        bool bus_ok = can_twai_bus_ok();
        g_can_tx_fail_count = 0;  // reset regardless
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
#ifdef CONFIG_LOG_CAN_RECOVERY
        ESP_LOGE(TAG, "CAN bus unhealthy after %d TX failures, initiating recovery",
                 CAN_TX_FAIL_THRESHOLD);
#endif
        mark_component_lost(&g_twai_ready, "TWAI", "runtime bus unhealthy after tx failures");
        (void)attempt_can_recovery();
        taskENTER_CRITICAL(&g_can_tx_lock);
        g_can_recovery_in_progress = false;
        taskEXIT_CRITICAL(&g_can_tx_lock);
    }
}

// ============================================================================
// Safety Heartbeat Send Helper
// ============================================================================

// Send the safety heartbeat on CAN. Called from safety_task on state change
// and from heartbeat_task on the 100ms periodic timer.
static void send_safety_heartbeat(bool log_as_change) {
    uint8_t data[8] = {0};
    taskENTER_CRITICAL(&g_safety_hb_seq_lock);
    uint8_t seq = g_safety_hb_seq;
    g_safety_hb_seq = (uint8_t)(seq + 1);
    taskEXIT_CRITICAL(&g_safety_hb_seq_lock);
    node_heartbeat_t hb = {
        .sequence = seq,
        .state = g_target_state,
        .fault_code = g_estop_fault_code,
        .flags = 0,
    };
    can_encode_heartbeat(data, &hb);
    esp_err_t err = can_twai_send(CAN_ID_SAFETY_HEARTBEAT, data, pdMS_TO_TICKS(10));
    track_can_tx(err);

    (void)log_as_change;  // only used when CONFIG_LOG_CAN_HEARTBEAT_TX is enabled
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
        if (log_as_change) {
            ESP_LOGI(TAG_TX, "Heartbeat: target=%s fault:%s",
                     node_state_to_string(g_target_state),
                     node_fault_to_string(g_estop_fault_code));
        } else {
            ESP_LOGI(TAG_TX, "Heartbeat periodic: target=%s fault:%s",
                     node_state_to_string(g_target_state),
                     node_fault_to_string(g_estop_fault_code));
        }
    }
#endif
}

// ============================================================================
// CAN RX Task
// ============================================================================

// Receives CAN heartbeats from Planner and Control ESP32.
// Updates heartbeat monitor timestamps and tracks node states/faults/flags.
void can_rx_task(void *param) {
    (void)param;
    twai_message_t msg{};
    while (true) {
        if (can_twai_receive(&msg, CAN_RX_TIMEOUT) != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        if (msg.extd || msg.rtr) continue;
        led_ws2812_mark_activity(xTaskGetTickCount());

        // Process Planner heartbeat (0x110)
        if (msg.identifier == CAN_ID_PLANNER_HEARTBEAT && msg.data_length_code >= 4) {
            node_heartbeat_t hb;
            can_decode_heartbeat(msg.data, &hb);
            heartbeat_monitor_update(&g_hb_monitor, g_node_planner, hb.sequence, hb.state);

            g_planner_state = hb.state;
            g_planner_flags = hb.flags;

#ifdef CONFIG_LOG_CAN_HEARTBEAT_RX
            static uint8_t prev_planner_state = 0xFF;
            uint8_t prev_state = prev_planner_state;
            prev_planner_state = hb.state;
            if (hb.state == NODE_STATE_FAULT && prev_state != NODE_STATE_FAULT) {
                ESP_LOGI(TAG_RX, "Planner FAULT: %s", node_fault_to_string(hb.fault_code));
            } else if (hb.state != NODE_STATE_FAULT && prev_state == NODE_STATE_FAULT) {
                ESP_LOGI(TAG_RX, "Planner fault cleared");
            }
            if (hb.state != prev_state) {
                ESP_LOGI(TAG_RX, "Planner state: %s -> %s",
                         node_state_to_string(prev_state), node_state_to_string(hb.state));
            } else {
                ESP_LOGI(TAG_RX, "Planner HB: seq=%u state=%s fault=%s flags=0x%02X",
                         hb.sequence, node_state_to_string(hb.state),
                         node_fault_to_string(hb.fault_code), hb.flags);
            }
#endif
        }

        // Process Control ESP32 heartbeat (0x120)
        else if (msg.identifier == CAN_ID_CONTROL_HEARTBEAT && msg.data_length_code >= 4) {
            node_heartbeat_t hb;
            can_decode_heartbeat(msg.data, &hb);
            heartbeat_monitor_update(&g_hb_monitor, g_node_control, hb.sequence, hb.state);

            g_control_node_state = hb.state;
            g_control_flags = hb.flags;

#ifdef CONFIG_LOG_CAN_HEARTBEAT_RX
            static uint8_t prev_control_state = 0xFF;
            uint8_t prev_state = prev_control_state;
            prev_control_state = hb.state;
            if (hb.state == NODE_STATE_FAULT && prev_state != NODE_STATE_FAULT) {
                ESP_LOGI(TAG_RX, "Control FAULT: %s", node_fault_to_string(hb.fault_code));
            } else if (hb.state != NODE_STATE_FAULT && prev_state == NODE_STATE_FAULT) {
                ESP_LOGI(TAG_RX, "Control fault cleared");
            }
            if (hb.state != prev_state) {
                ESP_LOGI(TAG_RX, "Control state: %s -> %s",
                         node_state_to_string(prev_state), node_state_to_string(hb.state));
            } else {
                ESP_LOGI(TAG_RX, "Control HB: seq=%u state=%s fault=%s flags=0x%02X",
                         hb.sequence, node_state_to_string(hb.state),
                         node_fault_to_string(hb.fault_code), hb.flags);
            }
#endif
        }
    }
}

// ============================================================================
// Safety Task
// ============================================================================

// Main safety monitoring loop. Runs at 20Hz.
// Reads e-stop inputs, checks heartbeat timeouts, controls power relay,
// runs the system state machine, and sends immediate heartbeat on state change.
//
// Decision logic (e-stop bitmask evaluation, relay) is delegated to the pure
// safety_logic module. State advancement is delegated to the pure
// system_state module. Both are unit-testable on host.
void safety_task(void *param) {
    (void)param;
    // One-shot autonomy request gate:
    // - latch on Planner request rising edge
    // - consume when transitioning READY -> ENABLING
    // - re-arm only after request drops
    bool request_level_prev = false;
    bool request_latched = false;

    while (true) {
        retry_failed_components();

        // Read hardware inputs into the pure-function input struct.
        // Push-button and RF remote use disengage debounce: engage is
        // immediate (safety-critical), disengage requires ESTOP_DISENGAGE_COUNT
        // consecutive clear reads to filter contact bounce / RF glitches.
        bool raw_push_button = g_push_button_init_ok
                               ? push_button_hb2es544_read_active(&g_push_button_cfg)
                               : true;
        bool raw_rf_remote   = g_rf_remote_init_ok
                               ? rf_remote_ev1527_is_active(&g_rf_remote_cfg)
                               : true;

        // Push-button debounce
        if (raw_push_button) {
            g_push_button_debounced = true;
            g_push_button_clear_count = 0;
        } else {
            if (g_push_button_clear_count < ESTOP_DISENGAGE_COUNT)
                g_push_button_clear_count++;
            if (g_push_button_clear_count >= ESTOP_DISENGAGE_COUNT)
                g_push_button_debounced = false;
        }

        // RF remote debounce
        if (raw_rf_remote) {
            g_rf_remote_debounced = true;
            g_rf_remote_clear_count = 0;
        } else {
            if (g_rf_remote_clear_count < ESTOP_DISENGAGE_COUNT)
                g_rf_remote_clear_count++;
            if (g_rf_remote_clear_count >= ESTOP_DISENGAGE_COUNT)
                g_rf_remote_debounced = false;
        }

        // Ultrasonic health with hysteresis: require ULTRASONIC_HEALTH_HYSTERESIS
        // consecutive agreeing samples before changing the reported health state.
        bool raw_ultrasonic_healthy = g_ultrasonic_init_ok
                                      ? ultrasonic_a02yyuw_is_healthy()
                                      : false;
        bool filtered_ultrasonic_healthy;
        if (g_ultrasonic_health_seen) {
            if (raw_ultrasonic_healthy != g_ultrasonic_health_prev) {
                g_ultrasonic_health_counter++;
                if (g_ultrasonic_health_counter >= ULTRASONIC_HEALTH_HYSTERESIS) {
                    g_ultrasonic_health_prev = raw_ultrasonic_healthy;
                    g_ultrasonic_health_counter = 0;
                }
            } else {
                g_ultrasonic_health_counter = 0;
            }
            filtered_ultrasonic_healthy = g_ultrasonic_health_prev;
        } else {
            // First sample: accept immediately without logging regained
            g_ultrasonic_health_prev = raw_ultrasonic_healthy;
            g_ultrasonic_health_seen = true;
            g_ultrasonic_health_counter = 0;
            filtered_ultrasonic_healthy = raw_ultrasonic_healthy;
        }

        safety_inputs_t inputs = {
            .push_button_active = g_push_button_debounced,
            .rf_remote_active   = g_rf_remote_debounced,
            .ultrasonic_too_close = g_ultrasonic_init_ok
                                    ? ultrasonic_a02yyuw_is_too_close(ULTRASONIC_STOP_DISTANCE_MM, NULL)
                                    : false,
            .ultrasonic_healthy = filtered_ultrasonic_healthy,
            .planner_alive = true,   // updated below from heartbeat monitor
            .control_alive = true,   // updated below from heartbeat monitor
            .planner_error = (g_planner_state == NODE_STATE_FAULT),
            .control_error = (g_control_node_state == NODE_STATE_FAULT),
        };

        // Check heartbeat timeouts
        heartbeat_monitor_check_timeouts(&g_hb_monitor);
        inputs.planner_alive = heartbeat_monitor_is_alive(&g_hb_monitor, g_node_planner);
        inputs.control_alive = heartbeat_monitor_is_alive(&g_hb_monitor, g_node_control);

        // Apply test bypasses
#ifdef CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS
        inputs.planner_alive = true;
        inputs.planner_error = false;
#endif
#ifdef CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS
        inputs.control_alive = true;
        inputs.control_error = false;
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
#ifndef CONFIG_BYPASS_INPUT_ULTRASONIC
        {
            static bool s_ultrasonic_log_prev = false;
            static bool s_ultrasonic_log_seen = false;
            if (g_ultrasonic_init_ok) {
                if (!s_ultrasonic_log_seen) {
                    // Baseline assumes healthy when init succeeded to avoid
                    // spurious "REGAINED" from the boot-time sensor startup delay.
                    s_ultrasonic_log_prev = g_ultrasonic_init_ok ? true : filtered_ultrasonic_healthy;
                    s_ultrasonic_log_seen = true;
                } else if (filtered_ultrasonic_healthy != s_ultrasonic_log_prev) {
                    if (filtered_ultrasonic_healthy) {
                        log_component_regained("ULTRASONIC");
                    } else {
                        log_component_lost("ULTRASONIC", "runtime health timeout");
                    }
                    s_ultrasonic_log_prev = filtered_ultrasonic_healthy;
                }
            }
        }
#endif

#ifdef CONFIG_LOG_SAFETY_ESTOP_INPUTS
        ESP_LOGI(TAG, "button=%d remote=%d ultra_close=%d ultra_healthy=%d planner=%s(%s) control=%s(%s)",
                 inputs.push_button_active, inputs.rf_remote_active,
                 inputs.ultrasonic_too_close, inputs.ultrasonic_healthy,
                 inputs.planner_alive ? "alive" : "DEAD",
                 node_state_to_string(g_planner_state),
                 inputs.control_alive ? "alive" : "DEAD",
                 node_state_to_string(g_control_node_state));
#endif

        // Evaluate all safety conditions (pure function — no side effects)
        safety_decision_t decision = safety_evaluate(&inputs);

        // Missing critical output/transport components force fail-safe behavior.
        if (!g_relay_init_ok || !g_twai_ready) {
            decision.estop_active = true;
            decision.fault_code = NODE_FAULT_GENERAL;
            decision.relay_enable = false;
        }

        // Publish fault code for heartbeat task
        g_estop_fault_code = decision.fault_code;

        // Run system state machine (pure function)
        system_state_inputs_t ss_in = {
            .current_target = g_target_state,
            .estop_active = decision.estop_active,
            .planner_state = g_planner_state,
            .control_state = g_control_node_state,
            .planner_alive = inputs.planner_alive,
            .control_alive = inputs.control_alive,
            .planner_enable_complete = (g_planner_flags & HEARTBEAT_FLAG_ENABLE_COMPLETE) != 0,
            .control_enable_complete = (g_control_flags & HEARTBEAT_FLAG_ENABLE_COMPLETE) != 0,
            .autonomy_request = false,
            .autonomy_hold = false,
        };

        // Planner bypass: simulate a cooperative Planner that mirrors the
        // current target state and always signals enable_complete.
#ifdef CONFIG_BYPASS_PLANNER_STATE_MIRROR
        ss_in.planner_state = (g_target_state >= NODE_STATE_ENABLING)
                              ? NODE_STATE_ENABLING : NODE_STATE_READY;
        ss_in.planner_enable_complete = true;
#endif
        // Control bypass: same pattern for Control.
#ifdef CONFIG_BYPASS_CONTROL_STATE_MIRROR
        ss_in.control_state = (g_target_state >= NODE_STATE_ENABLING)
                              ? NODE_STATE_ENABLING : NODE_STATE_READY;
        ss_in.control_enable_complete = true;
#endif

        bool planner_request_level =
            (g_planner_flags & HEARTBEAT_FLAG_AUTONOMY_REQUEST) != 0;
        bool request_accept_window = (ss_in.current_target == NODE_STATE_READY &&
                                      !ss_in.estop_active &&
                                      ss_in.planner_alive &&
                                      ss_in.control_alive &&
                                      ss_in.planner_state == NODE_STATE_READY &&
                                      ss_in.control_state == NODE_STATE_READY);
#ifdef CONFIG_BYPASS_PLANNER_AUTONOMY_GATE
        planner_request_level = true;
#endif

        if (!planner_request_level) {
#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
            if (request_level_prev)
                ESP_LOGI(TAG, "Autonomy request re-armed (request dropped)");
#endif
            request_level_prev = false;
        } else if (!request_level_prev) {
            request_level_prev = true;
            if (request_accept_window) {
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
        bool waiting_for_request = (ss_in.current_target == NODE_STATE_READY &&
                                    !ss_in.estop_active &&
                                    ss_in.planner_alive &&
                                    ss_in.control_alive &&
                                    ss_in.planner_state == NODE_STATE_READY &&
                                    ss_in.control_state == NODE_STATE_READY &&
                                    !ss_in.autonomy_request);
        if (waiting_for_request && !prev_waiting_for_request) {
            ESP_LOGI(TAG, "Waiting for autonomy request from Planner/Orin");
        } else if (!waiting_for_request && prev_waiting_for_request) {
            ESP_LOGI(TAG, "Autonomy request wait cleared");
        }
        prev_waiting_for_request = waiting_for_request;
#endif

        system_state_result_t ss_out = system_state_step(&ss_in);

#ifndef CONFIG_BYPASS_PLANNER_AUTONOMY_GATE
        if (ss_out.target_changed &&
            g_target_state == NODE_STATE_READY &&
            ss_out.new_target == NODE_STATE_ENABLING &&
            request_latched) {
            request_latched = false;
#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
            ESP_LOGI(TAG, "Autonomy request consumed");
#endif
        }
#endif

#ifdef CONFIG_LOG_SAFETY_STATE_TICK
        ESP_LOGI(TAG, "tick estop=%d planner=%s control=%s req=%d target=%s -> %s",
                 decision.estop_active,
                 node_state_to_string(ss_in.planner_state),
                 node_state_to_string(ss_in.control_state),
                 ss_in.autonomy_request,
                 node_state_to_string(g_target_state),
                 node_state_to_string(ss_out.new_target));
#endif

#ifdef CONFIG_LOG_SAFETY_STATE_CHANGES
        if (ss_out.target_changed) {
            const char *reason = "none";
            if (g_target_state == NODE_STATE_READY &&
                ss_out.new_target == NODE_STATE_ENABLING)
                reason = "autonomy_request";
            else if (g_target_state == NODE_STATE_ENABLING &&
                     ss_out.new_target == NODE_STATE_ACTIVE)
                reason = "enable_complete";
            else if (ss_out.new_target == NODE_STATE_READY) {
                if ((g_target_state == NODE_STATE_ENABLING || g_target_state == NODE_STATE_ACTIVE) &&
                    !ss_in.autonomy_hold &&
                    decision.fault_code == NODE_FAULT_NONE)
                    reason = "autonomy_halt";
                else
                    reason = node_fault_to_string(decision.fault_code);
            }

            ESP_LOGI(TAG, "System target: %s -> %s (reason: %s)",
                     node_state_to_string(g_target_state),
                     node_state_to_string(ss_out.new_target),
                     reason);
        }
#endif

        // Apply new target state
        g_target_state = ss_out.new_target;

        // Control power relay based on safety decision
        if (g_relay_init_ok) {
            if (decision.relay_enable) {
                esp_err_t relay_err = relay_srd05vdc_enable(&g_relay_cfg);
                if (relay_err != ESP_OK) {
                    ESP_LOGE(TAG, "relay_srd05vdc_enable FAILED: %s — relay may be stuck OFF", esp_err_to_name(relay_err));
                    mark_component_lost(&g_relay_init_ok, "RELAY", "runtime enable command failed");
                }
            } else {
                esp_err_t relay_err = relay_srd05vdc_disable(&g_relay_cfg);
                if (relay_err != ESP_OK) {
                    ESP_LOGE(TAG, "relay_srd05vdc_disable FAILED: %s — relay may be stuck ON!", esp_err_to_name(relay_err));
                    mark_component_lost(&g_relay_init_ok, "RELAY", "runtime disable command failed");
                }
            }
        }

        // Relay state changes are logged by the relay_srd05vdc component itself

        // Send immediate heartbeat on target state change
        if (ss_out.target_changed) {
            send_safety_heartbeat(true);
        }

        vTaskDelay(SAFETY_LOOP_INTERVAL);
    }
}

// ============================================================================
// Heartbeat Task
// ============================================================================

// Sends Safety heartbeat at 100ms and updates WS2812 LED state indicator.
// Also checks CAN bus health and attempts recovery if needed.
void heartbeat_task(void *param) {
    (void)param;

    while (true) {
        TickType_t now = xTaskGetTickCount();

        // Update LED heartbeat
        update_safety_led_mode();
        led_ws2812_tick(&g_heartbeat_led_cfg, now);

        // Send Safety heartbeat (periodic)
        send_safety_heartbeat(false);

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
            esp_err_t err = can_twai_recover_bus_off();
            if (err == ESP_OK) {
                if (!g_twai_ready) {
                    log_component_regained("TWAI");
                }
                g_twai_ready = true;
            } else {
                mark_component_lost(&g_twai_ready, "TWAI", "bus-off recovery failed");
#ifdef CONFIG_LOG_CAN_RECOVERY
                ESP_LOGE(TAG, "CAN bus-off recovery failed: %s", esp_err_to_name(err));
#endif
            }
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

// Initialization and task startup. Configures all peripherals and e-stop inputs,
// registers heartbeat monitor nodes, then spawns CAN RX, Safety, and Heartbeat tasks.
// Component failures keep Safety fail-safe and are retried indefinitely.
void main_task(void *param) {
    (void)param;

    // Initialize heartbeat monitor
    heartbeat_monitor_config_t hb_cfg = { .name = "SAFETY" };
    heartbeat_monitor_init(&g_hb_monitor, &hb_cfg);
    
    // Register nodes to monitor (Safety only cares about Planner and Control)
    g_node_planner = heartbeat_monitor_register(&g_hb_monitor, "Planner", HEARTBEAT_TIMEOUT_MS);
    g_node_control = heartbeat_monitor_register(&g_hb_monitor, "Control", HEARTBEAT_TIMEOUT_MS);

    // Configure push button e-stop (mxuteek HB2-ES544)
    g_push_button_cfg = {
        .gpio = PUSH_BUTTON_HB2ES544_GPIO,
        .active_level = PUSH_BUTTON_HB2ES544_ACTIVE_LEVEL,
        .enable_pullup = true,
        .enable_pulldown = false,
    };

    esp_err_t err = push_button_hb2es544_init(&g_push_button_cfg);
#ifdef CONFIG_BYPASS_INPUT_PUSH_BUTTON
    g_push_button_init_ok = true;
#else
    g_push_button_init_ok = (err == ESP_OK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_INIT, "Push button init failed: %s (will retry in background)",
                 esp_err_to_name(err));
    }
#endif

    // Configure RF remote e-stop (DieseRC EV1527)
    g_rf_remote_cfg = {
        .gpio = RF_REMOTE_EV1527_GPIO,
        .active_level = RF_REMOTE_EV1527_ACTIVE_LEVEL,
        .enable_pullup = true,
        .enable_pulldown = false,
    };

    err = rf_remote_ev1527_init(&g_rf_remote_cfg);
#ifdef CONFIG_BYPASS_INPUT_RF_REMOTE
    g_rf_remote_init_ok = true;
#else
    g_rf_remote_init_ok = (err == ESP_OK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_INIT, "RF remote init failed: %s (will retry in background)",
                 esp_err_to_name(err));
    }
#endif

    // Configure power relay (starts disabled = safe)
    g_relay_cfg = {
        .gpio = POWER_RELAY_GPIO,
        .active_high = POWER_RELAY_ACTIVE_HIGH,
        .enable_pullup = false,
        .enable_pulldown = true,  // Pull down to ensure OFF on reset
    };

    err = relay_srd05vdc_init(&g_relay_cfg);
    g_relay_init_ok = (err == ESP_OK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_INIT, "Power relay init failed: %s (will retry in background)",
                 esp_err_to_name(err));
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
    if (err != ESP_OK) {
        ESP_LOGE(TAG_INIT, "Ultrasonic init failed: %s (will retry in background)",
                 esp_err_to_name(err));
    }
#endif

    // Initialize TWAI for CAN communication. Retries continue in safety loop.
    err = can_twai_init_default(TWAI_TX_GPIO, TWAI_RX_GPIO);
    g_twai_ready = (err == ESP_OK);
    if (err != ESP_OK) {
        ESP_LOGE(TAG_INIT, "TWAI init failed: %s (will retry in background)",
                 esp_err_to_name(err));
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

    err = led_ws2812_init(&g_heartbeat_led_cfg);
    bool heartbeat_ready = (err == ESP_OK);
    if (!heartbeat_ready) {
        ESP_LOGE(TAG_INIT, "HEARTBEAT_LED init failed: %s (non-critical, no retry)",
                 esp_err_to_name(err));
    } else {
        led_ws2812_set_manual_mode(true);
        led_ws2812_set_manual_color(LED_LEVEL, LED_ORANGE_GREEN, 0, "state_init");
    }

    log_startup_device_status(g_twai_ready, g_relay_init_ok, heartbeat_ready);

    // Start CAN RX task first so heartbeat frames can be received during init wait
    if (xTaskCreate(can_rx_task, "can_rx", CAN_RX_TASK_STACK, nullptr, CAN_RX_TASK_PRIO, nullptr) != pdPASS) {
        ESP_LOGE(TAG_INIT, "Failed to create CAN RX task, restarting");
        esp_restart();
    }

    // Wait for Planner/Control heartbeats (if not bypassed)
    static constexpr TickType_t HB_INIT_WAIT = pdMS_TO_TICKS(1000);
    static constexpr TickType_t HB_INIT_POLL = pdMS_TO_TICKS(50);

#ifndef CONFIG_BYPASS_PLANNER_LIVENESS_CHECKS
    {
        TickType_t deadline = xTaskGetTickCount() + HB_INIT_WAIT;
        while (!heartbeat_monitor_is_alive(&g_hb_monitor, g_node_planner) &&
               xTaskGetTickCount() < deadline) {
            vTaskDelay(HB_INIT_POLL);
        }
        if (heartbeat_monitor_is_alive(&g_hb_monitor, g_node_planner))
            ESP_LOGI(TAG_INIT, "PLANNER: OK: heartbeat detected");
        else
            ESP_LOGE(TAG_INIT, "PLANNER: FAILED: no heartbeat within 1000ms");
    }
#endif

#ifndef CONFIG_BYPASS_CONTROL_LIVENESS_CHECKS
    {
        TickType_t deadline = xTaskGetTickCount() + HB_INIT_WAIT;
        while (!heartbeat_monitor_is_alive(&g_hb_monitor, g_node_control) &&
               xTaskGetTickCount() < deadline) {
            vTaskDelay(HB_INIT_POLL);
        }
        if (heartbeat_monitor_is_alive(&g_hb_monitor, g_node_control))
            ESP_LOGI(TAG_INIT, "CONTROL: OK: heartbeat detected");
        else
            ESP_LOGE(TAG_INIT, "CONTROL: FAILED: no heartbeat within 1000ms");
    }
#endif

    // Start remaining tasks
    if (xTaskCreate(safety_task, "safety", SAFETY_TASK_STACK, nullptr, SAFETY_TASK_PRIO, nullptr) != pdPASS) {
        ESP_LOGE(TAG_INIT, "Failed to create safety task, restarting");
        esp_restart();
    }
    if (xTaskCreate(heartbeat_task, "heartbeat", HEARTBEAT_TASK_STACK, nullptr, HEARTBEAT_TASK_PRIO, nullptr) != pdPASS) {
        ESP_LOGE(TAG_INIT, "Failed to create heartbeat task, restarting");
        esp_restart();
    }

    // Main task complete - delete self
    vTaskDelete(nullptr);
}
}

extern "C" void app_main(void) {
    if (xTaskCreate(main_task, "main_task", 4096, nullptr, 5, nullptr) != pdPASS) {
        ESP_LOGE("SAFETY_INIT", "Failed to create main_task, restarting");
        esp_restart();
    }
}
