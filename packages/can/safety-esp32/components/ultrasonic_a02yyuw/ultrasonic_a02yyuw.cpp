/**
 * @file ultrasonic_a02yyuw.cpp
 * @brief A02YYUW ultrasonic distance sensor driver over UART.
 */

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "ultrasonic_a02yyuw.hh"

namespace {

[[maybe_unused]] static const char *TAG = "ULTRASONIC";

// ============================================================================
// Configuration
// ============================================================================

constexpr int RX_TASK_STACK = 4096;
constexpr UBaseType_t RX_TASK_PRIO = 10;
constexpr int UART_BUFFER_SIZE = 256;
constexpr TickType_t UART_READ_TIMEOUT = pdMS_TO_TICKS(100);
constexpr TickType_t MAX_SAMPLE_AGE = pdMS_TO_TICKS(200);  // Reject stale readings
constexpr uint16_t DISTANCE_MIN_MM = 30;
constexpr uint16_t DISTANCE_MAX_MM = 4500;

// ============================================================================
// Module State
// ============================================================================

static portMUX_TYPE s_ultrasonic_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_last_distance_mm = 0;
static TickType_t s_last_update_tick = 0;
static bool s_has_measurement = false;
static TaskHandle_t s_rx_task = nullptr;
static bool s_uart_installed = false;
static uint32_t s_range_drop_count = 0;
static volatile bool s_stop_requested = false;
static uint8_t s_parser_frame[4] = {0};
static int s_parser_idx = 0;

// Internal copy of config - ensures background task always has a valid pointer
// regardless of how the caller allocated the original config struct
static ultrasonic_a02yyuw_config_t s_config = {};

// ============================================================================
// Stream Parser
// ============================================================================

// Parse A02YYUW UART protocol: [0xFF] [HIGH] [LOW] [CHECKSUM]
// Checksum = (0xFF + HIGH + LOW) & 0xFF
// Returns true when valid frame parsed, writes distance to *distance_mm
static void ultrasonic_a02yyuw_reset_parser_state(void) {
    s_parser_idx = 0;
    for (int i = 0; i < 4; ++i) s_parser_frame[i] = 0;
}

static bool ultrasonic_a02yyuw_parse_stream(const uint8_t *data, int len, uint16_t *distance_mm) {
    bool found = false;

    for (int i = 0; i < len; ++i) {
        uint8_t b = data[i];

        // Look for frame start byte
        if (s_parser_idx == 0) {
            if (b != 0xFF) continue;
            s_parser_frame[s_parser_idx++] = b;
            continue;
        }

        s_parser_frame[s_parser_idx++] = b;
        if (s_parser_idx == 4) {
            s_parser_idx = 0;
            // Verify checksum
            uint8_t sum = (uint8_t)(s_parser_frame[0] + s_parser_frame[1] + s_parser_frame[2]);
            if (sum == s_parser_frame[3]) {
                uint16_t parsed_mm = (uint16_t)((s_parser_frame[1] << 8) | s_parser_frame[2]);
                if (parsed_mm >= DISTANCE_MIN_MM && parsed_mm <= DISTANCE_MAX_MM) {
                    *distance_mm = parsed_mm;
                    found = true;
                    // Continue processing — use the freshest valid frame
                } else {
                    s_range_drop_count++;
#ifdef CONFIG_LOG_INPUT_ULTRASONIC_PARSE_ERRORS
                    if (s_range_drop_count == 1 || (s_range_drop_count % 20) == 0) {
                        ESP_LOGW(TAG, "Out-of-range sample dropped: %u mm (valid %u-%u)",
                                 parsed_mm, DISTANCE_MIN_MM, DISTANCE_MAX_MM);
                    }
#endif
                }
            }
            // Bad checksum - frame discarded, continue searching
        }
    }

    return found;
}

static void ultrasonic_a02yyuw_update_distance(uint16_t distance_mm) {
    taskENTER_CRITICAL(&s_ultrasonic_lock);
    s_last_distance_mm = distance_mm;
    s_last_update_tick = xTaskGetTickCount();
    s_has_measurement = true;
    taskEXIT_CRITICAL(&s_ultrasonic_lock);
}

// ============================================================================
// UART RX Task
// ============================================================================

// Background task continuously reads UART and updates distance
static void ultrasonic_a02yyuw_uart_rx_task(void *arg) {
    const ultrasonic_a02yyuw_config_t *config = static_cast<const ultrasonic_a02yyuw_config_t *>(arg);
    uint8_t buffer[UART_BUFFER_SIZE];
    uint16_t distance_mm = 0;

#ifdef CONFIG_LOG_INPUT_ULTRASONIC_RX
    ESP_LOGI(TAG, "UART RX task started (UART%d, TX=GPIO%d, RX=GPIO%d, %d baud)",
             config->uart_num, config->tx_gpio, config->rx_gpio, config->baud_rate);
#endif

    while (!s_stop_requested) {
        int read_len = uart_read_bytes(config->uart_num, buffer, sizeof(buffer), UART_READ_TIMEOUT);
        
        if (read_len > 0) {
#ifdef CONFIG_LOG_INPUT_ULTRASONIC_RX
            if (read_len <= 8) {
                ESP_LOGI(TAG, "RX %d bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
                         read_len,
                         read_len > 0 ? buffer[0] : 0,
                         read_len > 1 ? buffer[1] : 0,
                         read_len > 2 ? buffer[2] : 0,
                         read_len > 3 ? buffer[3] : 0,
                         read_len > 4 ? buffer[4] : 0,
                         read_len > 5 ? buffer[5] : 0,
                         read_len > 6 ? buffer[6] : 0,
                         read_len > 7 ? buffer[7] : 0);
            } else {
                ESP_LOGI(TAG, "RX %d bytes (showing first 8): %02X %02X %02X %02X %02X %02X %02X %02X ...",
                         read_len, buffer[0], buffer[1], buffer[2], buffer[3],
                         buffer[4], buffer[5], buffer[6], buffer[7]);
            }
#endif

            if (ultrasonic_a02yyuw_parse_stream(buffer, read_len, &distance_mm)) {
                ultrasonic_a02yyuw_update_distance(distance_mm);
#ifdef CONFIG_LOG_INPUT_ULTRASONIC_DISTANCE
                ESP_LOGI(TAG, "Distance: %u mm", distance_mm);
#endif
            }
#ifdef CONFIG_LOG_INPUT_ULTRASONIC_PARSE_ERRORS
            else {
                ESP_LOGI(TAG, "Parse failed - no valid 0xFF frame in %d bytes", read_len);
            }
#endif
        }
    }

    s_rx_task = nullptr;
    vTaskDelete(nullptr);
}

}  // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t ultrasonic_a02yyuw_init(const ultrasonic_a02yyuw_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    ultrasonic_a02yyuw_deinit();
    if (s_rx_task != nullptr) {
        // Previous RX task is still shutting down; caller should retry later.
        return ESP_ERR_INVALID_STATE;
    }

    uart_config_t uart_config = {
        .baud_rate = config->baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {0, 0},
    };

    esp_err_t err = uart_driver_install(config->uart_num, UART_BUFFER_SIZE, UART_BUFFER_SIZE, 0, nullptr, 0);
    if (err != ESP_OK) return err;
    s_uart_installed = true;

    err = uart_param_config(config->uart_num, &uart_config);
    if (err != ESP_OK) {
        uart_driver_delete(config->uart_num);
        s_uart_installed = false;
        return err;
    }

    err = uart_set_pin(config->uart_num,
                       config->tx_gpio,
                       config->rx_gpio,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        uart_driver_delete(config->uart_num);
        s_uart_installed = false;
        return err;
    }

    // Copy config internally so background task has a stable pointer
    s_config = *config;

    ultrasonic_a02yyuw_reset_parser_state();

    // Start background task to continuously read sensor (uses internal config copy)
    s_stop_requested = false;
    BaseType_t task_ok = xTaskCreate(ultrasonic_a02yyuw_uart_rx_task,
                                     "ultrasonic_a02yyuw_rx",
                                     RX_TASK_STACK,
                                     (void *)&s_config,
                                     RX_TASK_PRIO,
                                     &s_rx_task);
    if (task_ok != pdPASS) {
        uart_driver_delete(config->uart_num);
        s_uart_installed = false;
        s_rx_task = nullptr;
        return ESP_FAIL;
    }

    taskENTER_CRITICAL(&s_ultrasonic_lock);
    s_has_measurement = false;
    s_last_distance_mm = 0;
    s_last_update_tick = 0;
    taskEXIT_CRITICAL(&s_ultrasonic_lock);
    s_range_drop_count = 0;

    return ESP_OK;
}

void ultrasonic_a02yyuw_deinit(void) {
    if (s_rx_task) {
        s_stop_requested = true;
        for (int i = 0; i < 50 && s_rx_task != nullptr; ++i) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (s_rx_task != nullptr) {
            // RX task still owns UART APIs; do not tear down the driver underneath it.
            ESP_LOGW(TAG, "RX task did not stop within timeout; deferring UART teardown");
            return;
        }
    }

    if (s_uart_installed) {
        (void)uart_driver_delete(s_config.uart_num);
        s_uart_installed = false;
    }

    taskENTER_CRITICAL(&s_ultrasonic_lock);
    s_has_measurement = false;
    s_last_distance_mm = 0;
    s_last_update_tick = 0;
    taskEXIT_CRITICAL(&s_ultrasonic_lock);
    ultrasonic_a02yyuw_reset_parser_state();
    s_range_drop_count = 0;
    s_stop_requested = false;
}

// ============================================================================
// Distance Reading
// ============================================================================

// Get latest distance if fresh (within MAX_SAMPLE_AGE)
bool ultrasonic_a02yyuw_get_distance_mm(uint16_t *out_distance_mm) {
    if (!out_distance_mm) return false;

    TickType_t now = xTaskGetTickCount();
    taskENTER_CRITICAL(&s_ultrasonic_lock);
    bool valid = s_has_measurement && (now - s_last_update_tick <= MAX_SAMPLE_AGE);
    uint16_t distance_mm = s_last_distance_mm;
    taskEXIT_CRITICAL(&s_ultrasonic_lock);

    if (!valid) return false;

    *out_distance_mm = distance_mm;
    return true;
}

// Check if object detected within threshold distance.
// NOTE: Returns false (not too close) when sensor has no valid data. This is
// intentional — the fail-safe is handled by the caller's safety_compute_ultrasonic_trigger()
// which combines this function with is_healthy() to trigger e-stop when the sensor
// is unhealthy, regardless of the distance reading.
bool ultrasonic_a02yyuw_is_too_close(uint16_t threshold_mm, uint16_t *out_distance_mm) {
    uint16_t distance_mm = 0;
    if (!ultrasonic_a02yyuw_get_distance_mm(&distance_mm)) return false;

    if (out_distance_mm) *out_distance_mm = distance_mm;
    return distance_mm <= threshold_mm;
}

// ============================================================================
// Health Check
// ============================================================================

// Sensor health timeout - if no data for this long, sensor is considered unhealthy
constexpr TickType_t SENSOR_HEALTH_TIMEOUT = pdMS_TO_TICKS(500);

// Check if sensor is healthy (has received valid data recently)
bool ultrasonic_a02yyuw_is_healthy(void) {
    TickType_t now = xTaskGetTickCount();
    
    taskENTER_CRITICAL(&s_ultrasonic_lock);
    bool healthy = s_has_measurement && (now - s_last_update_tick <= SENSOR_HEALTH_TIMEOUT);
    taskEXIT_CRITICAL(&s_ultrasonic_lock);
    
    return healthy;
}
