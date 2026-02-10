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

static const char *TAG = "ULTRASONIC_A02YYUW";

// ============================================================================
// Configuration
// ============================================================================

constexpr int RX_TASK_STACK = 4096;
constexpr UBaseType_t RX_TASK_PRIO = 10;
constexpr int UART_BUFFER_SIZE = 256;
constexpr TickType_t UART_READ_TIMEOUT = pdMS_TO_TICKS(100);
constexpr TickType_t MAX_SAMPLE_AGE = pdMS_TO_TICKS(200);  // Reject stale readings

// ============================================================================
// Module State
// ============================================================================

static portMUX_TYPE s_ultrasonic_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_last_distance_mm = 0;
static TickType_t s_last_update_tick = 0;
static bool s_has_measurement = false;

// Internal copy of config - ensures background task always has a valid pointer
// regardless of how the caller allocated the original config struct
static ultrasonic_a02yyuw_config_t s_config = {};

// ============================================================================
// Stream Parser
// ============================================================================

// Parse A02YYUW UART protocol: [0xFF] [HIGH] [LOW] [CHECKSUM]
// Checksum = (0xFF + HIGH + LOW) & 0xFF
// Returns true when valid frame parsed, writes distance to *distance_mm
static bool ultrasonic_a02yyuw_parse_stream(const uint8_t *data, int len, uint16_t *distance_mm) {
    static uint8_t frame[4];
    static int idx = 0;
    bool found = false;

    for (int i = 0; i < len; ++i) {
        uint8_t b = data[i];

        // Look for frame start byte
        if (idx == 0) {
            if (b != 0xFF) continue;
            frame[idx++] = b;
            continue;
        }

        frame[idx++] = b;
        if (idx == 4) {
            idx = 0;
            // Verify checksum
            uint8_t sum = (uint8_t)(frame[0] + frame[1] + frame[2]);
            if (sum == frame[3]) {
                *distance_mm = (uint16_t)((frame[1] << 8) | frame[2]);
                found = true;
                // Continue processing — use the freshest valid frame
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
    uint32_t no_data_count = 0;
    uint32_t bad_parse_count = 0;

    ESP_LOGI(TAG, "UART RX task started (UART%d, TX=GPIO%d, RX=GPIO%d, %d baud)",
             config->uart_num, config->tx_gpio, config->rx_gpio, config->baud_rate);

    while (true) {
        int read_len = uart_read_bytes(config->uart_num, buffer, sizeof(buffer), UART_READ_TIMEOUT);
        
        if (read_len > 0) {
            // Reset no-data counter when we receive bytes
            no_data_count = 0;
            
            // Log raw bytes for debugging (first 8 bytes max)
            if (read_len <= 8) {
                ESP_LOGD(TAG, "RX %d bytes: %02X %02X %02X %02X %02X %02X %02X %02X",
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
                ESP_LOGD(TAG, "RX %d bytes (showing first 8): %02X %02X %02X %02X %02X %02X %02X %02X ...",
                         read_len, buffer[0], buffer[1], buffer[2], buffer[3],
                         buffer[4], buffer[5], buffer[6], buffer[7]);
            }
            
            if (ultrasonic_a02yyuw_parse_stream(buffer, read_len, &distance_mm)) {
                ultrasonic_a02yyuw_update_distance(distance_mm);
                ESP_LOGD(TAG, "Distance: %u mm", distance_mm);
                bad_parse_count = 0;
            } else {
                bad_parse_count++;
                if (bad_parse_count % 10 == 1) {
                    ESP_LOGD(TAG, "Parse failed (count=%lu) - looking for 0xFF header", bad_parse_count);
                }
            }
        } else {
            no_data_count++;
            // Log warning every 5 seconds (50 * 100ms timeout)
            if (no_data_count == 1 || no_data_count % 50 == 0) {
                ESP_LOGW(TAG, "No data received (count=%lu) - check wiring: SensorTX→GPIO%d",
                         no_data_count, config->rx_gpio);
            }
        }
    }
}

}  // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t ultrasonic_a02yyuw_init(const ultrasonic_a02yyuw_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    ESP_LOGI(TAG, "Initializing A02YYUW ultrasonic sensor");
    ESP_LOGI(TAG, "  UART%d, TX=GPIO%d (to sensor RX), RX=GPIO%d (from sensor TX)",
             config->uart_num, config->tx_gpio, config->rx_gpio);
    ESP_LOGI(TAG, "  Baud rate: %d", config->baud_rate);

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

    err = uart_param_config(config->uart_num, &uart_config);
    if (err != ESP_OK) {
        uart_driver_delete(config->uart_num);
        return err;
    }

    err = uart_set_pin(config->uart_num,
                       config->tx_gpio,
                       config->rx_gpio,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        uart_driver_delete(config->uart_num);
        return err;
    }

    // Copy config internally so background task has a stable pointer
    s_config = *config;

    // Start background task to continuously read sensor (uses internal config copy)
    xTaskCreate(ultrasonic_a02yyuw_uart_rx_task, "ultrasonic_a02yyuw_rx", RX_TASK_STACK, (void *)&s_config, RX_TASK_PRIO, nullptr);
    
    ESP_LOGI(TAG, "Initialization complete - waiting for sensor data");
    return ESP_OK;
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
