#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"

#include "ultrasonic.hh"

namespace {
// logging tag
static const char *kTag = "ULTRASONIC";

// task and buffer configuration
constexpr int kRxTaskStack = 4096;
constexpr UBaseType_t kRxTaskPrio = 10;
constexpr int kUartBufferSize = 256;
constexpr TickType_t kUartReadTimeout = pdMS_TO_TICKS(100);
constexpr TickType_t kMaxSampleAge = pdMS_TO_TICKS(200);

static portMUX_TYPE s_ultrasonic_lock = portMUX_INITIALIZER_UNLOCKED;
static uint16_t s_last_distance_mm = 0;
static TickType_t s_last_update_tick = 0;
static bool s_has_measurement = false;

static bool ultrasonic_parse_stream(const uint8_t *data, int len, uint16_t *distance_mm) {
    static uint8_t frame[4];
    static int idx = 0;

    for (int i = 0; i < len; ++i) {
        uint8_t b = data[i];
        if (idx == 0) {
            if (b != 0xFF) continue;
            frame[idx++] = b;
            continue;
        }

        frame[idx++] = b;
        if (idx == 4) {
            idx = 0;
            uint8_t sum = (uint8_t)(frame[0] + frame[1] + frame[2]);
            if (sum == frame[3]) {
                *distance_mm = (uint16_t)((frame[1] << 8) | frame[2]);
                return true;
            }
        }
    }

    return false;
}

static void ultrasonic_update_distance(uint16_t distance_mm) {
    taskENTER_CRITICAL(&s_ultrasonic_lock);
    s_last_distance_mm = distance_mm;
    s_last_update_tick = xTaskGetTickCount();
    s_has_measurement = true;
    taskEXIT_CRITICAL(&s_ultrasonic_lock);
}

static void ultrasonic_uart_rx_task(void *arg) {
    const ultrasonic_config_t *config = static_cast<const ultrasonic_config_t *>(arg);
    uint8_t buffer[kUartBufferSize];
    uint16_t distance_mm = 0;

    while (true) {
        int read_len = uart_read_bytes(config->uart_num, buffer, sizeof(buffer), kUartReadTimeout);
        if (read_len > 0 && ultrasonic_parse_stream(buffer, read_len, &distance_mm)) {
            ultrasonic_update_distance(distance_mm);
            ESP_LOGD(kTag, "distance %u mm", distance_mm);
        }
    }
}
}

esp_err_t ultrasonic_init(const ultrasonic_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

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

    esp_err_t err = uart_driver_install(config->uart_num, kUartBufferSize, kUartBufferSize, 0, nullptr, 0);
    if (err != ESP_OK) return err;

    err = uart_param_config(config->uart_num, &uart_config);
    if (err != ESP_OK) return err;

    err = uart_set_pin(config->uart_num,
                       config->tx_gpio,
                       config->rx_gpio,
                       UART_PIN_NO_CHANGE,
                       UART_PIN_NO_CHANGE);
    if (err != ESP_OK) return err;

    xTaskCreate(ultrasonic_uart_rx_task, "ultrasonic_uart_rx", kRxTaskStack, (void *)config, kRxTaskPrio, nullptr);
    return ESP_OK;
}

bool ultrasonic_get_distance_mm(uint16_t *out_distance_mm) {
    if (!out_distance_mm) return false;


    TickType_t now = xTaskGetTickCount();
    taskENTER_CRITICAL(&s_ultrasonic_lock);
    bool valid = s_has_measurement && (now - s_last_update_tick <= kMaxSampleAge);
    uint16_t distance_mm = s_last_distance_mm;
    taskEXIT_CRITICAL(&s_ultrasonic_lock);

    if (!valid) return false;

    *out_distance_mm = distance_mm;
    return true;
}

bool ultrasonic_is_too_close(uint16_t threshold_mm, uint16_t *out_distance_mm) {
    uint16_t distance_mm = 0;
    if (!ultrasonic_get_distance_mm(&distance_mm)) return false;

    if (out_distance_mm) *out_distance_mm = distance_mm;

    return distance_mm <= threshold_mm;
}