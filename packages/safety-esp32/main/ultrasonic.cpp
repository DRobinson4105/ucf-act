#include <stdint.h>
#include <stdbool.h>

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "ultrasonic.h"

#define ULTRASONIC_UART UART_NUM_1
#define ULTRASONIC_TX_GPIO UART_PIN_NO_CHANGE
#define ULTRASONIC_RX_GPIO 6
#define ULTRASONIC_BAUD 9600

static bool ultrasonic_parse_stream(const uint8_t* data, int len, uint16_t* dist) {
    static uint8_t frame[4];
    static int idx = 0;
    
    for (int i = 0; i < len; i++) {
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
                *dist = (uint16_t)((frame[1] << 8) | frame[2]);
                return true;
            }
        }
    }

    return false;
}

static void ultrasonic_uart_rx_task(void* arg) {
    uint8_t buf[256];
    uint16_t dist;

    while (true) {
        int n = uart_read_bytes(ULTRASONIC_UART, buf, sizeof(buf), pdMS_TO_TICKS(100));

        if (n > 0 && ultrasonic_parse_stream(buf, n, &dist)) {
            printf("distance: %u mm\n", dist);
        }
    }
}

void ultrasonic_init(void) {
    const int uart_buffer_size = 256;
    QueueHandle_t uart_queue;

    ESP_ERROR_CHECK(uart_driver_install(
        ULTRASONIC_UART,
        uart_buffer_size,
        uart_buffer_size,
        10,
        &uart_queue,
        0
    ));

    uart_config_t uart_config = {
        .baud_rate = ULTRASONIC_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
        .flags = {0, 0},
    };

    ESP_ERROR_CHECK(uart_param_config(ULTRASONIC_UART, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(
        ULTRASONIC_UART,
        ULTRASONIC_TX_GPIO,
        ULTRASONIC_RX_GPIO,
        UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE
    ));

    xTaskCreatePinnedToCore(
        ultrasonic_uart_rx_task,
        "ultrasonic_uart_rx",
        4096,
        nullptr,
        10,
        nullptr,
        0
    );
}
