#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"

static const char *TAG = "SERIAL_RX";

#define UART_NUM        UART_NUM_0
#define BUF_SIZE        256
#define BAUD_RATE       1000000

extern "C" void app_main(void) {
    uart_config_t uart_config = {
        .baud_rate  = BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "Serial receiver started at %d baud. Waiting for messages...", BAUD_RATE);

    uint8_t buf[BUF_SIZE];
    while (true) {
        int len = uart_read_bytes(UART_NUM, buf, sizeof(buf) - 1, pdMS_TO_TICKS(10));
        if (len > 0) {
            buf[len] = '\0';
            ESP_LOGI(TAG, "Received %d bytes: %s", len, (char *)buf);
            printf("  Hex:");
            for (int i = 0; i < len; i++) {
                printf(" %02X", buf[i]);
            }
            printf("\n");
        } else {
            ESP_LOGW(TAG, "No data received (timeout)");
        }
    }
}
