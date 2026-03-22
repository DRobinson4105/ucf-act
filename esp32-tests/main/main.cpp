#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/twai.h"
#include "esp_log.h"

static const char *TAG = "CAN_RX";

#define CAN_TX_GPIO 4
#define CAN_RX_GPIO 5

extern "C" void app_main(void) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        (gpio_num_t)CAN_TX_GPIO,
        (gpio_num_t)CAN_RX_GPIO,
        TWAI_MODE_LISTEN_ONLY
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());
    ESP_LOGI(TAG, "CAN receiver started. Waiting for messages...");

    twai_message_t msg;
    while (true) {
        if (twai_receive(&msg, pdMS_TO_TICKS(1000)) == ESP_OK) {
            if (msg.extd) {
                ESP_LOGI(TAG, "ID: 0x%08lX [EXT] DLC: %d", msg.identifier, msg.data_length_code);
            } else {
                ESP_LOGI(TAG, "ID: 0x%03lX [STD] DLC: %d", msg.identifier, msg.data_length_code);
            }
            if (!msg.rtr) {
                printf("  Data:");
                for (int i = 0; i < msg.data_length_code; i++) {
                    printf(" %02X", msg.data[i]);
                }
                printf("\n");
            }
        } else {
            ESP_LOGW(TAG, "No message received (timeout)");
        }
    }
}