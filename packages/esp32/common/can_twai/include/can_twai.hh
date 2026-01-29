#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "driver/twai.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// initialize TWAI with 1 mbps timing and accept all filter
esp_err_t can_twai_init_default(gpio_num_t tx_gpio, gpio_num_t rx_gpio);

// transmit a standard 11-bit CAN frame with 8-byte payload
esp_err_t can_twai_send(uint32_t identifier, const uint8_t data[8], TickType_t timeout);

// receive a CAN frame with timeout; returns ESP_OK if a frame is received
esp_err_t can_twai_receive(twai_message_t *msg, TickType_t timeout);

#ifdef __cplusplus
}
#endif
