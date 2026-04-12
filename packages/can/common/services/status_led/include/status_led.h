/**
 * @file status_led.h
 * @brief State-driven status LED adapter above the raw WS2812 transport.
 */
#pragma once

#include "can_protocol.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Initialize the status LED service and underlying WS2812 transport.
 *
 * @return ESP_OK on success, or an error code from the WS2812 driver
 */
esp_err_t status_led_init(void);

/**
 * @brief Update the node state reflected by the status LED.
 *
 * @param state  Node state to display
 */
void status_led_set_state(node_state_t state);

#ifdef __cplusplus
}
#endif
