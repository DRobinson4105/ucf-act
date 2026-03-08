/**
 * @file led_ws2812.h
 * @brief WS2812 status LED driver with node-state color mapping.
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "can_protocol.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// WS2812 Status LED Driver
// ============================================================================
// Visual status indicator using onboard WS2812 RGB LED (GPIO8 on ESP32-C6).
// Color is selected by node state and transmitted via the ESP32 RMT peripheral.
//
// Color mapping:
//   - Green:  READY
//   - Blue:   ACTIVE
//   - Yellow: INIT, NOT_READY, ENABLE, OVERRIDE
//   - Red:    FAULT
// ============================================================================

/**
 * @brief Initialize the WS2812 LED on GPIO8 via the RMT peripheral.
 *
 * Configures the ESP32 RMT peripheral to drive a single WS2812 RGB LED
 * on GPIO8. Must be called before led_ws2812_set_state().
 *
 * @return ESP_OK on success, or an error code if RMT setup fails
 */
esp_err_t led_ws2812_init(void);

/**
 * @brief Set the current node state and update the LED color.
 *
 * Maps the node state to an LED color (e.g., green = READY, red = FAULT)
 * and transmits it immediately when the state changes. Periodic
 * re-transmissions are rate-limited internally.
 *
 * @param node_state  Current node state to reflect on the LED
 */
void led_ws2812_set_state(node_state_t node_state);

#ifdef __cplusplus
}
#endif
