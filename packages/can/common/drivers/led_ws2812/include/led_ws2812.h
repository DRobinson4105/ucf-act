/**
 * @file led_ws2812.h
 * @brief Raw WS2812 LED transport driver.
 */
#pragma once

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// WS2812 LED Transport Driver
// ============================================================================
// Raw RGB transport for the onboard WS2812 LED (GPIO8 on ESP32-C6).
// Higher-level state/fault policy belongs in status_led.
// ============================================================================

typedef struct
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
} led_ws2812_rgb_t;

/**
 * @brief Initialize the WS2812 LED on GPIO8 via the RMT peripheral.
 *
 * Configures the ESP32 RMT peripheral to drive a single WS2812 RGB LED
 * on GPIO8. Must be called before led_ws2812_set_rgb().
 *
 * @return ESP_OK on success, or an error code if RMT setup fails
 */
esp_err_t led_ws2812_init(void);

/**
 * @brief Transmit an RGB color immediately to the WS2812 LED.
 *
 * @param color  RGB color to display
 */
void led_ws2812_set_rgb(led_ws2812_rgb_t color);

#ifdef __cplusplus
}
#endif
