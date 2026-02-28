/**
 * @file led_ws2812.hh
 * @brief WS2812 status LED driver with activity and error states.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// WS2812 Status LED Driver
// ============================================================================
// Visual status indicator using onboard WS2812 RGB LED (GPIO8 on ESP32-C6).
// Color is selected by software state logic and transmitted via the ESP32 RMT
// peripheral — the GPIO serves only as the RMT TX data output and does not
// read or determine LED state.  There is no input/reference GPIO; the WS2812
// is a one-wire output-only protocol with no feedback line.
//
// Displays a solid color indicating state:
//   - Idle (green): Normal operation, no recent CAN activity
//   - Activity (blue): Recent CAN traffic within activity_window
//   - Error (red): Fault condition active
// Color is updated every interval_ticks.
// ============================================================================

// ============================================================================
// Configuration
// ============================================================================

typedef struct {
    gpio_num_t gpio;                    // RMT TX output pin for WS2812 data line (GPIO8 on ESP32-C6 dev board)
    TickType_t interval_ticks;          // Color update interval
    TickType_t activity_window_ticks;   // Duration to show activity after mark_activity()
    const char *label;                  // Debug label for logging
    uint8_t idle_red;                   // Idle state color (0-255 each)
    uint8_t idle_green;
    uint8_t idle_blue;
    uint8_t activity_red;               // Activity state color
    uint8_t activity_green;
    uint8_t activity_blue;
    uint8_t error_red;                  // Error state color
    uint8_t error_green;
    uint8_t error_blue;
} led_ws2812_config_t;

// ============================================================================
// Initialization
// ============================================================================

// Initialize WS2812 LED via RMT peripheral
esp_err_t led_ws2812_init(const led_ws2812_config_t *config);

// ============================================================================
// Runtime Control
// ============================================================================

// Update LED state - call from main loop at regular intervals
// Handles color update timing and state transitions
void led_ws2812_tick(const led_ws2812_config_t *config, TickType_t now_ticks);

// Mark recent activity (e.g., CAN frame received/sent)
// Switches to activity state for activity_window_ticks duration
void led_ws2812_mark_activity(TickType_t now_ticks);

// Set or clear error state
// Error state takes priority over idle/activity states
void led_ws2812_set_error(bool active);

// Enable/disable manual LED mode.
// When enabled, led_ws2812_set_manual_color() controls LED color/reason and
// overrides idle/activity/error color selection.
void led_ws2812_set_manual_mode(bool enabled);

// Set manual LED color and reason string.
// Reason is copied internally and may be a temporary string.
void led_ws2812_set_manual_color(uint8_t red,
                                 uint8_t green,
                                 uint8_t blue,
                                 const char *reason);

#ifdef __cplusplus
}
#endif
