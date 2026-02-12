/**
 * @file heartbeat.hh
 * @brief WS2812 LED heartbeat indicator with activity and error states.
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
// WS2812 Heartbeat LED Driver
// ============================================================================
// Visual heartbeat indicator using onboard WS2812 RGB LED (GPIO8 on ESP32-C6).
// Blinks at interval_ticks rate with color indicating state:
//   - Idle (green): Normal operation, no recent CAN activity
//   - Activity (blue): Recent CAN traffic within activity_window
//   - Error (red): Fault condition active
// ============================================================================

// ============================================================================
// Configuration
// ============================================================================

typedef struct {
    gpio_num_t gpio;                    // WS2812 data pin (GPIO8 on ESP32-C6 dev board)
    TickType_t interval_ticks;          // Blink interval
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
} heartbeat_config_t;

// ============================================================================
// Initialization
// ============================================================================

// Initialize WS2812 LED via RMT peripheral
esp_err_t heartbeat_init(const heartbeat_config_t *config);

// ============================================================================
// Runtime Control
// ============================================================================

// Update heartbeat state - call from main loop at regular intervals
// Handles blink timing and state transitions
void heartbeat_tick(const heartbeat_config_t *config, TickType_t now_ticks);

// Mark recent activity (e.g., CAN frame received/sent)
// Switches to activity state for activity_window_ticks duration
void heartbeat_mark_activity(TickType_t now_ticks);

// Set or clear error state
// Error state takes priority over idle/activity states
void heartbeat_set_error(bool active);

// Enable/disable manual LED mode.
// When enabled, heartbeat_set_manual_color() controls LED color/reason and
// overrides idle/activity/error color selection.
void heartbeat_set_manual_mode(bool enabled);

// Set manual LED color and reason string.
// Reason is copied internally and may be a temporary string.
void heartbeat_set_manual_color(uint8_t red,
                                uint8_t green,
                                uint8_t blue,
                                const char *reason);

#ifdef __cplusplus
}
#endif
