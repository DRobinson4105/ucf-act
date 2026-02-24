/**
 * @file multiplexer_dg408djz.hh
 * @brief DG408DJZ 8-channel analog multiplexer for throttle level selection.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// DG408DJZ Multiplexer Driver
// ============================================================================
// Controls throttle output level using a DG408DJZ 8-channel analog
// multiplexer. The throttle source relay (AEDIKO SRD-05VDC) that switches
// between manual pedal and mux output is managed separately via relay_srd05vdc.
//
// Hardware:
//   - 8:1 analog mux selects one of 8 voltage divider taps (throttle levels 0-7)
//   - Three address lines (A0-A2) select the active channel
//   - Enable line (EN) gates the mux output (LOW = disabled)
//
// Safe state: EN LOW (mux disabled)
// ============================================================================

// ============================================================================
// Configuration
// ============================================================================

// multiplexer_dg408djz_config_t - GPIO pin assignments
//   a0:    Address bit 0 (LSB) - selects mux channel bit 0
//   a1:    Address bit 1 - selects mux channel bit 1
//   a2:    Address bit 2 (MSB) - selects mux channel bit 2
//   en:    Enable pin - HIGH enables mux output, LOW disables (safe state)
typedef struct {
    gpio_num_t a0;
    gpio_num_t a1;
    gpio_num_t a2;
    gpio_num_t en;
} multiplexer_dg408djz_config_t;

// ============================================================================
// Initialization
// ============================================================================

// Initialize GPIO pins and set safe state (EN LOW, address lines zeroed)
esp_err_t multiplexer_dg408djz_init(const multiplexer_dg408djz_config_t *config);

// ============================================================================
// Level Control
// ============================================================================

// Set throttle level (0-7), or -1 to disable mux output
// Level 0 = minimum throttle, Level 7 = maximum throttle
esp_err_t multiplexer_dg408djz_set_level(int8_t level);

// Disable mux output (EN LOW) without affecting relay state
esp_err_t multiplexer_dg408djz_disable(void);

// Get current throttle level
// Returns -1 if disabled, 0-7 if active
int8_t multiplexer_dg408djz_get_level(void);

// ============================================================================
// Autonomous Mode Control
// ============================================================================

// Enable autonomous mode: sets mux to level 0 and marks active.
// Caller must energize throttle relay (relay_srd05vdc) separately.
esp_err_t multiplexer_dg408djz_enable_autonomous(void);

// Emergency stop: immediate mux disable, clears autonomous flag.
// Caller must de-energize throttle relay separately.
esp_err_t multiplexer_dg408djz_emergency_stop(void);

// Check if currently in autonomous mode
bool multiplexer_dg408djz_is_autonomous(void);

#ifdef __cplusplus
}
#endif
