/**
 * @file throttle_mux.hh
 * @brief DG408 analog multiplexer throttle control with relay-based mode switching.
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
// Throttle Multiplexer Driver
// ============================================================================
// Controls throttle output using an 8-channel analog multiplexer (CD4051/74HC4051)
// and a DPDT relay for autonomous/manual mode switching.
//
// Hardware:
//   - 8:1 analog mux selects one of 8 voltage divider taps (throttle levels 0-7)
//   - Three address lines (A0-A2) select the active channel
//   - Enable line (EN) gates the mux output (LOW = disabled)
//   - DPDT relay switches between:
//       - NC (de-energized): Manual throttle from pedal
//       - NO (energized): Autonomous throttle from mux
//
// Safe state: EN LOW (mux disabled), relay de-energized (manual mode)
// ============================================================================

// ============================================================================
// Configuration
// ============================================================================

// throttle_mux_config_t - GPIO pin assignments
//   a0:    Address bit 0 (LSB) - selects mux channel bit 0
//   a1:    Address bit 1 - selects mux channel bit 1
//   a2:    Address bit 2 (MSB) - selects mux channel bit 2
//   en:    Enable pin - HIGH enables mux output, LOW disables (safe state)
//   relay: Relay control - HIGH energizes (autonomous), LOW de-energizes (manual)
typedef struct {
    gpio_num_t a0;
    gpio_num_t a1;
    gpio_num_t a2;
    gpio_num_t en;
    gpio_num_t relay;
} throttle_mux_config_t;

// ============================================================================
// Initialization
// ============================================================================

// Initialize GPIO pins and set safe state (EN LOW, relay de-energized)
esp_err_t throttle_mux_init(const throttle_mux_config_t *config);

// ============================================================================
// Level Control
// ============================================================================

// Set throttle level (0-7), or -1 to disable mux output
// Level 0 = minimum throttle, Level 7 = maximum throttle
void throttle_mux_set_level(int8_t level);

// Disable mux output (EN LOW) without affecting relay state
void throttle_mux_disable(void);

// Get current throttle level
// Returns -1 if disabled, 0-7 if active
int8_t throttle_mux_get_level(void);

// ============================================================================
// Autonomous Mode Control
// ============================================================================

// Enable autonomous mode: sets level to 0, then energizes relay
// Call this when entering autonomous control state
void throttle_mux_enable_autonomous(void);

// Disable autonomous mode: ramps down throttle, de-energizes relay, disables mux
// Provides smooth transition back to manual control
void throttle_mux_disable_autonomous(void);

// Emergency stop: immediate mux disable + relay de-energize
// Use for fault conditions requiring instant throttle cutoff
void throttle_mux_emergency_stop(void);

// Check if currently in autonomous mode (relay energized)
bool throttle_mux_is_autonomous(void);

#ifdef __cplusplus
}
#endif
