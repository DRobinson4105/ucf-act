/**
 * @file relay_jd2912.hh
 * @brief JD-2912 automotive relay driver (via IRLZ44N MOSFET).
 *
 * Controls the pedal bypass relay that allows the Curtis motor controller
 * to accept autonomous throttle input without the accelerator pedal
 * microswitch being physically pressed.
 */
#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Pedal Bypass Relay Driver (JD-2912 via IRLZ44N)
// ============================================================================
// Controls a relay that bypasses the accelerator pedal microswitch.
//
// Hardware:
//   - ESP32 GPIO drives an IRLZ44N logic-level N-channel MOSFET gate
//   - MOSFET switches the JD-2912 automotive relay coil (12V)
//   - 10k pull-down on gate ensures safe default (relay off at boot/reset)
//   - Golf cart throttle requires pedal microswitch to be closed before
//     motor controller accepts throttle input
//   - This relay bypasses the microswitch during autonomous operation
//   - De-energized (NC): Normal pedal operation - microswitch must be closed
//   - Energized (NO): Microswitch bypassed - throttle always enabled
//
// Safety:
//   - Default state is de-energized (manual pedal control)
//   - Should only be energized when autonomous mode is active
//   - Must de-energize on any fault or override condition
// ============================================================================

// ============================================================================
// Configuration
// ============================================================================

// relay_jd2912_config_t - relay control pin configuration
//   gpio:        GPIO pin controlling MOSFET gate (drives relay coil)
//   active_high: true = HIGH energizes relay, false = LOW energizes relay
typedef struct {
    gpio_num_t gpio;
    bool active_high;
} relay_jd2912_config_t;

// ============================================================================
// Initialization
// ============================================================================

// Initialize GPIO and set relay to de-energized state (safe default)
esp_err_t relay_jd2912_init(const relay_jd2912_config_t *config);

// ============================================================================
// Relay Control
// ============================================================================

// Energize relay - bypasses pedal microswitch (for autonomous mode)
esp_err_t relay_jd2912_energize(void);

// De-energize relay - restores normal pedal operation (safe state)
esp_err_t relay_jd2912_deenergize(void);

// Check if relay is currently energized
bool relay_jd2912_is_energized(void);

#ifdef __cplusplus
}
#endif
