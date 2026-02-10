/**
 * @file enable_relay.hh
 * @brief MOSFET-driven relay control for autonomous actuator enable circuit.
 */
#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Pedal Enable Relay Driver
// ============================================================================
// Controls a relay that bypasses the accelerator pedal microswitch.
//
// Hardware:
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

// enable_relay_config_t - relay control pin configuration
//   gpio:        GPIO pin controlling relay coil
//   active_high: true = HIGH energizes relay, false = LOW energizes relay
typedef struct {
    gpio_num_t gpio;
    bool active_high;
} enable_relay_config_t;

// ============================================================================
// Initialization
// ============================================================================

// Initialize GPIO and set relay to de-energized state (safe default)
esp_err_t enable_relay_init(const enable_relay_config_t *config);

// ============================================================================
// Relay Control
// ============================================================================

// Energize relay - bypasses pedal microswitch (for autonomous mode)
void enable_relay_energize(void);

// De-energize relay - restores normal pedal operation (safe state)
void enable_relay_deenergize(void);

// Check if relay is currently energized
bool enable_relay_is_energized(void);

#ifdef __cplusplus
}
#endif
