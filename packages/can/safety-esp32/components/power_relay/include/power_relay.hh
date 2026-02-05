#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// Power Relay Driver
// =============================================================================
// Controls a relay that enables/disables power to the vehicle motor controller.
//
// Hardware:
//   - High-current relay in series with motor controller power supply
//   - Energized: Power flows to motor controller (vehicle can move)
//   - De-energized: Power cut to motor controller (vehicle stopped)
//
// Safety:
//   - Default state is OFF (de-energized) - vehicle cannot move
//   - Only enabled when all safety conditions are met
//   - Any e-stop condition immediately de-energizes relay
//   - Fail-safe design: relay coil failure = power cut
// =============================================================================

// =============================================================================
// Configuration
// =============================================================================

// power_relay_config_t - relay control pin configuration
//   gpio:            GPIO pin controlling relay coil
//   active_high:     true = HIGH enables relay, false = LOW enables relay
//   enable_pullup:   true to enable internal pull-up on GPIO
//   enable_pulldown: true to enable internal pull-down on GPIO
typedef struct {
    gpio_num_t gpio;
    bool active_high;
    bool enable_pullup;
    bool enable_pulldown;
} power_relay_config_t;

// =============================================================================
// Initialization
// =============================================================================

// Initialize GPIO and set relay to OFF state (safe default)
esp_err_t power_relay_init(const power_relay_config_t *config);

// =============================================================================
// Relay Control
// =============================================================================

// Enable power relay - closes circuit, allows power to motor controller
// Only call when all safety conditions are satisfied
esp_err_t power_relay_enable(const power_relay_config_t *config);

// Disable power relay - opens circuit, cuts power (safe state)
// Call immediately on any e-stop or fault condition
esp_err_t power_relay_disable(const power_relay_config_t *config);

// Check if relay is currently enabled (power flowing)
bool power_relay_is_enabled(const power_relay_config_t *config);

#ifdef __cplusplus
}
#endif
