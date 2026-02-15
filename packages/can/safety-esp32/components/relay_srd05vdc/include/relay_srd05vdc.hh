/**
 * @file relay_srd05vdc.hh
 * @brief GPIO-driven SRD-05VDC-SL-C relay for actuator safety cutoff.
 */
#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// SRD-05VDC-SL-C Relay Driver
// ============================================================================
// Controls a 5V relay module that enables/disables the 24V autonomous power
// rail feeding the stepper motor drivers.
//
// Hardware:
//   - AEDIKO 1-channel 5V relay module (SRD-05VDC-SL-C)
//   - Optocoupler-isolated trigger input, 5V coil
//   - Load connected through NO (normally open) terminal for fail-safe
//   - Energized: NO closes, 24V flows to motor drivers (vehicle can move)
//   - De-energized: NO opens, 24V cut to motor drivers (vehicle stopped)
//
// Safety:
//   - Default state is OFF (de-energized) - vehicle cannot move
//   - Only enabled when all safety conditions are met
//   - Any e-stop condition immediately de-energizes relay
//   - Fail-safe design: coil/module power loss = power cut
// ============================================================================

// ============================================================================
// Configuration
// ============================================================================

// relay_srd05vdc_config_t - relay control pin configuration
//   gpio:            GPIO pin controlling relay trigger (IN pin)
//   active_high:     true = HIGH energizes relay, false = LOW energizes relay
//   enable_pullup:   true to enable internal pull-up on GPIO
//   enable_pulldown: true to enable internal pull-down on GPIO
typedef struct {
    gpio_num_t gpio;
    bool active_high;
    bool enable_pullup;
    bool enable_pulldown;
} relay_srd05vdc_config_t;

// ============================================================================
// Initialization
// ============================================================================

// Initialize GPIO and set relay to OFF state (safe default)
esp_err_t relay_srd05vdc_init(const relay_srd05vdc_config_t *config);

// ============================================================================
// Relay Control
// ============================================================================

// Enable relay - energizes coil, closes NO contact, allows 24V to motors
// Only call when all safety conditions are satisfied
esp_err_t relay_srd05vdc_enable(const relay_srd05vdc_config_t *config);

// Disable relay - de-energizes coil, opens NO contact, cuts 24V (safe state)
// Call immediately on any e-stop or fault condition
esp_err_t relay_srd05vdc_disable(const relay_srd05vdc_config_t *config);

// Check if relay is currently enabled (coil energized, 24V flowing)
bool relay_srd05vdc_is_enabled(const relay_srd05vdc_config_t *config);

#ifdef __cplusplus
}
#endif
