/**
 * @file relay_srd05vdc.hh
 * @brief GPIO-driven AEDIKO SRD-05VDC-SL-C relay module driver.
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
// Generic driver for AEDIKO 1-channel 5V relay modules (SRD-05VDC-SL-C).
// Supports multiple independent instances — each identified by its config.
//
// Hardware:
//   - AEDIKO 1-channel 5V relay module (SRD-05VDC-SL-C)
//   - Optocoupler-isolated trigger input, 5V coil
//   - Load connected through NO (normally open) terminal for fail-safe
//   - Energized: NO closes (load connected)
//   - De-energized: NO opens (load disconnected)
//
// Usage:
//   - Safety ESP32: power relay (24V autonomous rail to stepper motors)
//   - Control ESP32: throttle relay (switches Curtis throttle source)
//
// Safety:
//   - Default state is OFF (de-energized)
//   - Fail-safe design: coil/module power loss = relay off
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

// Enable relay - energizes coil, closes NO contact
esp_err_t relay_srd05vdc_enable(const relay_srd05vdc_config_t *config);

// Disable relay - de-energizes coil, opens NO contact (safe state)
esp_err_t relay_srd05vdc_disable(const relay_srd05vdc_config_t *config);

// Check if relay is currently enabled (reads GPIO state directly)
bool relay_srd05vdc_is_enabled(const relay_srd05vdc_config_t *config);

#ifdef __cplusplus
}
#endif
