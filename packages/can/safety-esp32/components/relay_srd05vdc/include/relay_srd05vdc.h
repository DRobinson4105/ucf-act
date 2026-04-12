/**
 * @file relay_srd05vdc.h
 * @brief GPIO-driven AEDIKO SRD-05VDC-SL-C relay module driver.
 */
#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
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
//   - Safety ESP32: power relay (24V autonomous rail to UIM2852 motors)
//
// Safety:
//   - Default state is OFF (de-energized)
//   - Fail-safe design: coil/module power loss = relay off
// ============================================================================

// ============================================================================
// Configuration
// ============================================================================

// relay_srd05vdc_config_t - relay control pin configuration
//   gpio: GPIO pin controlling relay trigger (IN pin)
//
// Polarity is hardcoded active-high (HIGH energizes relay) to match the
// AEDIKO module hardware. The output latch is preloaded LOW before GPIO
// configuration to keep the relay de-energized during boot/reset.
typedef struct
{
	gpio_num_t gpio;
} relay_srd05vdc_config_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize GPIO and set relay to OFF state (safe default).
 *
 * Configures the relay control pin as a push-pull output with internal
 * pull-down and drives it LOW to ensure the relay starts de-energized.
 *
 * @param config  Pointer to relay configuration (GPIO pin)
 * @return ESP_OK on success, or an error code if GPIO configuration fails
 */
esp_err_t relay_srd05vdc_init(const relay_srd05vdc_config_t *config);

// ============================================================================
// Relay Control
// ============================================================================

/**
 * @brief Energize the relay coil, closing the NO contact.
 *
 * Drives the control GPIO HIGH to energize the relay coil, which closes
 * the normally-open contact and connects the load.
 *
 * @param config  Pointer to relay configuration (GPIO pin)
 * @return ESP_OK on success, or an error code if GPIO write fails
 */
esp_err_t relay_srd05vdc_enable(const relay_srd05vdc_config_t *config);

/**
 * @brief De-energize the relay coil, opening the NO contact (safe state).
 *
 * Drives the control GPIO LOW to de-energize the relay coil, which opens
 * the normally-open contact and disconnects the load.
 *
 * @param config  Pointer to relay configuration (GPIO pin)
 * @return ESP_OK on success, or an error code if GPIO write fails
 */
esp_err_t relay_srd05vdc_disable(const relay_srd05vdc_config_t *config);

/**
 * @brief Check if the relay is currently enabled.
 *
 * Reads the GPIO output level directly to determine relay state.
 *
 * @param config  Pointer to relay configuration (GPIO pin)
 * @return true if the relay is energized (coil ON), false otherwise
 */
bool relay_srd05vdc_is_enabled(const relay_srd05vdc_config_t *config);

#ifdef __cplusplus
}
#endif
