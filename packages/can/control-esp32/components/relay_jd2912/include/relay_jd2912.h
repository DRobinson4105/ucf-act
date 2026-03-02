/**
 * @file relay_jd2912.h
 * @brief JD-2912 automotive relay driver (via S8050 NPN transistor).
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
extern "C"
{
#endif

// ============================================================================
// Pedal Bypass Relay Driver (JD-2912 via S8050)
// ============================================================================
// Controls a relay that bypasses the accelerator pedal microswitch.
//
// Hardware:
//   - ESP32 GPIO drives an S8050 NPN transistor base via 680R current limiter
//   - Transistor switches the JD-2912 automotive relay coil (12V)
//   - 10k pull-down on base ensures safe default (relay off at boot/reset)
//   - 1N4007 flyback diode across coil protects transistor from back-EMF
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
//   gpio: GPIO pin controlling transistor base (via 680R, drives relay coil)
//
// Polarity is hardcoded active-high (HIGH = base current = transistor on =
// coil energized) to match the S8050 NPN circuit. External 10k pull-down on
// the base keeps the relay safely de-energized during boot/reset.
typedef struct
{
	gpio_num_t gpio;
} relay_jd2912_config_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize GPIO and set the relay to de-energized state (safe default).
 *
 * Configures the transistor-base GPIO as a push-pull output and drives it
 * LOW to ensure the relay starts de-energized (normal pedal operation).
 *
 * @param config  Pointer to relay configuration (GPIO pin)
 * @return ESP_OK on success, or an error code if GPIO configuration fails
 */
esp_err_t relay_jd2912_init(const relay_jd2912_config_t *config);

// ============================================================================
// Relay Control
// ============================================================================

/**
 * @brief Energize the relay, bypassing the pedal microswitch.
 *
 * Drives the transistor base HIGH, energizing the JD-2912 relay coil.
 * This closes the NO contact and allows the Curtis motor controller
 * to accept autonomous throttle input without the pedal being pressed.
 *
 * @return ESP_OK on success, or an error code if GPIO write fails
 */
esp_err_t relay_jd2912_energize(void);

/**
 * @brief De-energize the relay, restoring normal pedal operation.
 *
 * Drives the transistor base LOW, de-energizing the relay coil.
 * The NC contact returns the microswitch to the circuit, requiring
 * physical pedal press for throttle input (safe state).
 *
 * @return ESP_OK on success, or an error code if GPIO write fails
 */
esp_err_t relay_jd2912_deenergize(void);

/**
 * @brief Check if the relay is currently energized.
 *
 * Reads the GPIO output level to determine whether the relay coil
 * is energized (pedal microswitch bypassed).
 *
 * @return true if the relay is energized, false otherwise
 */
bool relay_jd2912_is_energized(void);

#ifdef __cplusplus
}
#endif
