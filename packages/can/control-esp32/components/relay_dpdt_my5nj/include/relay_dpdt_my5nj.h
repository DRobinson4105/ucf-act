/**
 * @file relay_dpdt_my5nj.h
 * @brief MY5NJ DPDT relay driver (24V coil via 2N5551 NPN transistor).
 *
 * Replaces both the SRD-05VDC throttle relay and the JD-2912 pedal bypass
 * relay with a single DPDT relay. Both poles switch together:
 *   - Pole 1: switches Curtis throttle input between manual pedal (NC)
 *             and digipot wiper (NO)
 *   - Pole 2: bypasses pedal microswitch (NC = normal operation,
 *             NO = bypassed)
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
// DPDT Relay Driver (MY5NJ via 2N5551)
// ============================================================================
// Controls a DPDT relay that simultaneously switches the throttle source
// and bypasses the pedal microswitch.
//
// Hardware:
//   - ESP32 GPIO drives an 2N5551 NPN transistor base via 680R current limiter
//   - Transistor switches the MY5NJ DPDT relay coil (24V)
//   - 10k pull-down on base ensures safe default (relay off at boot/reset)
//   - 1N4007 flyback diode across coil protects transistor from back-EMF
//   - Pole 1 (throttle source):
//       NC: manual pedal drives Curtis throttle input
//       NO: digipot wiper drives Curtis throttle input
//   - Pole 2 (pedal microswitch):
//       NC: normal pedal operation - microswitch must be closed
//       NO: microswitch bypassed - throttle always enabled
//   - Both poles switch together (DPDT)
//   - De-energized (NC): safe state - manual pedal control, no bypass
//   - Energized (NO): autonomous mode - digipot throttle, pedal bypassed
//
// Safety:
//   - Default state is de-energized (manual pedal control)
//   - Should only be energized when autonomous mode is active
//   - Must de-energize on any fault or override condition
// ============================================================================

// ============================================================================
// Configuration
// ============================================================================

// relay_dpdt_my5nj_config_t - relay control pin configuration
//   gpio: GPIO pin controlling transistor base (via 680R, drives relay coil)
//
// Polarity is hardcoded active-high (HIGH = base current = transistor on =
// coil energized) to match the 2N5551 NPN circuit. External 10k pull-down on
// the base keeps the relay safely de-energized during boot/reset.
typedef struct
{
	gpio_num_t gpio;
} relay_dpdt_my5nj_config_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize GPIO and set the relay to de-energized state (safe default).
 *
 * Configures the transistor-base GPIO as a push-pull output and drives it
 * LOW to ensure the relay starts de-energized (manual pedal control,
 * no microswitch bypass).
 *
 * @param config  Pointer to relay configuration (GPIO pin)
 * @return ESP_OK on success, or an error code if GPIO configuration fails
 */
esp_err_t relay_dpdt_my5nj_init(const relay_dpdt_my5nj_config_t *config);

// ============================================================================
// Relay Control
// ============================================================================

/**
 * @brief Energize the relay, switching to autonomous throttle and bypassing
 *        the pedal microswitch.
 *
 * Drives the transistor base HIGH, energizing the MY5NJ relay coil.
 * Pole 1 switches throttle source from manual pedal (NC) to digipot
 * wiper (NO). Pole 2 bypasses the pedal microswitch (NO), allowing
 * the Curtis motor controller to accept autonomous throttle input.
 *
 * @return ESP_OK on success, or an error code if GPIO write fails
 */
esp_err_t relay_dpdt_my5nj_energize(void);

/**
 * @brief De-energize the relay, restoring manual pedal control.
 *
 * Drives the transistor base LOW, de-energizing the relay coil.
 * Pole 1 returns throttle source to manual pedal (NC). Pole 2
 * restores normal pedal microswitch operation (NC). This is the
 * safe state.
 *
 * @return ESP_OK on success, or an error code if GPIO write fails
 */
esp_err_t relay_dpdt_my5nj_deenergize(void);

/**
 * @brief Check if the relay is currently energized.
 *
 * Reads the GPIO output level to determine whether the relay coil
 * is energized (autonomous throttle active, pedal bypassed).
 *
 * @return true if the relay is energized, false otherwise
 */
bool relay_dpdt_my5nj_is_energized(void);

#ifdef __cplusplus
}
#endif
