/**
 * @file multiplexer_dg408djz.h
 * @brief DG408DJZ 8-channel analog multiplexer for throttle level selection.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
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
typedef struct
{
	gpio_num_t a0;
	gpio_num_t a1;
	gpio_num_t a2;
	gpio_num_t en;
} multiplexer_dg408djz_config_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Initialize GPIO pins and set the mux to its safe state.
 *
 * Configures the address (A0-A2) and enable (EN) pins as outputs and
 * drives them to the safe state (EN LOW, address lines zeroed) so the
 * mux output is disabled on startup.
 *
 * @param config  GPIO pin assignments for address and enable lines
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t multiplexer_dg408djz_init(const multiplexer_dg408djz_config_t *config);

// ============================================================================
// Level Control
// ============================================================================

/**
 * @brief Set the throttle level or disable mux output.
 *
 * Writes the 3-bit channel address to the A0-A2 lines and asserts EN.
 * Pass -1 to disable the mux output (EN LOW) instead.
 *
 * @param level  Throttle level 0-7 (0 = minimum, 7 = maximum), or -1 to disable
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t multiplexer_dg408djz_set_level(int8_t level);

/**
 * @brief Disable the mux output.
 *
 * Drives EN LOW to disconnect the mux output without affecting
 * the throttle source relay state.
 *
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t multiplexer_dg408djz_disable(void);

/**
 * @brief Get the current throttle level.
 *
 * Returns the most recently set channel address, or -1 if the mux
 * output is currently disabled.
 *
 * @return Throttle level 0-7 if active, or -1 if disabled
 */
int8_t multiplexer_dg408djz_get_level(void);

// ============================================================================
// Autonomous Mode Control
// ============================================================================

/**
 * @brief Enable autonomous mode.
 *
 * Sets the mux to throttle level 0 and marks autonomous mode as active.
 * The caller must energize the throttle source relay (relay_srd05vdc)
 * separately to route the mux output to the motor controller.
 *
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t multiplexer_dg408djz_enable_autonomous(void);

/**
 * @brief Emergency stop: immediately disable the mux and clear autonomous flag.
 *
 * Drives EN LOW to cut mux output and resets the autonomous mode flag.
 * The caller must de-energize the throttle source relay separately to
 * complete the emergency stop sequence.
 *
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t multiplexer_dg408djz_emergency_stop(void);

/**
 * @brief Check if the mux is currently in autonomous mode.
 *
 * Returns true after a successful call to
 * multiplexer_dg408djz_enable_autonomous() and until autonomous mode
 * is cleared by multiplexer_dg408djz_emergency_stop().
 *
 * @return true if autonomous mode is active, false otherwise
 */
bool multiplexer_dg408djz_is_autonomous(void);

#ifdef __cplusplus
}
#endif
