/**
 * @file gpio_input.h
 * @brief Generic GPIO digital input driver with configurable active polarity.
 */
#pragma once

#include "driver/gpio.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// GPIO Input Driver
// ============================================================================
// Reads the state of a digital input pin with configurable active polarity
// and optional internal pull resistors.
//
// Used for binary sensors such as e-stop push buttons, RF remote receiver
// outputs, or any device that presents a logic-level signal indicating an
// on/off state.
//
// Safety:
//   - Fail-safe default: gpio_input_is_active() returns true (active) if
//     config is NULL, so a misconfigured or missing sensor blocks autonomy
//     rather than silently allowing it.
// ============================================================================

// ============================================================================
// Configuration
// ============================================================================

// gpio_input_config_t - digital input pin configuration
//   gpio:            GPIO pin number
//   active_level:    Logic level when the input is in its active state (0 or 1)
//   enable_pullup:   true to enable internal pull-up resistor
//   enable_pulldown: true to enable internal pull-down resistor
//   name:            Human-readable label for logging (e.g. "Push button", "RF remote")
typedef struct
{
	gpio_num_t gpio;
	int active_level;
	bool enable_pullup;
	bool enable_pulldown;
	const char *name;
} gpio_input_config_t;

// ============================================================================
// Initialization
// ============================================================================

/**
 * @brief Configure a GPIO pin as a digital input.
 *
 * Sets the pin direction to input and applies the pull-up / pull-down
 * resistor settings specified in @p config.
 *
 * @param config  Pin configuration (GPIO number, active level, pull resistors, name)
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t gpio_input_init(const gpio_input_config_t *config);

// ============================================================================
// State Reading
// ============================================================================

/**
 * @brief Read whether the input is in its active state.
 *
 * Compares the current GPIO level against the configured active_level.
 * If @p config is NULL the function returns true (fail-safe: a
 * misconfigured or missing sensor blocks autonomy rather than silently
 * allowing it).
 *
 * @param config  Pin configuration used during init (NULL triggers fail-safe)
 * @return true when the input is active or config is NULL, false when inactive
 */
bool gpio_input_is_active(const gpio_input_config_t *config);

#ifdef __cplusplus
}
#endif
