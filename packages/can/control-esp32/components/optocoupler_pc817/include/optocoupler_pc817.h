/**
 * @file optocoupler_pc817.h
 * @brief F/R switch decode + debounce for dual PC817 inputs.
 */
#pragma once

#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "control_domain_types.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define FR_PC817_DEBOUNCE_MS 20

typedef struct
{
	gpio_num_t forward_gpio;
	gpio_num_t reverse_gpio;
} optocoupler_pc817_config_t;

/**
 * @brief Initialize dual PC817 optocoupler GPIOs for F/R switch reading.
 *
 * Configures the forward and reverse GPIO pins as inputs so the
 * driver can decode the forward/reverse switch position through
 * the PC817 optocoupler pair.
 *
 * @param config  GPIO pin assignments for forward and reverse inputs
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t optocoupler_pc817_init(const optocoupler_pc817_config_t *config);

/**
 * @brief Get the debounced forward/reverse switch state.
 *
 * Returns the current F/R switch position after debounce filtering.
 * The state is only updated by optocoupler_pc817_update(), so it
 * remains stable between update calls.
 *
 * @return One of the FR_STATE_* values indicating switch position
 */
fr_state_t optocoupler_pc817_get_state(void);

/**
 * @brief Get the raw (undebounced) forward/reverse switch state.
 *
 * Reads the GPIO levels directly and returns the instantaneous
 * switch position without any debounce filtering.
 *
 * @return One of the FR_STATE_* values indicating raw switch position
 */
fr_state_t optocoupler_pc817_get_state_raw(void);

/**
 * @brief Update the debounce state machine.
 *
 * Samples the raw GPIO state and advances the debounce filter.
 * Must be called periodically (recommended interval <= FR_PC817_DEBOUNCE_MS)
 * to keep the debounced state current.
 *
 * @param now_ms  Current system time in milliseconds
 */
void optocoupler_pc817_update(uint32_t now_ms);

#ifdef __cplusplus
}
#endif
