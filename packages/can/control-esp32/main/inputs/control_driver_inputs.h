/**
 * @file control_driver_inputs.h
 * @brief Control-local pedal and F/R input sampling helpers.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "adc_12bitsar.h"
#include "esp_err.h"
#include "optocoupler_pc817.h"

typedef struct
{
	const adc_12bitsar_config_t *pedal_adc_config;
	const optocoupler_pc817_config_t *fr_pc817_config;
	uint16_t pedal_threshold_mv;
	uint32_t pedal_rearm_ms;
	uint8_t pedal_oversample_count;
	uint8_t pedal_trigger_count;
} control_driver_inputs_config_t;

typedef struct
{
	bool pedal_ready;
	bool fr_ready;
	uint16_t pedal_mv;
	bool pedal_was_above;
	uint32_t pedal_below_threshold_since;
	uint8_t pedal_above_count;
} control_driver_inputs_t;

typedef struct
{
	bool pedal_runtime_lost;
} control_driver_inputs_update_result_t;

esp_err_t control_driver_inputs_init_pedal(control_driver_inputs_t *state,
                                           const control_driver_inputs_config_t *config);
esp_err_t control_driver_inputs_init_fr(control_driver_inputs_t *state,
                                        const control_driver_inputs_config_t *config);
void control_driver_inputs_update(control_driver_inputs_t *state, const control_driver_inputs_config_t *config,
                                  uint32_t now_ms, control_driver_inputs_update_result_t *out);
bool control_driver_inputs_pedal_pressed(const control_driver_inputs_t *state,
                                         const control_driver_inputs_config_t *config);
bool control_driver_inputs_pedal_rearmed(const control_driver_inputs_t *state,
                                         const control_driver_inputs_config_t *config);
fr_state_t control_driver_inputs_fr_state(const control_driver_inputs_t *state);
