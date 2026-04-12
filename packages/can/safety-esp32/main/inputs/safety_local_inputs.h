/**
 * @file safety_local_inputs.h
 * @brief Safety-local stop input sampling and ultrasonic filtering helpers.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "obstacle_filter.h"

typedef struct
{
	gpio_num_t gpio;
	int active_level;
	bool enable_pullup;
	bool enable_pulldown;
	const char *name;
} estop_input_config_t;

typedef struct
{
	uint8_t estop_disengage_count;
	uint8_t ultrasonic_health_hysteresis;
	uint32_t ultrasonic_engage_hold_ms;
	uint8_t ultrasonic_disengage_count;
	uint16_t ultrasonic_stop_distance_mm;
	uint16_t ultrasonic_clear_distance_mm;
} safety_local_inputs_config_t;

typedef struct
{
	uint8_t push_button_clear_count;
	bool push_button_debounced;
	uint8_t rf_remote_clear_count;
	bool rf_remote_debounced;
	bool ultrasonic_health_prev;
	bool ultrasonic_health_seen;
	uint8_t ultrasonic_health_counter;
	obstacle_filter_t ultrasonic_filter;
} safety_local_inputs_state_t;

typedef struct
{
	bool push_button_active;
	bool rf_remote_active;
	bool ultrasonic_too_close;
	bool ultrasonic_healthy;
} safety_local_inputs_snapshot_t;

esp_err_t estop_input_init(const estop_input_config_t *config);
bool estop_input_is_active(const estop_input_config_t *config);
void safety_local_inputs_init_state(safety_local_inputs_state_t *state, const safety_local_inputs_config_t *config);
void safety_local_inputs_reset_ultrasonic(safety_local_inputs_state_t *state,
                                          const safety_local_inputs_config_t *config);
void safety_local_inputs_poll(safety_local_inputs_state_t *state, const safety_local_inputs_config_t *config,
                              bool push_button_ready, const estop_input_config_t *push_button_config,
                              bool rf_remote_ready, const estop_input_config_t *rf_remote_config,
                              bool ultrasonic_ready, uint32_t now_ms, safety_local_inputs_snapshot_t *out);
