/**
 * @file safety_local_inputs.cpp
 * @brief Safety-local stop input sampling and ultrasonic filtering helpers.
 */

#include "safety_local_inputs.h"

#include "driver/gpio.h"
#include "ultrasonic_a02yyuw.h"

namespace
{

void reset_obstacle_filter(obstacle_filter_t *filter, const safety_local_inputs_config_t *config)
{
	filter->engage_hold_ms = config->ultrasonic_engage_hold_ms;
	filter->disengage_count = config->ultrasonic_disengage_count;
	filter->assert_since_ms = 0;
	filter->clear_count = 0;
	filter->assert_pending = false;
	filter->latched = false;
}

void update_estop_latch(bool raw_active, uint8_t disengage_count, uint8_t *clear_count, bool *latched)
{
	if (raw_active)
	{
		*latched = true;
		*clear_count = 0;
		return;
	}

	if (*clear_count < disengage_count)
		(*clear_count)++;
	if (*clear_count >= disengage_count)
		*latched = false;
}

bool update_ultrasonic_health(safety_local_inputs_state_t *state, const safety_local_inputs_config_t *config,
                              bool raw_healthy)
{
	if (state->ultrasonic_health_seen)
	{
		if (raw_healthy != state->ultrasonic_health_prev)
		{
			state->ultrasonic_health_counter++;
			if (state->ultrasonic_health_counter >= config->ultrasonic_health_hysteresis)
			{
				state->ultrasonic_health_prev = raw_healthy;
				state->ultrasonic_health_counter = 0;
			}
		}
		else
		{
			state->ultrasonic_health_counter = 0;
		}
		return state->ultrasonic_health_prev;
	}

	state->ultrasonic_health_prev = raw_healthy;
	state->ultrasonic_health_seen = true;
	state->ultrasonic_health_counter = 0;
	return raw_healthy;
}

} // namespace

esp_err_t estop_input_init(const estop_input_config_t *config)
{
	if (!config)
		return ESP_ERR_INVALID_ARG;

	gpio_config_t io_conf = {
		.pin_bit_mask = 1ULL << config->gpio,
		.mode = GPIO_MODE_INPUT,
		.pull_up_en = config->enable_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
		.pull_down_en = config->enable_pulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	return gpio_config(&io_conf);
}

bool estop_input_is_active(const estop_input_config_t *config)
{
	if (!config)
		return true;
	return gpio_get_level(config->gpio) == config->active_level;
}

void safety_local_inputs_init_state(safety_local_inputs_state_t *state, const safety_local_inputs_config_t *config)
{
	if (!state || !config)
		return;

	state->push_button_clear_count = 0;
	state->push_button_debounced = false;
	state->rf_remote_clear_count = 0;
	state->rf_remote_debounced = false;
	state->ultrasonic_health_prev = false;
	state->ultrasonic_health_seen = false;
	state->ultrasonic_health_counter = 0;
	reset_obstacle_filter(&state->ultrasonic_filter, config);
}

void safety_local_inputs_reset_ultrasonic(safety_local_inputs_state_t *state,
                                          const safety_local_inputs_config_t *config)
{
	if (!state || !config)
		return;

	state->ultrasonic_health_prev = false;
	state->ultrasonic_health_seen = false;
	state->ultrasonic_health_counter = 0;
	reset_obstacle_filter(&state->ultrasonic_filter, config);
}

void safety_local_inputs_poll(safety_local_inputs_state_t *state, const safety_local_inputs_config_t *config,
                              bool push_button_ready, const estop_input_config_t *push_button_config,
                              bool rf_remote_ready, const estop_input_config_t *rf_remote_config,
                              bool ultrasonic_ready, uint32_t now_ms, safety_local_inputs_snapshot_t *out)
{
	if (!state || !config || !out)
		return;

	bool raw_push_button = push_button_ready ? estop_input_is_active(push_button_config) : true;
	bool raw_rf_remote = rf_remote_ready ? estop_input_is_active(rf_remote_config) : true;
	update_estop_latch(raw_push_button, config->estop_disengage_count, &state->push_button_clear_count,
	                   &state->push_button_debounced);
	update_estop_latch(raw_rf_remote, config->estop_disengage_count, &state->rf_remote_clear_count,
	                   &state->rf_remote_debounced);

	bool raw_ultrasonic_healthy = ultrasonic_ready ? ultrasonic_a02yyuw_is_healthy() : false;
	bool filtered_ultrasonic_healthy = update_ultrasonic_health(state, config, raw_ultrasonic_healthy);

	bool raw_ultrasonic_too_close = false;
	if (ultrasonic_ready)
	{
		uint16_t threshold_mm = state->ultrasonic_filter.latched ? config->ultrasonic_clear_distance_mm
		                                                         : config->ultrasonic_stop_distance_mm;
		raw_ultrasonic_too_close = ultrasonic_a02yyuw_is_too_close(threshold_mm, nullptr);
	}

	out->push_button_active = state->push_button_debounced;
	out->rf_remote_active = state->rf_remote_debounced;
	out->ultrasonic_healthy = filtered_ultrasonic_healthy;
	out->ultrasonic_too_close = obstacle_filter_update(&state->ultrasonic_filter, raw_ultrasonic_too_close, now_ms);
}
