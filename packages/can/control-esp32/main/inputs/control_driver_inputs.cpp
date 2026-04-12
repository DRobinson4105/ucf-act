/**
 * @file control_driver_inputs.cpp
 * @brief Control-local pedal and F/R input sampling helpers.
 */

#include "control_driver_inputs.h"

#include "esp_log.h"

namespace
{

void reset_pedal_tracking(control_driver_inputs_t *state, const control_driver_inputs_config_t *config)
{
	state->pedal_was_above = (state->pedal_mv >= config->pedal_threshold_mv);
	state->pedal_above_count = state->pedal_was_above ? config->pedal_trigger_count : 0;
	state->pedal_below_threshold_since = 0;
}

} // namespace

esp_err_t control_driver_inputs_init_pedal(control_driver_inputs_t *state, const control_driver_inputs_config_t *config)
{
	if (!state || !config || !config->pedal_adc_config)
		return ESP_ERR_INVALID_ARG;

	adc_12bitsar_deinit();
	state->pedal_ready = false;

	esp_err_t err = adc_12bitsar_init(config->pedal_adc_config);
	if (err != ESP_OK)
		return err;

	err = adc_12bitsar_read_mv_checked(&state->pedal_mv);
	if (err != ESP_OK)
		return err;

	reset_pedal_tracking(state, config);
	state->pedal_ready = true;
	return ESP_OK;
}

esp_err_t control_driver_inputs_init_fr(control_driver_inputs_t *state, const control_driver_inputs_config_t *config)
{
	if (!state || !config || !config->fr_pc817_config)
		return ESP_ERR_INVALID_ARG;

	state->fr_ready = false;

	esp_err_t err = optocoupler_pc817_init(config->fr_pc817_config);
	if (err != ESP_OK)
		return err;

	state->fr_ready = true;
	return ESP_OK;
}

void control_driver_inputs_update(control_driver_inputs_t *state, const control_driver_inputs_config_t *config,
                                  uint32_t now_ms, control_driver_inputs_update_result_t *out)
{
	if (out)
		*out = {};
	if (!state || !config)
		return;

	if (state->pedal_ready)
	{
		uint32_t sample_sum = 0;
		uint8_t good_samples = 0;
		for (uint8_t i = 0; i < config->pedal_oversample_count; ++i)
		{
			uint16_t sample = 0;
			if (adc_12bitsar_read_mv_checked(&sample) == ESP_OK)
			{
				sample_sum += sample;
				++good_samples;
			}
		}

		if (good_samples == 0)
		{
			if (out)
				out->pedal_runtime_lost = true;
		}
		else
		{
			state->pedal_mv = (uint16_t)(sample_sum / good_samples);

			if (state->pedal_mv >= config->pedal_threshold_mv)
			{
				if (state->pedal_above_count < config->pedal_trigger_count)
					++state->pedal_above_count;

				if (state->pedal_above_count >= config->pedal_trigger_count)
				{
					if (!state->pedal_was_above)
					{
#ifdef CONFIG_LOG_INPUT_PEDAL_CHANGES
						ESP_LOGI("PEDAL", "Pressed: mv=%u threshold=%u", state->pedal_mv, config->pedal_threshold_mv);
#endif
					}
					state->pedal_was_above = true;
					state->pedal_below_threshold_since = 0;
				}
			}
			else
			{
				state->pedal_above_count = 0;

				if (state->pedal_was_above && state->pedal_below_threshold_since == 0)
					state->pedal_below_threshold_since = now_ms;

				if (state->pedal_below_threshold_since > 0 &&
				    (now_ms - state->pedal_below_threshold_since) >= config->pedal_rearm_ms)
				{
					state->pedal_was_above = false;
					state->pedal_below_threshold_since = 0;
#ifdef CONFIG_LOG_INPUT_PEDAL_CHANGES
					ESP_LOGI("PEDAL", "Re-armed: mv=%u rearm_ms=%lu", state->pedal_mv, (unsigned long)config->pedal_rearm_ms);
#endif
				}
			}
		}
	}

	if (state->fr_ready)
		optocoupler_pc817_update(now_ms);
}

bool control_driver_inputs_pedal_pressed(const control_driver_inputs_t *state,
                                         const control_driver_inputs_config_t *config)
{
	return state && config && state->pedal_ready && (state->pedal_above_count >= config->pedal_trigger_count);
}

bool control_driver_inputs_pedal_rearmed(const control_driver_inputs_t *state,
                                         const control_driver_inputs_config_t *config)
{
	if (!state || !config)
		return true;
	if (!state->pedal_ready)
		return true;
	if (state->pedal_mv >= config->pedal_threshold_mv)
		return false;
	return !state->pedal_was_above;
}

fr_state_t control_driver_inputs_fr_state(const control_driver_inputs_t *state)
{
	if (!state || !state->fr_ready)
		return FR_STATE_NEUTRAL;
	return optocoupler_pc817_get_state();
}
