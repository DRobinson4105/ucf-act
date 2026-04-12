/**
 * @file test_control_driver_inputs.cpp
 * @brief Unit tests for Control driver input aggregation helpers.
 */

#include "test_harness.h"

#include "control_driver_inputs.h"
#include "esp_idf_mock.h"

namespace
{

adc_12bitsar_config_t default_pedal_cfg(void)
{
	adc_12bitsar_config_t cfg = {
		.adc_unit = ADC_UNIT_1,
		.adc_channel = ADC_CHANNEL_0,
	};
	return cfg;
}

optocoupler_pc817_config_t default_fr_cfg(void)
{
	optocoupler_pc817_config_t cfg = {
		.forward_gpio = 22,
		.reverse_gpio = 23,
	};
	return cfg;
}

control_driver_inputs_config_t default_inputs_cfg(const adc_12bitsar_config_t *pedal_cfg,
                                                  const optocoupler_pc817_config_t *fr_cfg)
{
	control_driver_inputs_config_t cfg = {
		.pedal_adc_config = pedal_cfg,
		.fr_pc817_config = fr_cfg,
		.pedal_threshold_mv = 500,
		.pedal_rearm_ms = 500,
		.pedal_oversample_count = 1,
		.pedal_trigger_count = 3,
	};
	return cfg;
}

void set_fr_forward(void)
{
	mock_gpio_levels[22] = 0;
	mock_gpio_levels[23] = 1;
}

void test_pedal_init_seeds_ready_state(void)
{
	mock_reset_all();
	mock_adc_cali_create_result = ESP_OK;
	mock_adc_cali_voltage_mv = 650;

	adc_12bitsar_config_t pedal_cfg = default_pedal_cfg();
	optocoupler_pc817_config_t fr_cfg = default_fr_cfg();
	control_driver_inputs_config_t cfg = default_inputs_cfg(&pedal_cfg, &fr_cfg);
	control_driver_inputs_t state = {};

	assert(control_driver_inputs_init_pedal(&state, &cfg) == ESP_OK);
	assert(state.pedal_ready);
	assert(state.pedal_mv == 650);
	assert(control_driver_inputs_pedal_pressed(&state, &cfg));
	assert(!control_driver_inputs_pedal_rearmed(&state, &cfg));
}

void test_pedal_trigger_requires_consecutive_samples(void)
{
	mock_reset_all();
	mock_adc_cali_create_result = ESP_OK;
	mock_adc_cali_voltage_mv = 100;

	adc_12bitsar_config_t pedal_cfg = default_pedal_cfg();
	optocoupler_pc817_config_t fr_cfg = default_fr_cfg();
	control_driver_inputs_config_t cfg = default_inputs_cfg(&pedal_cfg, &fr_cfg);
	control_driver_inputs_t state = {};

	assert(control_driver_inputs_init_pedal(&state, &cfg) == ESP_OK);
	assert(!control_driver_inputs_pedal_pressed(&state, &cfg));

	control_driver_inputs_update_result_t update = {};
	mock_adc_cali_voltage_mv = 650;
	control_driver_inputs_update(&state, &cfg, 0, &update);
	assert(!update.pedal_runtime_lost);
	assert(!control_driver_inputs_pedal_pressed(&state, &cfg));
	control_driver_inputs_update(&state, &cfg, 10, &update);
	assert(!control_driver_inputs_pedal_pressed(&state, &cfg));
	control_driver_inputs_update(&state, &cfg, 20, &update);
	assert(control_driver_inputs_pedal_pressed(&state, &cfg));
}

void test_pedal_rearm_requires_hold_time(void)
{
	mock_reset_all();
	mock_adc_cali_create_result = ESP_OK;
	mock_adc_cali_voltage_mv = 650;

	adc_12bitsar_config_t pedal_cfg = default_pedal_cfg();
	optocoupler_pc817_config_t fr_cfg = default_fr_cfg();
	control_driver_inputs_config_t cfg = default_inputs_cfg(&pedal_cfg, &fr_cfg);
	control_driver_inputs_t state = {};
	assert(control_driver_inputs_init_pedal(&state, &cfg) == ESP_OK);

	control_driver_inputs_update_result_t update = {};
	mock_adc_cali_voltage_mv = 100;
	control_driver_inputs_update(&state, &cfg, 100, &update);
	assert(!control_driver_inputs_pedal_rearmed(&state, &cfg));
	control_driver_inputs_update(&state, &cfg, 599, &update);
	assert(!control_driver_inputs_pedal_rearmed(&state, &cfg));
	control_driver_inputs_update(&state, &cfg, 600, &update);
	assert(control_driver_inputs_pedal_rearmed(&state, &cfg));
}

void test_pedal_runtime_loss_is_reported(void)
{
	mock_reset_all();
	mock_adc_cali_create_result = ESP_FAIL;
	mock_adc_read_raw_value = 1000;

	adc_12bitsar_config_t pedal_cfg = default_pedal_cfg();
	optocoupler_pc817_config_t fr_cfg = default_fr_cfg();
	control_driver_inputs_config_t cfg = default_inputs_cfg(&pedal_cfg, &fr_cfg);
	cfg.pedal_oversample_count = 2;
	control_driver_inputs_t state = {};
	assert(control_driver_inputs_init_pedal(&state, &cfg) == ESP_OK);

	mock_adc_read_raw_value = -1;
	control_driver_inputs_update_result_t update = {};
	control_driver_inputs_update(&state, &cfg, 0, &update);
	assert(update.pedal_runtime_lost);
	assert(state.pedal_ready);
}

void test_fr_state_returns_neutral_until_ready(void)
{
	mock_reset_all();
	set_fr_forward();

	adc_12bitsar_config_t pedal_cfg = default_pedal_cfg();
	optocoupler_pc817_config_t fr_cfg = default_fr_cfg();
	control_driver_inputs_config_t cfg = default_inputs_cfg(&pedal_cfg, &fr_cfg);
	control_driver_inputs_t state = {};

	assert(control_driver_inputs_fr_state(&state) == FR_STATE_NEUTRAL);
	assert(control_driver_inputs_init_fr(&state, &cfg) == ESP_OK);
	assert(control_driver_inputs_fr_state(&state) == FR_STATE_FORWARD);
}

} // namespace

int main(void)
{
	printf("\n=== control_driver_inputs unit tests ===\n\n");

	TEST(test_pedal_init_seeds_ready_state);
	TEST(test_pedal_trigger_requires_consecutive_samples);
	TEST(test_pedal_rearm_requires_hold_time);
	TEST(test_pedal_runtime_loss_is_reported);
	TEST(test_fr_state_returns_neutral_until_ready);

	TEST_REPORT();
	TEST_EXIT();
}
