/**
 * @file control_test_mode.h
 * @brief Standalone actuator test mode for Control ESP32.
 */
#pragma once

#include "sdkconfig.h"
#ifdef CONFIG_CONTROL_TEST_MODE

#include <stdint.h>
#include <stdbool.h>

#include "driver/gpio.h"

#include "dac_mcp4728.h"
#include "relay_dpdt_my5nj.h"
#include "control_driver_inputs.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef enum
{
	CONTROL_TEST_ACTUATOR_THROTTLE = 0,
	CONTROL_TEST_ACTUATOR_STEERING = 1,
	CONTROL_TEST_ACTUATOR_BRAKING = 2,
} control_test_actuator_t;

typedef struct
{
	control_test_actuator_t actuator;

	const dac_mcp4728_config_t *dac_cfg;
	const relay_dpdt_my5nj_config_t *relay_cfg;
	control_driver_inputs_t *driver_inputs;
	const control_driver_inputs_config_t *driver_inputs_cfg;
	uint16_t pedal_threshold_mv;
	uint32_t fr_debounce_ms;
	bool bypass_input_fr_sensor;
	bool bypass_input_pedal_adc;

	uint8_t motor_node_id;
	const char *motor_label;
	gpio_num_t twai_tx_gpio;
	gpio_num_t twai_rx_gpio;
	int32_t motor_min_position;
	int32_t motor_max_position;
} control_test_mode_context_t;

void control_test_mode_task(void *param);

#ifdef __cplusplus
}
#endif

#endif // CONFIG_CONTROL_TEST_MODE
