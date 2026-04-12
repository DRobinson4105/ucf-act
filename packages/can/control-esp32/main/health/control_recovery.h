/**
 * @file control_recovery.h
 * @brief Control-local component retry and CAN recovery helpers.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "can_protocol.h"
#include "dac_mcp4728.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "relay_dpdt_my5nj.h"

#include "control_driver_inputs.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef void (*control_recovery_quiesce_fn_t)(void);

typedef struct
{
	const char *tag;
	uint32_t *last_retry_ms;
	uint32_t retry_interval_ms;
	uint16_t retry_log_every_n;
	portMUX_TYPE *can_tx_lock;
	volatile bool *can_recovery_in_progress;
	volatile bool *twai_ready;
	gpio_num_t twai_tx_gpio;
	gpio_num_t twai_rx_gpio;
	node_state_t probe_state;
	node_fault_t probe_fault_flags;
	node_stop_t probe_stop_flags;
	node_status_flags_t probe_status_flags;
	control_recovery_quiesce_fn_t quiesce_can_rx;
	const dac_mcp4728_config_t *dac_cfg;
	volatile bool *dac_ready;
	const relay_dpdt_my5nj_config_t *dpdt_relay_cfg;
	volatile bool *dpdt_relay_ready;
	control_driver_inputs_t *driver_inputs;
	const control_driver_inputs_config_t *driver_inputs_cfg;
	const optocoupler_pc817_config_t *fr_pc817_cfg;
	volatile bool *motor_uim2852_braking_ready;
	volatile bool *motor_uim2852_steering_ready;
} control_recovery_context_t;

void control_retry_failed_components(control_recovery_context_t *ctx);

#ifdef __cplusplus
}
#endif
