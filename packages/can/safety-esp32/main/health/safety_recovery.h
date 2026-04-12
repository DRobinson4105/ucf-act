/**
 * @file safety_recovery.h
 * @brief Safety-local component retry and CAN recovery helpers.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "battery_monitor.h"
#include "can_protocol.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "relay_srd05vdc.h"
#include "ultrasonic_a02yyuw.h"

#include "safety_local_inputs.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef void (*safety_recovery_quiesce_fn_t)(void);

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
	uint8_t probe_soc_pct;
	safety_recovery_quiesce_fn_t quiesce_can_rx;
	const estop_input_config_t *push_button_cfg;
	volatile bool *push_button_init_ok;
	const estop_input_config_t *rf_remote_cfg;
	volatile bool *rf_remote_init_ok;
	const relay_srd05vdc_config_t *relay_cfg;
	volatile bool *relay_init_ok;
	const ultrasonic_a02yyuw_config_t *ultrasonic_cfg;
	volatile bool *ultrasonic_init_ok;
	TickType_t *ultrasonic_init_tick;
	safety_local_inputs_state_t *local_inputs;
	const safety_local_inputs_config_t *local_inputs_cfg;
	const battery_monitor_config_t *battery_cfg;
	volatile bool *battery_monitor_init_ok;
} safety_recovery_context_t;

void safety_retry_failed_components(safety_recovery_context_t *ctx);

#ifdef __cplusplus
}
#endif
