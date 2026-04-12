/**
 * @file can_runtime.h
 * @brief Shared CAN runtime helpers for TX reservation, recovery, and bus-failure handling.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "can_protocol.h"
#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	gpio_num_t tx_gpio;
	gpio_num_t rx_gpio;
	const char *log_tag;
	uint32_t probe_can_id;
	node_heartbeat_t probe_heartbeat;
	TickType_t probe_send_timeout;
	TickType_t probe_settle_delay;
} can_runtime_recovery_probe_t;

bool can_runtime_try_reserve_tx(portMUX_TYPE *lock, volatile bool *twai_ready, volatile bool *recovery_in_progress,
                                volatile uint8_t *in_flight);
void can_runtime_release_tx(portMUX_TYPE *lock, volatile uint8_t *in_flight);
bool can_runtime_try_begin_recovery(portMUX_TYPE *lock, volatile bool *recovery_in_progress);
void can_runtime_finish_recovery(portMUX_TYPE *lock, volatile bool *recovery_in_progress);
bool can_runtime_note_tx_result(portMUX_TYPE *lock, volatile bool *twai_ready, volatile bool *recovery_in_progress,
                                uint8_t *fail_count, uint8_t threshold, esp_err_t tx_err);
bool can_runtime_recover_with_probe(const can_runtime_recovery_probe_t *probe, esp_err_t *recover_err);

#ifdef __cplusplus
}
#endif
