/**
 * @file control_heartbeat_tx.h
 * @brief Control heartbeat transmit and CAN TX fault tracking helpers.
 */
#pragma once

#include "can_protocol.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	const char *tag;
	const char *tag_tx;
	portMUX_TYPE *can_tx_lock;
	portMUX_TYPE *hb_seq_lock;
	volatile bool *twai_ready;
	volatile bool *can_recovery_in_progress;
	volatile uint8_t *can_tx_in_flight;
	volatile node_seq_t *heartbeat_seq;
	volatile node_state_t *control_state;
	volatile node_fault_t *fault_flags;
	volatile node_stop_t *stop_flags;
	volatile node_status_flags_t *heartbeat_status_flags;
	volatile bool *orin_link_ready;
	uint8_t *can_tx_fail_count;
	uint8_t can_tx_fail_threshold;
} control_heartbeat_tx_context_t;

void control_heartbeat_tx_track(control_heartbeat_tx_context_t *ctx, esp_err_t err);
void control_heartbeat_tx_send(control_heartbeat_tx_context_t *ctx);

#ifdef __cplusplus
}
#endif
