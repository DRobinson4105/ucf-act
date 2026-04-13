/**
 * @file safety_heartbeat_tx.cpp
 * @brief Safety heartbeat transmit plus CAN/Orin link TX helpers.
 */

#include "safety_heartbeat_tx.h"

#include <cstring>

#include "can_runtime.h"
#include "can_twai.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "orin_link_protocol.h"
#include "usb_serial_link.h"

void safety_heartbeat_tx_track(safety_heartbeat_tx_context_t *ctx, esp_err_t err)
{
	if (!ctx || !ctx->can_tx_lock || !ctx->twai_ready || !ctx->can_recovery_in_progress || !ctx->can_tx_fail_count)
		return;

#ifdef CONFIG_BYPASS_CAN_TWAI
	(void)err;
	return;
#endif

	bool triggered = can_runtime_note_tx_result(ctx->can_tx_lock, ctx->twai_ready, ctx->can_recovery_in_progress,
	                                            ctx->can_tx_fail_count, ctx->can_tx_fail_threshold, err);
	if (triggered)
	{
#ifdef CONFIG_LOG_HEALTH_CAN_RECOVERY
		ESP_LOGE(ctx->tag, "CAN bus unhealthy after %d TX failures", ctx->can_tx_fail_threshold);
#endif
	}
}

void safety_heartbeat_tx_send(safety_heartbeat_tx_context_t *ctx, bool log_as_change)
{
	if (!ctx || !ctx->hb_seq_lock || !ctx->heartbeat_seq || !ctx->target_state || !ctx->fault_flags ||
	    !ctx->stop_flags || !ctx->battery_soc || !ctx->hb_tx_failing || !ctx->fault_to_log_string)
	{
		return;
	}

	uint8_t data[8] = {};
	node_seq_t seq = 0;
	node_state_t target_state = NODE_STATE_NOT_READY;
	node_fault_t fault_flags = NODE_FAULT_NONE;
	node_stop_t stop_flags = NODE_STOP_NONE;
	uint8_t soc = 0;

	taskENTER_CRITICAL(ctx->hb_seq_lock);
	seq = *ctx->heartbeat_seq;
	*ctx->heartbeat_seq = (node_seq_t)(seq + 1);
	target_state = *ctx->target_state;
	fault_flags = *ctx->fault_flags;
	stop_flags = *ctx->stop_flags;
	soc = *ctx->battery_soc;
	taskEXIT_CRITICAL(ctx->hb_seq_lock);

	node_heartbeat_t hb = {
		.sequence = seq,
		.state = target_state,
		.fault_flags = fault_flags,
		.status_flags = 0,
		.stop_flags = stop_flags,
		.soc_pct = soc,
	};
	can_encode_heartbeat(data, &hb);

	if (ctx->orin_link_ready && *ctx->orin_link_ready)
	{
		usb_serial_link_message_t serial_msg = {};
		uint8_t payload_len = 0;
		if (orin_link_encode_safety_heartbeat_message(&serial_msg.type, serial_msg.payload, &payload_len, &hb))
		{
			serial_msg.payload_len = payload_len;
			(void)usb_serial_link_write_message(&serial_msg, ORIN_LINK_SYNC_BYTE, pdMS_TO_TICKS(10),
			                                    pdMS_TO_TICKS(10));
		}
	}

	// Heartbeat is sent via UART to Orin only — no CAN TX.
	// Orin forwards it to Control ESP32 over the Control UART link.
	(void)log_as_change;
}
