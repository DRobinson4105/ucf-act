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

	if (!ctx->can_tx_lock || !ctx->twai_ready || !ctx->can_recovery_in_progress || !ctx->can_tx_in_flight)
	{
#ifndef CONFIG_LOG_TRANSPORT_CAN_HEARTBEAT_TX
		(void)log_as_change;
#endif
		return;
	}

	if (!can_runtime_try_reserve_tx(ctx->can_tx_lock, ctx->twai_ready, ctx->can_recovery_in_progress,
	                                ctx->can_tx_in_flight))
	{
#ifndef CONFIG_LOG_TRANSPORT_CAN_HEARTBEAT_TX
		(void)log_as_change;
#endif
		return;
	}

#ifdef CONFIG_LOG_HEALTH_CAN_RECOVERY
	if (!heap_caps_check_integrity_all(true))
	{
		ESP_LOGE(ctx->tag, "HEAP CORRUPTION detected in send_safety_heartbeat! free=%lu",
		         (unsigned long)esp_get_free_heap_size());
	}
#endif

	twai_message_t msg = {};
	msg.identifier = CAN_ID_SAFETY_HEARTBEAT;
	msg.data_length_code = 8;
	memcpy(msg.data, data, 8);
	esp_err_t err = can_twai_send(&msg, pdMS_TO_TICKS(10));

	can_runtime_release_tx(ctx->can_tx_lock, ctx->can_tx_in_flight);
	safety_heartbeat_tx_track(ctx, err);

#ifdef CONFIG_LOG_TRANSPORT_CAN_HEARTBEAT_TX
	if (err != ESP_OK)
	{
		if (!*ctx->hb_tx_failing)
		{
			ESP_LOGE(ctx->tag_tx, "Heartbeat TX failing: %s", esp_err_to_name(err));
			*ctx->hb_tx_failing = true;
		}
	}
	else
	{
		if (*ctx->hb_tx_failing)
		{
			ESP_LOGI(ctx->tag_tx, "Heartbeat TX recovered");
			*ctx->hb_tx_failing = false;
		}
		ESP_LOGI(ctx->tag_tx, "HB TX: target=%s fault=%s stop=%s trigger=%s", node_state_to_string(target_state),
		         ctx->fault_to_log_string(fault_flags), node_stop_to_string(stop_flags),
		         log_as_change ? "change" : "periodic");
	}
#else
	(void)log_as_change;
#endif
}
