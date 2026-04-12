/**
 * @file safety_orin_link.cpp
 * @brief Safety-side Orin UART link helpers.
 */

#include "safety_orin_link.h"

#include "esp_log.h"
#include "node_support.h"
#include "orin_link_protocol.h"
#include "usb_serial_link.h"

void safety_orin_link_rx_task(void *param)
{
	safety_orin_link_rx_context_t *ctx = static_cast<safety_orin_link_rx_context_t *>(param);
	if (!ctx || !ctx->monitor || !ctx->mirror_lock || !ctx->mirror)
	{
		vTaskDelete(nullptr);
		return;
	}

	node_task_wdt_add_self_or_log(ctx->tag, "safety_orin_rx_task");
	bool wdt_reset_failed = false;
	usb_serial_link_message_t msg = {};

	while (true)
	{
		node_task_wdt_reset_or_log(ctx->tag, "safety_orin_rx_task", &wdt_reset_failed);

		esp_err_t err = usb_serial_link_read_message(&msg, ORIN_LINK_SYNC_BYTE, pdMS_TO_TICKS(100), pdMS_TO_TICKS(50));
		if (err == ESP_ERR_TIMEOUT)
			continue;
		if (err != ESP_OK)
		{
			ESP_LOGW(ctx->tag, "Orin RX read failed: %s", esp_err_to_name(err));
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}

		if (msg.type != ORIN_LINK_MSG_PLANNER_HEARTBEAT)
		{
			ESP_LOGW(ctx->tag, "Ignoring Orin message type=%s len=%u", orin_link_message_type_to_string(msg.type),
			         msg.payload_len);
			continue;
		}

		if (!safety_can_rx_process_heartbeat_payload(msg.payload, msg.payload_len, ctx->monitor,
		                                             ctx->planner_node_handle, ctx->mirror_lock, ctx->mirror, true,
		                                             ctx->tag, "Orin planner HB"))
		{
			ESP_LOGW(ctx->tag, "Orin planner heartbeat decode failed (len=%u)", msg.payload_len);
		}
#ifdef CONFIG_LOG_TRANSPORT_ORIN_PLANNER_HEARTBEAT_RX
		else
		{
			node_heartbeat_t hb;
			if (can_decode_heartbeat(msg.payload, msg.payload_len, &hb))
				ESP_LOGI(ctx->tag, "Orin planner HB: seq=%u state=%s fault=%s",
				         hb.sequence, node_state_to_string(hb.state), node_fault_to_string(hb.fault_flags));
		}
#endif
	}
}
