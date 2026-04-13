/**
 * @file control_orin_link.cpp
 * @brief Control-side Orin UART link helpers.
 *
 * Receives two message types from the Orin:
 *   - Type 0x01 (PLANNER_COMMAND): throttle/steering/braking commands
 *   - Type 0x02 (PLANNER_HEARTBEAT): planner liveness + autonomy request
 */

#include "control_orin_link.h"

#include "can_protocol.h"
#include "esp_log.h"
#include "heartbeat_monitor.h"
#include "node_support.h"
#include "orin_link_protocol.h"
#include "usb_serial_link.h"

void control_orin_link_rx_task(void *param)
{
	control_orin_link_rx_context_t *ctx = static_cast<control_orin_link_rx_context_t *>(param);
	if (!ctx || !ctx->cmd_lock || !ctx->snapshot)
	{
		vTaskDelete(nullptr);
		return;
	}

	node_task_wdt_add_self_or_log(ctx->tag, "control_orin_rx_task");
	bool wdt_reset_failed = false;
	usb_serial_link_message_t msg = {};

	while (true)
	{
		node_task_wdt_reset_or_log(ctx->tag, "control_orin_rx_task", &wdt_reset_failed);

		esp_err_t err = usb_serial_link_read_message(&msg, ORIN_LINK_SYNC_BYTE, pdMS_TO_TICKS(100), pdMS_TO_TICKS(50));
		if (err == ESP_ERR_TIMEOUT)
			continue;
		if (err != ESP_OK)
		{
			ESP_LOGW(ctx->tag, "Orin RX read failed: %s", esp_err_to_name(err));
			vTaskDelay(pdMS_TO_TICKS(10));
			continue;
		}

		if (msg.type == ORIN_LINK_MSG_PLANNER_COMMAND)
		{
			if (!control_can_rx_process_planner_command_payload(msg.payload, msg.payload_len, ctx->cmd_lock, ctx->snapshot,
			                                                    ctx->tag, "Orin planner cmd"))
			{
				ESP_LOGW(ctx->tag, "Orin planner command decode failed (len=%u)", msg.payload_len);
			}
		}
		else if (msg.type == ORIN_LINK_MSG_PLANNER_HEARTBEAT)
		{
			// Feed planner heartbeat to the heartbeat monitor for liveness tracking.
			if (ctx->monitor)
			{
				node_heartbeat_t hb;
				if (can_decode_heartbeat(msg.payload, msg.payload_len, &hb))
				{
					heartbeat_monitor_update(ctx->monitor, ctx->planner_node_handle, hb.sequence, hb.state);
				}
			}
#ifdef CONFIG_LOG_TRANSPORT_ORIN_PLANNER_HEARTBEAT_RX
			{
				node_heartbeat_t hb;
				if (can_decode_heartbeat(msg.payload, msg.payload_len, &hb))
					ESP_LOGI(ctx->tag, "Orin planner HB: seq=%u state=%s fault=%s",
					         hb.sequence, node_state_to_string(hb.state), node_fault_to_string(hb.fault_flags));
			}
#endif
		}
		else if (msg.type == ORIN_LINK_MSG_SAFETY_HEARTBEAT)
		{
			// Safety heartbeat forwarded by Orin (replaces CAN-based safety heartbeat RX)
			if (!control_can_rx_process_safety_heartbeat_payload(msg.payload, msg.payload_len, ctx->cmd_lock,
			                                                     ctx->snapshot, ctx->monitor, ctx->safety_node_handle,
			                                                     ctx->tag, "Orin safety HB"))
			{
				ESP_LOGW(ctx->tag, "Orin safety heartbeat decode failed (len=%u)", msg.payload_len);
			}
		}
		else
		{
			ESP_LOGW(ctx->tag, "Ignoring Orin message type=%s len=%u", orin_link_message_type_to_string(msg.type),
			         msg.payload_len);
		}
	}
}
