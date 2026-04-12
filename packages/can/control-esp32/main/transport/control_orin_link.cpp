/**
 * @file control_orin_link.cpp
 * @brief Control-side Orin UART link helpers.
 */

#include "control_orin_link.h"

#include "esp_log.h"
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

		if (msg.type != ORIN_LINK_MSG_PLANNER_COMMAND)
		{
			ESP_LOGW(ctx->tag, "Ignoring Orin message type=%s len=%u", orin_link_message_type_to_string(msg.type),
			         msg.payload_len);
			continue;
		}

		if (!control_can_rx_process_planner_command_payload(msg.payload, msg.payload_len, ctx->cmd_lock, ctx->snapshot,
		                                                    ctx->tag, "Orin planner cmd"))
		{
			ESP_LOGW(ctx->tag, "Orin planner command decode failed (len=%u)", msg.payload_len);
		}
	}
}
