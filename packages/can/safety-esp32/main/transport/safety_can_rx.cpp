/**
 * @file safety_can_rx.cpp
 * @brief Safety CAN RX mirror/update helpers.
 */

#include "safety_can_rx.h"

#include "esp_log.h"

hb_mirror_snapshot_t safety_hb_mirror_read(portMUX_TYPE *lock, const volatile hb_mirror_snapshot_t *mirror)
{
	hb_mirror_snapshot_t snapshot = {};
	if (!lock || !mirror)
		return snapshot;

	taskENTER_CRITICAL(lock);
	snapshot.planner_state = mirror->planner_state;
	snapshot.planner_fault_flags = mirror->planner_fault_flags;
	snapshot.planner_status_flags = mirror->planner_status_flags;
	snapshot.planner_stop_flags = mirror->planner_stop_flags;
	snapshot.control_state = mirror->control_state;
	snapshot.control_fault_flags = mirror->control_fault_flags;
	snapshot.control_status_flags = mirror->control_status_flags;
	snapshot.control_stop_flags = mirror->control_stop_flags;
	taskEXIT_CRITICAL(lock);
	return snapshot;
}

bool safety_can_rx_process_heartbeat(const twai_message_t *msg, heartbeat_monitor_t *monitor, int node_handle,
                                     portMUX_TYPE *mirror_lock, volatile hb_mirror_snapshot_t *mirror,
                                     bool planner_node, const char *tag_rx, const char *label)
{
	if (!msg)
		return false;

	return safety_can_rx_process_heartbeat_payload(msg->data, msg->data_length_code, monitor, node_handle, mirror_lock,
	                                               mirror, planner_node, tag_rx, label);
}

bool safety_can_rx_process_heartbeat_payload(const uint8_t *data, uint8_t data_len, heartbeat_monitor_t *monitor,
                                             int node_handle, portMUX_TYPE *mirror_lock,
                                             volatile hb_mirror_snapshot_t *mirror, bool planner_node,
                                             const char *tag_rx, const char *label)
{
	if (!data || !monitor || !mirror_lock || !mirror)
		return false;

	node_heartbeat_t hb;
	if (!can_decode_heartbeat(data, data_len, &hb))
		return false;

	heartbeat_monitor_update(monitor, node_handle, hb.sequence, hb.state);

	taskENTER_CRITICAL(mirror_lock);
	if (planner_node)
	{
		mirror->planner_state = hb.state;
		mirror->planner_fault_flags = hb.fault_flags;
		mirror->planner_status_flags = hb.status_flags;
		mirror->planner_stop_flags = hb.stop_flags;
	}
	else
	{
		mirror->control_state = hb.state;
		mirror->control_fault_flags = hb.fault_flags;
		mirror->control_status_flags = hb.status_flags;
		mirror->control_stop_flags = hb.stop_flags;
	}
	taskEXIT_CRITICAL(mirror_lock);

#ifdef CONFIG_LOG_TRANSPORT_CAN_HEARTBEAT_RX
	ESP_LOGI(tag_rx, "%s HB RX: seq=%u state=%s fault=%s stop=%s status_flags=0x%02X", label, hb.sequence,
	         node_state_to_string(hb.state), node_fault_to_string(hb.fault_flags), node_stop_to_string(hb.stop_flags),
	         hb.status_flags);
#else
	(void)tag_rx;
	(void)label;
#endif
	return true;
}
