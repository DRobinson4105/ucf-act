/**
 * @file safety_can_rx.h
 * @brief Safety CAN RX mirror/update helpers.
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "can_protocol.h"
#include "driver/twai.h"
#include "heartbeat_monitor.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	node_state_t planner_state;
	node_fault_t planner_fault_flags;
	node_status_flags_t planner_status_flags;
	node_stop_t planner_stop_flags;
	node_state_t control_state;
	node_fault_t control_fault_flags;
	node_status_flags_t control_status_flags;
	node_stop_t control_stop_flags;
} hb_mirror_snapshot_t;

hb_mirror_snapshot_t safety_hb_mirror_read(portMUX_TYPE *lock, const volatile hb_mirror_snapshot_t *mirror);
bool safety_can_rx_process_heartbeat_payload(const uint8_t *data, uint8_t data_len, heartbeat_monitor_t *monitor,
                                             int node_handle, portMUX_TYPE *mirror_lock,
                                             volatile hb_mirror_snapshot_t *mirror, bool planner_node,
                                             const char *tag_rx, const char *label);
bool safety_can_rx_process_heartbeat(const twai_message_t *msg, heartbeat_monitor_t *monitor, int node_handle,
                                     portMUX_TYPE *mirror_lock, volatile hb_mirror_snapshot_t *mirror,
                                     bool planner_node, const char *tag_rx, const char *label);

#ifdef __cplusplus
}
#endif
