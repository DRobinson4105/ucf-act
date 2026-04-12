/**
 * @file control_can_rx.h
 * @brief Control CAN RX decode and mirror/update helpers.
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
	node_state_t target_state;
	node_fault_t safety_fault_flags;
	node_stop_t safety_stop_flags;
	int16_t throttle_cmd;
	int32_t steering_cmd;
	int32_t braking_cmd;
	node_fault_t motor_fault_code;
	TickType_t last_planner_cmd_tick;
	node_seq_t planner_cmd_last_seq;
	uint8_t planner_cmd_stale_count;
} command_snapshot_t;

command_snapshot_t control_can_rx_snapshot_take_and_clear_motor_fault(portMUX_TYPE *cmd_lock,
                                                                      volatile command_snapshot_t *snapshot);
bool control_can_rx_process_planner_command_payload(const uint8_t *data, uint8_t data_len, portMUX_TYPE *cmd_lock,
                                                    volatile command_snapshot_t *snapshot, const char *tag_rx,
                                                    const char *source_label);
bool control_can_rx_process_motor_message(const twai_message_t *msg, struct motor_uim2852_state_t *braking_state,
                                          struct motor_uim2852_state_t *steering_state, portMUX_TYPE *cmd_lock,
                                          volatile command_snapshot_t *snapshot, const char *tag_rx);
bool control_can_rx_process_safety_heartbeat_payload(const uint8_t *data, uint8_t data_len, portMUX_TYPE *cmd_lock,
                                                     volatile command_snapshot_t *snapshot,
                                                     heartbeat_monitor_t *monitor, int node_safety,
                                                     const char *tag_rx, const char *source_label);
bool control_can_rx_process_safety_heartbeat(const twai_message_t *msg, portMUX_TYPE *cmd_lock,
                                             volatile command_snapshot_t *snapshot, heartbeat_monitor_t *monitor,
                                             int node_safety, const char *tag_rx);

#ifdef __cplusplus
}
#endif
