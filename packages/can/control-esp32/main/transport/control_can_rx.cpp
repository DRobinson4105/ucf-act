/**
 * @file control_can_rx.cpp
 * @brief Control CAN RX decode and mirror/update helpers.
 */

#include "control_can_rx.h"
#include "control_globals.h"

#include "esp_log.h"
#include "motor_dispatch.h"
#include "motor_protocol.h"

namespace
{

node_state_t sanitize_safety_target_state(node_state_t raw_state)
{
	switch (raw_state)
	{
	case NODE_STATE_NOT_READY:
	case NODE_STATE_READY:
	case NODE_STATE_ENABLE:
	case NODE_STATE_ACTIVE:
		return raw_state;
	default:
		return NODE_STATE_NOT_READY;
	}
}

void mark_motor_fault(portMUX_TYPE *cmd_lock, volatile command_snapshot_t *snapshot)
{
	taskENTER_CRITICAL(cmd_lock);
	snapshot->motor_fault_code = NODE_FAULT_CONTROL_MOTOR_COMM;
	taskEXIT_CRITICAL(cmd_lock);
}

} // namespace

command_snapshot_t control_can_rx_snapshot_take_and_clear_motor_fault(portMUX_TYPE *cmd_lock,
                                                                      volatile command_snapshot_t *shared_snapshot)
{
	command_snapshot_t snapshot = {};
	if (!cmd_lock || !shared_snapshot)
		return snapshot;

	taskENTER_CRITICAL(cmd_lock);
	snapshot.target_state = shared_snapshot->target_state;
	snapshot.safety_fault_flags = shared_snapshot->safety_fault_flags;
	snapshot.safety_stop_flags = shared_snapshot->safety_stop_flags;
	snapshot.throttle_cmd = shared_snapshot->throttle_cmd;
	snapshot.steering_cmd = shared_snapshot->steering_cmd;
	snapshot.braking_cmd = shared_snapshot->braking_cmd;
	snapshot.motor_fault_code = shared_snapshot->motor_fault_code;
	snapshot.last_planner_cmd_tick = shared_snapshot->last_planner_cmd_tick;
	snapshot.planner_cmd_last_seq = shared_snapshot->planner_cmd_last_seq;
	snapshot.planner_cmd_stale_count = shared_snapshot->planner_cmd_stale_count;
	shared_snapshot->motor_fault_code = NODE_FAULT_NONE;
	taskEXIT_CRITICAL(cmd_lock);
	return snapshot;
}

/**
 * @brief Update motor_uim2852_state_t from a parsed motor RX frame under spinlock.
 *
 * For status notifications the UIM2852 SN frame layout is:
 *   data[0] (d0) = status byte 1: bits 0-1 mode, bit 2 driver_on
 *   data[1] (d1) = status byte 2: bit 0 stopped, bit 1 in_position,
 *                   bit 2 pvt_stopped, bit 3 stall, bit 5 lock, bit 7 error
 *   data[2..5]   = position (i32 LE, parsed into notification.current_position)
 */
static void update_motor_state(motor_uim2852_state_t *state, const motor_rx_t *rx)
{
	if (!state || !rx)
		return;

	taskENTER_CRITICAL(&state->lock);
	state->last_response_tick = xTaskGetTickCount();

	if (motor_rx_is_notification(rx) && motor_rx_notification_is_status(rx))
	{
		// d0 = status byte 1 (mode / driver_on)
		uint8_t d0 = 0;
		motor_rx_notification_d0(rx, &d0);
		state->driver_enabled = (d0 & 0x04) != 0;
		state->motion_in_progress = ((d0 & 0x03) != 0) && !((rx->data[1] & 0x01) != 0);

		// d1 = status byte 2 (stopped / in_position / stall / error)
		uint8_t d1 = rx->data[1];
		state->stopped = (d1 & 0x01) != 0;
		state->in_position = (d1 & 0x02) != 0;
		state->stall_detected = (d1 & 0x08) != 0;
		state->error_detected = (d1 & 0x80) != 0;

		// Position from notification payload
		int32_t pos = 0;
		if (motor_rx_notification_current_position(rx, &pos))
			state->absolute_position = pos;
	}
	else if (motor_rx_er_is_thrown_error(rx))
	{
		state->error_detected = true;
	}
	taskEXIT_CRITICAL(&state->lock);
}

static bool check_motor_fault(const motor_uim2852_state_t *state, const motor_rx_t *rx,
                               portMUX_TYPE *cmd_lock, volatile command_snapshot_t *snapshot,
                               const char *tag_rx, const char *label)
{
	bool fault = false;

	taskENTER_CRITICAL(&((motor_uim2852_state_t *)state)->lock);
	fault = state->stall_detected || state->error_detected;
	taskEXIT_CRITICAL(&((motor_uim2852_state_t *)state)->lock);

	if (!fault)
		return false;

	ESP_LOGE(tag_rx, "%s: fault: stall=%d err=%d", label, state->stall_detected, state->error_detected);

	if (motor_rx_er_is_thrown_error(rx))
	{
		uint8_t err_code = 0, related_cw = 0, related_sub = 0;
		motor_rx_error_code(rx, &err_code);
		motor_rx_error_related_cw(rx, &related_cw);
		motor_rx_error_related_subindex(rx, &related_sub);
		ESP_LOGE(tag_rx, "%s: error: code=0x%02X cw=0x%02X sub=%u", label, err_code, related_cw, related_sub);
	}

	mark_motor_fault(cmd_lock, snapshot);
	return true;
}

bool control_can_rx_process_motor_message(const twai_message_t *msg, motor_uim2852_state_t *braking_state,
                                          motor_uim2852_state_t *steering_state, portMUX_TYPE *cmd_lock,
                                          volatile command_snapshot_t *snapshot, const char *tag_rx)
{
	if (!msg || !cmd_lock || !snapshot)
		return false;

	// Feed the dispatch queue so motor_dispatch_exec() can match ACKs
	motor_dispatch_enqueue_rx(msg);

	// Parse the raw frame
	motor_rx_t rx = {};
	if (motor_rx_parse(msg, &rx) != ESP_OK)
		return false;

	// Route by producer_id to the matching motor state
	bool matched = false;

	if (braking_state)
	{
		// Braking motor node ID is MOTOR_NODE_BRAKING (6)
		if (rx.producer_id == MOTOR_NODE_BRAKING)
		{
			matched = true;
			update_motor_state(braking_state, &rx);
			check_motor_fault(braking_state, &rx, cmd_lock, snapshot, tag_rx, "MOTOR_UIM2852_BRAKING");
		}
	}

	if (steering_state)
	{
		// Steering motor node ID is MOTOR_NODE_STEERING (7)
		if (rx.producer_id == MOTOR_NODE_STEERING)
		{
			matched = true;
			update_motor_state(steering_state, &rx);
			check_motor_fault(steering_state, &rx, cmd_lock, snapshot, tag_rx, "MOTOR_UIM2852_STEERING");
		}
	}

	return matched;
}

bool control_can_rx_process_planner_command_payload(const uint8_t *data, uint8_t data_len, portMUX_TYPE *cmd_lock,
                                                    volatile command_snapshot_t *snapshot, const char *tag_rx,
                                                    const char *source_label)
{
	if (!data || !cmd_lock || !snapshot)
		return false;

	planner_command_t cmd;
	if (!can_decode_planner_command(data, data_len, &cmd))
		return false;

	int16_t throttle_level = (int16_t)cmd.throttle;

	taskENTER_CRITICAL(cmd_lock);
	snapshot->last_planner_cmd_tick = xTaskGetTickCount();
	if (cmd.sequence == snapshot->planner_cmd_last_seq)
	{
		if (snapshot->planner_cmd_stale_count < 255)
			snapshot->planner_cmd_stale_count = (uint8_t)(snapshot->planner_cmd_stale_count + 1);
	}
	else
	{
		snapshot->planner_cmd_last_seq = cmd.sequence;
		snapshot->planner_cmd_stale_count = 0;
	}

	snapshot->throttle_cmd = throttle_level;
	snapshot->steering_cmd = ((int32_t)cmd.steering_position - 360) * (3200 * 50) / 360;
	// Braking is a 4-level discrete command (0-3, see PLANNER_BRAKING_MAX_LEVEL
	// in can_protocol.h). Clamp out-of-range values to fully engaged (the
	// fail-safe direction) before translating to raw pulses. Level N maps to
	// position (MAX - N) / MAX * BRAKE_RELEASE_POSITION.
	uint8_t brake_level = (cmd.braking_position > PLANNER_BRAKING_MAX_LEVEL) ? PLANNER_BRAKING_MAX_LEVEL
	                                                                         : cmd.braking_position;
	snapshot->braking_cmd =
	    ((int32_t)(PLANNER_BRAKING_MAX_LEVEL - brake_level) * CONFIG_BRAKE_RELEASE_POSITION) / PLANNER_BRAKING_MAX_LEVEL;
	taskEXIT_CRITICAL(cmd_lock);

#ifdef CONFIG_LOG_TRANSPORT_ORIN_PLANNER_COMMAND_RX
	ESP_LOGI(tag_rx, "%s: seq=%u throttle=%u steering=%u braking=%u", source_label ? source_label : "Planner CMD RX",
	         cmd.sequence, cmd.throttle, cmd.steering_position, cmd.braking_position);
#else
	(void)tag_rx;
	(void)source_label;
#endif
	return true;
}

bool control_can_rx_process_safety_heartbeat(const twai_message_t *msg, portMUX_TYPE *cmd_lock,
                                             volatile command_snapshot_t *snapshot, heartbeat_monitor_t *monitor,
                                             int node_safety, const char *tag_rx)
{
	if (!msg)
		return false;

	return control_can_rx_process_safety_heartbeat_payload(msg->data, msg->data_length_code, cmd_lock, snapshot,
	                                                       monitor, node_safety, tag_rx, "Safety HB RX");
}

bool control_can_rx_process_safety_heartbeat_payload(const uint8_t *data, uint8_t data_len, portMUX_TYPE *cmd_lock,
                                                     volatile command_snapshot_t *snapshot,
                                                     heartbeat_monitor_t *monitor, int node_safety,
                                                     const char *tag_rx, const char *source_label)
{
	if (!data || !cmd_lock || !snapshot || !monitor)
		return false;

	node_heartbeat_t hb;
	if (!can_decode_heartbeat(data, data_len, &hb))
		return false;

	node_state_t new_target = sanitize_safety_target_state(hb.state);

	taskENTER_CRITICAL(cmd_lock);
	snapshot->target_state = new_target;
	snapshot->safety_fault_flags = hb.fault_flags;
	snapshot->safety_stop_flags = hb.stop_flags;
	taskEXIT_CRITICAL(cmd_lock);

	heartbeat_monitor_update(monitor, node_safety, hb.sequence, new_target);

#ifdef CONFIG_LOG_TRANSPORT_CAN_HEARTBEAT_RX
	ESP_LOGI(tag_rx, "%s: seq=%u target=%s fault=%s stop=%s status=0x%02X",
	         source_label ? source_label : "Safety HB RX", hb.sequence, node_state_to_string(new_target),
	         node_fault_to_string(hb.fault_flags), node_stop_to_string(hb.stop_flags), hb.status_flags);
#else
	(void)tag_rx;
	(void)source_label;
#endif
	return true;
}
