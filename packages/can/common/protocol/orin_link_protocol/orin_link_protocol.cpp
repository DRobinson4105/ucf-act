/**
 * @file orin_link_protocol.cpp
 * @brief Message typing helpers for the Orin <-> ESP UART links (transported via usb_serial_link).
 */

#include "orin_link_protocol.h"

namespace
{

bool encode_heartbeat_message(uint8_t expected_type, uint8_t *type_out, uint8_t *payload_out, uint8_t *payload_len_out,
                              const node_heartbeat_t *hb)
{
	if (!type_out || !payload_out || !payload_len_out || !hb)
		return false;

	*type_out = expected_type;
	*payload_len_out = 8;
	can_encode_heartbeat(payload_out, hb);
	return true;
}

bool decode_heartbeat_message(uint8_t expected_type, uint8_t type, const uint8_t *payload, uint8_t payload_len,
                              node_heartbeat_t *hb)
{
	if (!payload || !hb || type != expected_type)
		return false;
	return can_decode_heartbeat(payload, payload_len, hb);
}

} // namespace

const char *orin_link_message_type_to_string(uint8_t type)
{
	switch (type)
	{
	case ORIN_LINK_MSG_PLANNER_COMMAND:
		return "planner_command";
	case ORIN_LINK_MSG_PLANNER_HEARTBEAT:
		return "planner_heartbeat";
	case ORIN_LINK_MSG_CONTROL_HEARTBEAT:
		return "control_heartbeat";
	case ORIN_LINK_MSG_SAFETY_HEARTBEAT:
		return "safety_heartbeat";
	default:
		return "unknown";
	}
}

uint8_t orin_link_expected_payload_len(uint8_t type)
{
	switch (type)
	{
	case ORIN_LINK_MSG_PLANNER_COMMAND:
		return 6;
	case ORIN_LINK_MSG_PLANNER_HEARTBEAT:
	case ORIN_LINK_MSG_CONTROL_HEARTBEAT:
	case ORIN_LINK_MSG_SAFETY_HEARTBEAT:
		return 8;
	default:
		return 0;
	}
}

bool orin_link_encode_planner_command_message(uint8_t *type_out, uint8_t *payload_out, uint8_t *payload_len_out,
                                              const planner_command_t *cmd)
{
	if (!type_out || !payload_out || !payload_len_out || !cmd)
		return false;

	*type_out = ORIN_LINK_MSG_PLANNER_COMMAND;
	*payload_len_out = 6;
	can_encode_planner_command(payload_out, cmd);
	return true;
}

bool orin_link_decode_planner_command_message(uint8_t type, const uint8_t *payload, uint8_t payload_len,
                                              planner_command_t *cmd)
{
	if (!payload || !cmd || type != ORIN_LINK_MSG_PLANNER_COMMAND)
		return false;
	return can_decode_planner_command(payload, payload_len, cmd);
}

bool orin_link_encode_planner_heartbeat_message(uint8_t *type_out, uint8_t *payload_out, uint8_t *payload_len_out,
                                                const node_heartbeat_t *hb)
{
	return encode_heartbeat_message(ORIN_LINK_MSG_PLANNER_HEARTBEAT, type_out, payload_out, payload_len_out, hb);
}

bool orin_link_decode_planner_heartbeat_message(uint8_t type, const uint8_t *payload, uint8_t payload_len,
                                                node_heartbeat_t *hb)
{
	return decode_heartbeat_message(ORIN_LINK_MSG_PLANNER_HEARTBEAT, type, payload, payload_len, hb);
}

bool orin_link_encode_control_heartbeat_message(uint8_t *type_out, uint8_t *payload_out, uint8_t *payload_len_out,
                                                const node_heartbeat_t *hb)
{
	return encode_heartbeat_message(ORIN_LINK_MSG_CONTROL_HEARTBEAT, type_out, payload_out, payload_len_out, hb);
}

bool orin_link_encode_safety_heartbeat_message(uint8_t *type_out, uint8_t *payload_out, uint8_t *payload_len_out,
                                               const node_heartbeat_t *hb)
{
	return encode_heartbeat_message(ORIN_LINK_MSG_SAFETY_HEARTBEAT, type_out, payload_out, payload_len_out, hb);
}
