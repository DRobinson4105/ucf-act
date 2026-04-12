/**
 * @file test_orin_link_protocol.cpp
 * @brief Unit tests for the Orin UART link message typing helpers.
 */

#include <assert.h>
#include <stdio.h>

#include "orin_link_protocol.h"

#define TEST(fn)                                                                                                         \
	do                                                                                                                   \
	{                                                                                                                    \
		printf("  [TEST] %-60s", #fn);                                                                                   \
		fn();                                                                                                             \
		printf("PASS\n");                                                                                                \
		++tests_passed;                                                                                                   \
	} while (0)

static int tests_passed = 0;

static void test_planner_command_roundtrip(void)
{
	planner_command_t in = {.sequence = 7, .throttle = 1234, .steering_position = 456, .braking_position = 2};
	uint8_t type = 0;
	uint8_t payload[ORIN_LINK_MAX_PAYLOAD_LEN] = {};
	uint8_t payload_len = 0;

	assert(orin_link_encode_planner_command_message(&type, payload, &payload_len, &in));
	assert(type == ORIN_LINK_MSG_PLANNER_COMMAND);
	assert(payload_len == 6);

	planner_command_t out = {};
	assert(orin_link_decode_planner_command_message(type, payload, payload_len, &out));
	assert(out.sequence == in.sequence);
	assert(out.throttle == in.throttle);
	assert(out.steering_position == in.steering_position);
	assert(out.braking_position == in.braking_position);
}

static void test_planner_command_rejects_wrong_type(void)
{
	planner_command_t out = {};
	uint8_t payload[6] = {};
	assert(!orin_link_decode_planner_command_message(ORIN_LINK_MSG_PLANNER_HEARTBEAT, payload, sizeof(payload), &out));
}

static void test_planner_heartbeat_roundtrip(void)
{
	node_heartbeat_t in = {
		.sequence = 9,
		.state = NODE_STATE_READY,
		.fault_flags = NODE_FAULT_NONE,
		.status_flags = NODE_STATUS_FLAG_AUTONOMY_REQUEST,
		.stop_flags = NODE_STOP_NONE,
		.soc_pct = 0,
	};
	uint8_t type = 0;
	uint8_t payload[ORIN_LINK_MAX_PAYLOAD_LEN] = {};
	uint8_t payload_len = 0;

	assert(orin_link_encode_planner_heartbeat_message(&type, payload, &payload_len, &in));
	assert(type == ORIN_LINK_MSG_PLANNER_HEARTBEAT);
	assert(payload_len == 8);

	node_heartbeat_t out = {};
	assert(orin_link_decode_planner_heartbeat_message(type, payload, payload_len, &out));
	assert(out.sequence == in.sequence);
	assert(out.state == in.state);
	assert(out.fault_flags == in.fault_flags);
	assert(out.status_flags == in.status_flags);
	assert(out.stop_flags == in.stop_flags);
}

static void test_heartbeat_types_and_lengths(void)
{
	node_heartbeat_t hb = {.sequence = 1, .state = NODE_STATE_ACTIVE, .fault_flags = NODE_FAULT_NONE, .status_flags = 0,
	                       .stop_flags = NODE_STOP_NONE, .soc_pct = 55};
	uint8_t type = 0;
	uint8_t payload[ORIN_LINK_MAX_PAYLOAD_LEN] = {};
	uint8_t payload_len = 0;

	assert(orin_link_encode_control_heartbeat_message(&type, payload, &payload_len, &hb));
	assert(type == ORIN_LINK_MSG_CONTROL_HEARTBEAT);
	assert(payload_len == 8);

	assert(orin_link_encode_safety_heartbeat_message(&type, payload, &payload_len, &hb));
	assert(type == ORIN_LINK_MSG_SAFETY_HEARTBEAT);
	assert(payload_len == 8);
}

static void test_expected_lengths_and_names(void)
{
	assert(orin_link_expected_payload_len(ORIN_LINK_MSG_PLANNER_COMMAND) == 6);
	assert(orin_link_expected_payload_len(ORIN_LINK_MSG_PLANNER_HEARTBEAT) == 8);
	assert(orin_link_expected_payload_len(ORIN_LINK_MSG_CONTROL_HEARTBEAT) == 8);
	assert(orin_link_expected_payload_len(ORIN_LINK_MSG_SAFETY_HEARTBEAT) == 8);
	assert(orin_link_expected_payload_len(0xFF) == 0);

	assert(orin_link_message_type_to_string(ORIN_LINK_MSG_PLANNER_COMMAND));
	assert(orin_link_message_type_to_string(0xFF));
}

int main(void)
{
	printf("\n=== orin_link_protocol unit tests ===\n\n");

	TEST(test_planner_command_roundtrip);
	TEST(test_planner_command_rejects_wrong_type);
	TEST(test_planner_heartbeat_roundtrip);
	TEST(test_heartbeat_types_and_lengths);
	TEST(test_expected_lengths_and_names);

	printf("\n=== %d / %d tests passed ===\n", tests_passed, 5);
	return 0;
}
