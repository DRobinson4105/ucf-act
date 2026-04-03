/**
 * @file test_can_protocol.cpp
 * @brief Unit tests for shared CAN protocol definitions and message encoding.
 */

#include "test_harness.h"
#include <string.h>
#include <stdint.h>
#include <limits.h>

#include "can_protocol.h"

// ============================================================================
// LE16 pack/unpack
// ============================================================================

static void test_pack_unpack_le16_zero(void)
{
	uint8_t buf[2] = {0xFF, 0xFF};
	can_pack_le16(buf, 0);
	assert(buf[0] == 0x00);
	assert(buf[1] == 0x00);
	assert(can_unpack_le16(buf) == 0);
}

static void test_pack_unpack_le16_max(void)
{
	uint8_t buf[2];
	can_pack_le16(buf, 0xFFFF);
	assert(buf[0] == 0xFF);
	assert(buf[1] == 0xFF);
	assert(can_unpack_le16(buf) == 0xFFFF);
}

static void test_pack_unpack_le16_value(void)
{
	uint8_t buf[2];
	can_pack_le16(buf, 0x1234);
	assert(buf[0] == 0x34); // LSB first
	assert(buf[1] == 0x12);
	assert(can_unpack_le16(buf) == 0x1234);
}

static void test_pack_unpack_le16s_positive(void)
{
	uint8_t buf[2];
	can_pack_le16s(buf, 1000);
	assert(can_unpack_le16s(buf) == 1000);
}

static void test_pack_unpack_le16s_negative(void)
{
	uint8_t buf[2];
	can_pack_le16s(buf, -1000);
	assert(can_unpack_le16s(buf) == -1000);
}

static void test_pack_unpack_le16s_min(void)
{
	uint8_t buf[2];
	can_pack_le16s(buf, INT16_MIN);
	assert(can_unpack_le16s(buf) == INT16_MIN);
	// INT16_MIN = -32768 = 0x8000 LE: 0x00, 0x80
	assert(buf[0] == 0x00);
	assert(buf[1] == 0x80);
}

static void test_pack_unpack_le16s_max(void)
{
	uint8_t buf[2];
	can_pack_le16s(buf, INT16_MAX);
	assert(can_unpack_le16s(buf) == INT16_MAX);
	// INT16_MAX = 32767 = 0x7FFF LE: 0xFF, 0x7F
	assert(buf[0] == 0xFF);
	assert(buf[1] == 0x7F);
}

// ============================================================================
// Planner command encode/decode
// ============================================================================

static void test_planner_command_roundtrip(void)
{
	planner_command_t cmd_in = {
		.sequence = 42,
		.throttle = 5,
		.steering_position = 720,
		.braking_position = 3,
	};

	uint8_t data[8] = {};
	can_encode_planner_command(data, &cmd_in);

	planner_command_t cmd_out = {};
	assert(can_decode_planner_command(data, 6, &cmd_out));

	assert(cmd_out.throttle == 5);
	assert(cmd_out.steering_position == 720);
	assert(cmd_out.braking_position == 3);
	assert(cmd_out.sequence == 42);
}

static void test_planner_command_zero(void)
{
	planner_command_t cmd_in = {};

	uint8_t data[8];
	memset(data, 0xFF, 8);
	can_encode_planner_command(data, &cmd_in);

	planner_command_t cmd_out = {};
	assert(can_decode_planner_command(data, 8, &cmd_out));

	assert(cmd_out.throttle == 0);
	assert(cmd_out.steering_position == 0);
	assert(cmd_out.braking_position == 0);
	assert(cmd_out.sequence == 0);
}

static void test_planner_command_extremes(void)
{
	planner_command_t cmd_in = {
		.sequence = 255,
		.throttle = 4095,
		.steering_position = 720,
		.braking_position = 255,
	};

	uint8_t data[8];
	can_encode_planner_command(data, &cmd_in);

	planner_command_t cmd_out = {};
	assert(can_decode_planner_command(data, 6, &cmd_out));

	assert(cmd_out.throttle == 4095);
	assert(cmd_out.steering_position == 720);
	assert(cmd_out.braking_position == 255);
	assert(cmd_out.sequence == 255);
}

static void test_planner_command_wire_format(void)
{
	planner_command_t cmd = {.sequence = 10, .throttle = 3, .steering_position = 300, .braking_position = 2};
	uint8_t data[8];
	can_encode_planner_command(data, &cmd);

	// byte 0: sequence
	assert(data[0] == 10);
	// byte 1: throttle MSB (3 = 0x0003 -> MSB 0x00)
	assert(data[1] == 0x00);
	// byte 2: throttle LSB (3 = 0x0003 -> LSB 0x03)
	assert(data[2] == 0x03);
	// byte 3: steering MSB (300 = 0x012C -> MSB 0x01)
	assert(data[3] == 0x01);
	// byte 4: steering LSB (300 = 0x012C -> LSB 0x2C)
	assert(data[4] == 0x2C);
	// byte 5: braking
	assert(data[5] == 2);
	// bytes 6-7: reserved, should be zero
	assert(data[6] == 0);
	assert(data[7] == 0);
}

static void test_planner_command_decode_rejects_short_dlc(void)
{
	uint8_t data[8] = {};
	planner_command_t cmd_out = {
		.sequence = 0xAA,
		.throttle = 0xBB,
		.steering_position = 0x1234,
		.braking_position = 0xCC,
	};

	assert(!can_decode_planner_command(data, 5, &cmd_out));
	assert(cmd_out.sequence == 0xAA);
	assert(cmd_out.throttle == 0xBB);
	assert(cmd_out.steering_position == 0x1234);
	assert(cmd_out.braking_position == 0xCC);
}

static void test_planner_command_decode_full_byte_throttle(void)
{
	uint8_t data[8] = {};
	data[0] = 42;
	// throttle = 4095 = 0x0FFF MSB/LSB
	data[1] = 0x0F;
	data[2] = 0xFF;
	// steering = 100 = 0x0064 MSB/LSB
	data[3] = 0x00;
	data[4] = 0x64;
	// braking = 50
	data[5] = 50;

	planner_command_t cmd_out = {};
	assert(can_decode_planner_command(data, 6, &cmd_out));
	assert(cmd_out.sequence == 42);
	assert(cmd_out.throttle == 4095);
	assert(cmd_out.steering_position == 100);
	assert(cmd_out.braking_position == 50);
}

// ============================================================================
// Safety heartbeat encode/decode
// ============================================================================

static void test_safety_heartbeat_roundtrip_advancing(void)
{
	node_heartbeat_t hb_in = {
		.sequence = 1,
		.state = NODE_STATE_ENABLE,
		.fault_flags = NODE_FAULT_NONE,
		.status_flags = 0,
		.stop_flags = NODE_STOP_NONE,
		.soc_pct = 0,
	};

	uint8_t data[8];
	can_encode_heartbeat(data, &hb_in);

	node_heartbeat_t hb_out = {};
	assert(can_decode_heartbeat(data, 8, &hb_out));

	assert(hb_out.sequence == 1);
	assert(hb_out.state == NODE_STATE_ENABLE);
	assert(hb_out.fault_flags == NODE_FAULT_NONE);
}

static void test_safety_heartbeat_roundtrip_retreating(void)
{
	node_heartbeat_t hb_in = {
		.sequence = 55,
		.state = NODE_STATE_READY,
		.fault_flags = NODE_FAULT_SAFETY_PLANNER_TIMEOUT,
		.status_flags = 0,
		.stop_flags = NODE_STOP_PUSH_BUTTON,
		.soc_pct = 0,
	};

	uint8_t data[8];
	can_encode_heartbeat(data, &hb_in);

	node_heartbeat_t hb_out = {};
	assert(can_decode_heartbeat(data, 8, &hb_out));

	assert(hb_out.sequence == 55);
	assert(hb_out.state == NODE_STATE_READY);
	assert(hb_out.fault_flags == NODE_FAULT_SAFETY_PLANNER_TIMEOUT);
	assert(hb_out.stop_flags == NODE_STOP_PUSH_BUTTON);
}

static void test_safety_heartbeat_reserved_bytes_zero(void)
{
	node_heartbeat_t hb = {
		.sequence = 1,
		.state = NODE_STATE_ACTIVE,
		.fault_flags = NODE_FAULT_NONE,
		.status_flags = 0,
		.stop_flags = NODE_STOP_NONE,
		.soc_pct = 0,
	};
	uint8_t data[8];
	memset(data, 0xFF, 8);
	can_encode_heartbeat(data, &hb);

	// byte 4 = stop_flags, byte 5 = soc_pct, bytes 6-7 reserved
	assert(data[4] == 0);
	assert(data[5] == 0);
	assert(data[6] == 0);
	assert(data[7] == 0);
}

// ============================================================================
// Node heartbeat encode/decode
// ============================================================================

static void test_heartbeat_roundtrip_basic(void)
{
	node_heartbeat_t hb_in = {
		.sequence = 100,
		.state = NODE_STATE_ACTIVE,
		.fault_flags = NODE_FAULT_NONE,
		.status_flags = 0,
		.stop_flags = NODE_STOP_NONE,
		.soc_pct = 72,
	};

	uint8_t data[8];
	can_encode_heartbeat(data, &hb_in);

	node_heartbeat_t hb_out = {};
	assert(can_decode_heartbeat(data, 8, &hb_out));

	assert(hb_out.sequence == 100);
	assert(hb_out.state == NODE_STATE_ACTIVE);
	assert(hb_out.fault_flags == NODE_FAULT_NONE);
	assert(hb_out.status_flags == 0);
	assert(hb_out.soc_pct == 72);
}

static void test_heartbeat_roundtrip_with_fault(void)
{
	node_heartbeat_t hb_in = {
		.sequence = 99,
		.state = NODE_STATE_NOT_READY,
		.fault_flags = NODE_FAULT_PLANNER_PERCEPTION,
		.status_flags = 0,
		.stop_flags = NODE_STOP_NONE,
		.soc_pct = 0,
	};

	uint8_t data[8];
	can_encode_heartbeat(data, &hb_in);

	node_heartbeat_t hb_out = {};
	assert(can_decode_heartbeat(data, 8, &hb_out));

	assert(hb_out.sequence == 99);
	assert(hb_out.state == NODE_STATE_NOT_READY);
	assert(hb_out.fault_flags == NODE_FAULT_PLANNER_PERCEPTION);
	assert(hb_out.status_flags == 0);
}

static void test_heartbeat_roundtrip_enable_complete(void)
{
	node_heartbeat_t hb_in = {
		.sequence = 50,
		.state = NODE_STATE_ENABLE,
		.fault_flags = NODE_FAULT_NONE,
		.status_flags = NODE_STATUS_FLAG_ENABLE_COMPLETE,
		.stop_flags = NODE_STOP_NONE,
		.soc_pct = 0,
	};

	uint8_t data[8];
	can_encode_heartbeat(data, &hb_in);

	node_heartbeat_t hb_out = {};
	assert(can_decode_heartbeat(data, 8, &hb_out));

	assert(hb_out.sequence == 50);
	assert(hb_out.state == NODE_STATE_ENABLE);
	assert(hb_out.fault_flags == NODE_FAULT_NONE);
	assert(hb_out.status_flags == NODE_STATUS_FLAG_ENABLE_COMPLETE);
}

static void test_heartbeat_roundtrip_autonomy_request(void)
{
	node_heartbeat_t hb_in = {
		.sequence = 51,
		.state = NODE_STATE_READY,
		.fault_flags = NODE_FAULT_NONE,
		.status_flags = NODE_STATUS_FLAG_AUTONOMY_REQUEST,
		.stop_flags = NODE_STOP_NONE,
		.soc_pct = 0,
	};

	uint8_t data[8];
	can_encode_heartbeat(data, &hb_in);

	node_heartbeat_t hb_out = {};
	assert(can_decode_heartbeat(data, 8, &hb_out));

	assert(hb_out.sequence == 51);
	assert(hb_out.state == NODE_STATE_READY);
	assert(hb_out.fault_flags == NODE_FAULT_NONE);
	assert(hb_out.status_flags == NODE_STATUS_FLAG_AUTONOMY_REQUEST);
}

static void test_heartbeat_roundtrip_stop_flags(void)
{
	node_heartbeat_t hb_in = {
		.sequence = 53,
		.state = NODE_STATE_ACTIVE,
		.fault_flags = NODE_FAULT_NONE,
		.status_flags = 0,
		.stop_flags = NODE_STOP_OPERATOR_REVERSE,
		.soc_pct = 0,
	};

	uint8_t data[8];
	can_encode_heartbeat(data, &hb_in);

	// Verify byte 4 carries stop_flags
	assert(data[4] == NODE_STOP_OPERATOR_REVERSE);

	node_heartbeat_t hb_out = {};
	assert(can_decode_heartbeat(data, 8, &hb_out));

	assert(hb_out.sequence == 53);
	assert(hb_out.state == NODE_STATE_ACTIVE);
	assert(hb_out.stop_flags == NODE_STOP_OPERATOR_REVERSE);
}

static void test_heartbeat_roundtrip_reserved_bit2(void)
{
	node_heartbeat_t hb_in = {
		.sequence = 52,
		.state = NODE_STATE_READY,
		.fault_flags = NODE_FAULT_NONE,
		.status_flags = 0x04,
		.stop_flags = NODE_STOP_NONE,
		.soc_pct = 0,
	};

	uint8_t data[8];
	can_encode_heartbeat(data, &hb_in);

	node_heartbeat_t hb_out = {};
	assert(can_decode_heartbeat(data, 8, &hb_out));

	assert(hb_out.sequence == 52);
	assert(hb_out.state == NODE_STATE_READY);
	assert(hb_out.fault_flags == NODE_FAULT_NONE);
	assert(hb_out.status_flags == 0x04);
}

static void test_heartbeat_reserved_bytes_zero(void)
{
	node_heartbeat_t hb = {
		.sequence = 1,
		.state = NODE_STATE_READY,
		.fault_flags = NODE_FAULT_NONE,
		.status_flags = 0,
		.stop_flags = NODE_STOP_REMOTE,
		.soc_pct = 0,
	};
	uint8_t data[8];
	memset(data, 0xFF, 8);
	can_encode_heartbeat(data, &hb);

	// byte 4 = stop_flags, byte 5 = soc_pct, bytes 6-7 reserved
	assert(data[4] == NODE_STOP_REMOTE);
	assert(data[5] == 0);
	assert(data[6] == 0);
	assert(data[7] == 0);
}

static void test_heartbeat_decode_rejects_short_dlc(void)
{
	uint8_t data[8] = {};
	node_heartbeat_t hb_out = {
		.sequence = 0xAA,
		.state = 0xBB,
		.fault_flags = 0xCC,
		.status_flags = 0xDD,
		.stop_flags = 0x11,
		.soc_pct = 0x22,
	};

	assert(!can_decode_heartbeat(data, 3, &hb_out));
	assert(!can_decode_heartbeat(data, 5, &hb_out));
	assert(can_decode_heartbeat(data, 6, &hb_out));  // minimum valid DLC
	// Reset hb_out to sentinel values for the remaining checks
	hb_out = {.sequence = 0xAA, .state = 0xBB, .fault_flags = 0xCC, .status_flags = 0xDD, .stop_flags = 0x11, .soc_pct = 0x22};
	assert(!can_decode_heartbeat(data, 0, &hb_out));
	assert(hb_out.sequence == 0xAA);
	assert(hb_out.state == 0xBB);
	assert(hb_out.fault_flags == 0xCC);
	assert(hb_out.status_flags == 0xDD);
	assert(hb_out.stop_flags == 0x11);
}

// ============================================================================
// String helpers
// ============================================================================

static void test_node_state_all_values(void)
{
	assert(strcmp(node_state_to_string(NODE_STATE_INIT), "INIT") == 0);
	assert(strcmp(node_state_to_string(NODE_STATE_NOT_READY), "NOT_READY") == 0);
	assert(strcmp(node_state_to_string(NODE_STATE_READY), "READY") == 0);
	assert(strcmp(node_state_to_string(NODE_STATE_ENABLE), "ENABLE") == 0);
	assert(strcmp(node_state_to_string(NODE_STATE_ACTIVE), "ACTIVE") == 0);
	assert(strcmp(node_state_to_string(0xFF), "UNKNOWN") == 0);
}

static void test_node_fault_all_values(void)
{
	// Common
	assert(strcmp(node_fault_to_string(NODE_FAULT_NONE), "none") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_GENERAL), "general") == 0);
	// Safety fault bitmask flags (single bits)
	assert(strcmp(node_fault_to_string(NODE_FAULT_SAFETY_ULTRASONIC_UNHEALTHY), "ultrasonic_unhealthy") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_SAFETY_PLANNER_ISSUE), "planner_issue") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_SAFETY_PLANNER_TIMEOUT), "planner_timeout") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_SAFETY_CONTROL_ISSUE), "control_issue") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_SAFETY_CONTROL_TIMEOUT), "control_timeout") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_SAFETY_RELAY_UNAVAILABLE), "relay_unavailable") == 0);
	// Safety fault bitmask combinations
	assert(strcmp(node_fault_to_string(NODE_FAULT_SAFETY_PLANNER_TIMEOUT | NODE_FAULT_SAFETY_CONTROL_TIMEOUT),
	              "planner_timeout+control_timeout") == 0);
	// Planner faults
	assert(strcmp(node_fault_to_string(NODE_FAULT_PLANNER_PERCEPTION), "perception") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_PLANNER_LOCALIZATION), "localization") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_PLANNER_PLANNING), "planning") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_PLANNER_HARDWARE), "planner_hardware") == 0);
	// Control faults
	assert(strcmp(node_fault_to_string(NODE_FAULT_CONTROL_THROTTLE_INIT), "throttle_init") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_CONTROL_CAN_TX), "can_tx") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_CONTROL_MOTOR_COMM), "motor_comm") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_CONTROL_SENSOR_INVALID), "sensor_invalid") == 0);
	assert(strcmp(node_fault_to_string(NODE_FAULT_CONTROL_RELAY_INIT), "relay_init") == 0);
	// Unknown
	assert(strcmp(node_fault_to_string(0xFE), "unknown(0xFE)") == 0);
}

static void test_fault_classification_helpers(void)
{
	assert(node_fault_is_planner(NODE_FAULT_PLANNER_PERCEPTION));
	assert(node_fault_is_planner(NODE_FAULT_PLANNER_PERCEPTION | NODE_FAULT_PLANNER_LOCALIZATION));
	assert(!node_fault_is_planner(NODE_FAULT_CONTROL_THROTTLE_INIT));
	assert(!node_fault_is_planner(NODE_FAULT_NONE));

	assert(node_fault_is_control(NODE_FAULT_CONTROL_THROTTLE_INIT));
	assert(node_fault_is_control(NODE_FAULT_CONTROL_MOTOR_COMM | NODE_FAULT_CONTROL_CAN_TX));
	assert(!node_fault_is_control(NODE_FAULT_PLANNER_PERCEPTION));
	assert(!node_fault_is_control(NODE_FAULT_NONE));

	assert(node_stop_has_operator_intervention(NODE_STOP_OPERATOR_THROTTLE));
	assert(node_stop_has_operator_intervention(NODE_STOP_OPERATOR_REVERSE));
	assert(node_stop_has_operator_intervention(NODE_STOP_OPERATOR_STEER));
	assert(node_stop_has_operator_intervention(NODE_STOP_OPERATOR_BRAKE));
	assert(!node_stop_has_operator_intervention(NODE_STOP_PUSH_BUTTON));
	assert(!node_stop_has_operator_intervention(NODE_STOP_NONE));
}

// ============================================================================
// String helper reentrant variants
// ============================================================================

static void test_stop_to_string_r_buffer(void)
{
	char buf[80];
	const char *result = node_stop_to_string_r(NODE_STOP_PUSH_BUTTON | NODE_STOP_REMOTE, buf, sizeof(buf));
	assert(result == buf); // should return the buffer pointer
	assert(strcmp(buf, "push_button+remote") == 0);
}

static void test_stop_to_string_r_none(void)
{
	char buf[80];
	node_stop_to_string_r(NODE_STOP_NONE, buf, sizeof(buf));
	assert(strcmp(buf, "none") == 0);
}

static void test_fault_to_string_r_safety_bitmask(void)
{
	char buf[96];
	const char *result = node_fault_to_string_r(
		NODE_FAULT_SAFETY_PLANNER_TIMEOUT | NODE_FAULT_SAFETY_CONTROL_TIMEOUT, buf, sizeof(buf));
	assert(result == buf);
	assert(strcmp(buf, "planner_timeout+control_timeout") == 0);
}

static void test_fault_to_string_r_planner_bitmask(void)
{
	char buf[96];
	const char *result = node_fault_to_string_r(
		NODE_FAULT_PLANNER_PERCEPTION | NODE_FAULT_PLANNER_LOCALIZATION, buf, sizeof(buf));
	assert(result == buf);
	assert(strcmp(buf, "perception+localization") == 0);
}

static void test_fault_to_string_r_control_bitmask(void)
{
	char buf[96];
	const char *result = node_fault_to_string_r(
		NODE_FAULT_CONTROL_MOTOR_COMM | NODE_FAULT_CONTROL_SENSOR_INVALID, buf, sizeof(buf));
	assert(result == buf);
	assert(strcmp(buf, "motor_comm+sensor_invalid") == 0);
}

// ============================================================================
// Compile-time safety checks
// ============================================================================

static void test_safety_fault_mask_completeness(void)
{
	// NODE_FAULT_SAFETY_ANY must equal the OR of all individual safety fault bits.
	// If a new flag is added without being included in the mask, this test will fail.
	node_fault_t expected = NODE_FAULT_SAFETY_ULTRASONIC_UNHEALTHY | NODE_FAULT_SAFETY_PLANNER_ISSUE |
	                        NODE_FAULT_SAFETY_PLANNER_TIMEOUT | NODE_FAULT_SAFETY_CONTROL_ISSUE |
	                        NODE_FAULT_SAFETY_CONTROL_TIMEOUT | NODE_FAULT_SAFETY_RELAY_UNAVAILABLE;
	assert(NODE_FAULT_SAFETY_ANY == expected);
}

// ============================================================================
// CAN ID constants
// ============================================================================

static void test_can_id_ranges(void)
{
	// Safety IDs in 0x100-0x10F range
	assert(CAN_ID_SAFETY_HEARTBEAT >= 0x100 && CAN_ID_SAFETY_HEARTBEAT <= 0x10F);

	// Planner IDs in 0x110-0x11F range
	assert(CAN_ID_PLANNER_HEARTBEAT >= 0x110 && CAN_ID_PLANNER_HEARTBEAT <= 0x11F);
	assert(CAN_ID_PLANNER_COMMAND >= 0x110 && CAN_ID_PLANNER_COMMAND <= 0x11F);

	// Control IDs in 0x120-0x12F range
	assert(CAN_ID_CONTROL_HEARTBEAT >= 0x120 && CAN_ID_CONTROL_HEARTBEAT <= 0x12F);
}

static void test_no_id_collisions(void)
{
	// All 4 message IDs should be unique
	uint16_t ids[] = {
		CAN_ID_SAFETY_HEARTBEAT,
		CAN_ID_PLANNER_HEARTBEAT,
		CAN_ID_PLANNER_COMMAND,
		CAN_ID_CONTROL_HEARTBEAT,
	};
	int n = sizeof(ids) / sizeof(ids[0]);
	for (int i = 0; i < n; i++)
	{
		for (int j = i + 1; j < n; j++)
		{
			assert(ids[i] != ids[j]);
		}
	}
}

// ============================================================================
// Main
// ============================================================================

int main(void)
{
	printf("can_protocol tests:\n");

	// LE16 pack/unpack (7)
	printf("\n--- LE16 pack/unpack ---\n");
	TEST(test_pack_unpack_le16_zero);
	TEST(test_pack_unpack_le16_max);
	TEST(test_pack_unpack_le16_value);
	TEST(test_pack_unpack_le16s_positive);
	TEST(test_pack_unpack_le16s_negative);
	TEST(test_pack_unpack_le16s_min);
	TEST(test_pack_unpack_le16s_max);

	// Planner command encode/decode (6)
	printf("\n--- Planner command encode/decode ---\n");
	TEST(test_planner_command_roundtrip);
	TEST(test_planner_command_zero);
	TEST(test_planner_command_extremes);
	TEST(test_planner_command_wire_format);
	TEST(test_planner_command_decode_rejects_short_dlc);
	TEST(test_planner_command_decode_full_byte_throttle);

	// Safety heartbeat encode/decode (3)
	printf("\n--- Safety heartbeat encode/decode ---\n");
	TEST(test_safety_heartbeat_roundtrip_advancing);
	TEST(test_safety_heartbeat_roundtrip_retreating);
	TEST(test_safety_heartbeat_reserved_bytes_zero);

	// Node heartbeat encode/decode (6)
	printf("\n--- Heartbeat encode/decode ---\n");
	TEST(test_heartbeat_roundtrip_basic);
	TEST(test_heartbeat_roundtrip_with_fault);
	TEST(test_heartbeat_roundtrip_enable_complete);
	TEST(test_heartbeat_roundtrip_autonomy_request);
	TEST(test_heartbeat_roundtrip_stop_flags);
	TEST(test_heartbeat_roundtrip_reserved_bit2);
	TEST(test_heartbeat_reserved_bytes_zero);
	TEST(test_heartbeat_decode_rejects_short_dlc);

	// String helpers + cause classification (3)
	printf("\n--- String helpers ---\n");
	TEST(test_node_state_all_values);
	TEST(test_node_fault_all_values);
	TEST(test_fault_classification_helpers);

	// String helper reentrant variants (5)
	printf("\n--- String helpers (reentrant) ---\n");
	TEST(test_stop_to_string_r_buffer);
	TEST(test_stop_to_string_r_none);
	TEST(test_fault_to_string_r_safety_bitmask);
	TEST(test_fault_to_string_r_planner_bitmask);
	TEST(test_fault_to_string_r_control_bitmask);

	// Safety checks (1)
	printf("\n--- Compile-time safety checks ---\n");
	TEST(test_safety_fault_mask_completeness);

	// CAN ID constants (2)
	printf("\n--- CAN ID constants ---\n");
	TEST(test_can_id_ranges);
	TEST(test_no_id_collisions);

	TEST_REPORT();
	TEST_EXIT();
}
