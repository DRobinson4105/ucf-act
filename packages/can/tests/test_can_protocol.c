/**
 * @file test_can_protocol.c
 * @brief Unit tests for shared CAN protocol definitions and message encoding.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>

#include "can_protocol.hh"

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  [TEST] %-50s ", #name); \
    name(); \
    tests_passed++; \
    printf("PASS\n"); \
} while (0)

// ============================================================================
// LE16 pack/unpack
// ============================================================================

static void test_pack_unpack_le16_zero(void) {
    uint8_t buf[2] = {0xFF, 0xFF};
    can_pack_le16(buf, 0);
    assert(buf[0] == 0x00);
    assert(buf[1] == 0x00);
    assert(can_unpack_le16(buf) == 0);
}

static void test_pack_unpack_le16_max(void) {
    uint8_t buf[2];
    can_pack_le16(buf, 0xFFFF);
    assert(buf[0] == 0xFF);
    assert(buf[1] == 0xFF);
    assert(can_unpack_le16(buf) == 0xFFFF);
}

static void test_pack_unpack_le16_value(void) {
    uint8_t buf[2];
    can_pack_le16(buf, 0x1234);
    assert(buf[0] == 0x34);  // LSB first
    assert(buf[1] == 0x12);
    assert(can_unpack_le16(buf) == 0x1234);
}

static void test_pack_unpack_le16s_positive(void) {
    uint8_t buf[2];
    can_pack_le16s(buf, 1000);
    assert(can_unpack_le16s(buf) == 1000);
}

static void test_pack_unpack_le16s_negative(void) {
    uint8_t buf[2];
    can_pack_le16s(buf, -1000);
    assert(can_unpack_le16s(buf) == -1000);
}

static void test_pack_unpack_le16s_min(void) {
    uint8_t buf[2];
    can_pack_le16s(buf, INT16_MIN);
    assert(can_unpack_le16s(buf) == INT16_MIN);
    // INT16_MIN = -32768 = 0x8000 LE: 0x00, 0x80
    assert(buf[0] == 0x00);
    assert(buf[1] == 0x80);
}

static void test_pack_unpack_le16s_max(void) {
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

static void test_planner_command_roundtrip(void) {
    planner_command_t cmd_in = {
        .throttle = 5,
        .steering_position = 1500,
        .braking_position = -800,
        .sequence = 42,
    };
    
    uint8_t data[8] = {0};
    can_encode_planner_command(data, &cmd_in);
    
    planner_command_t cmd_out = {0};
    can_decode_planner_command(data, &cmd_out);
    
    assert(cmd_out.throttle == 5);
    assert(cmd_out.steering_position == 1500);
    assert(cmd_out.braking_position == -800);
    assert(cmd_out.sequence == 42);
}

static void test_planner_command_zero(void) {
    planner_command_t cmd_in = {0};
    
    uint8_t data[8];
    memset(data, 0xFF, 8);
    can_encode_planner_command(data, &cmd_in);
    
    planner_command_t cmd_out = {0};
    can_decode_planner_command(data, &cmd_out);
    
    assert(cmd_out.throttle == 0);
    assert(cmd_out.steering_position == 0);
    assert(cmd_out.braking_position == 0);
    assert(cmd_out.sequence == 0);
}

static void test_planner_command_extremes(void) {
    planner_command_t cmd_in = {
        .throttle = 7,
        .steering_position = INT16_MAX,
        .braking_position = INT16_MIN,
        .sequence = 255,
    };
    
    uint8_t data[8];
    can_encode_planner_command(data, &cmd_in);
    
    planner_command_t cmd_out = {0};
    can_decode_planner_command(data, &cmd_out);
    
    assert(cmd_out.throttle == 7);
    assert(cmd_out.steering_position == INT16_MAX);
    assert(cmd_out.braking_position == INT16_MIN);
    assert(cmd_out.sequence == 255);
}

static void test_planner_command_wire_format(void) {
    planner_command_t cmd = {.sequence = 10, .throttle = 3, .steering_position = 100, .braking_position = -100};
    uint8_t data[8];
    can_encode_planner_command(data, &cmd);
    
    // byte 0: sequence
    assert(data[0] == 10);
    // byte 1: throttle
    assert(data[1] == 3);
    // bytes 2-3: steering LE16 (100 = 0x0064 -> 0x64, 0x00)
    assert(data[2] == 0x64);
    assert(data[3] == 0x00);
    // bytes 4-5: braking LE16 (-100 = 0xFF9C -> 0x9C, 0xFF)
    assert(data[4] == 0x9C);
    assert(data[5] == 0xFF);
    // bytes 6-7: reserved, should be zero
    assert(data[6] == 0);
    assert(data[7] == 0);
}

// ============================================================================
// Safety heartbeat encode/decode
// ============================================================================

static void test_safety_heartbeat_roundtrip_advancing(void) {
    node_heartbeat_t hb_in = {
        .sequence = 1,
        .state = NODE_STATE_ENABLING,
        .fault_code = NODE_FAULT_NONE,
        .flags = 0,
    };
    
    uint8_t data[8];
    can_encode_heartbeat(data, &hb_in);
    
    node_heartbeat_t hb_out = {0};
    can_decode_heartbeat(data, &hb_out);
    
    assert(hb_out.sequence == 1);
    assert(hb_out.state == NODE_STATE_ENABLING);
    assert(hb_out.fault_code == NODE_FAULT_NONE);
}

static void test_safety_heartbeat_roundtrip_retreating(void) {
    node_heartbeat_t hb_in = {
        .sequence = 55,
        .state = NODE_STATE_READY,
        .fault_code = NODE_FAULT_ESTOP_BUTTON,
        .flags = 0,
    };
    
    uint8_t data[8];
    can_encode_heartbeat(data, &hb_in);
    
    node_heartbeat_t hb_out = {0};
    can_decode_heartbeat(data, &hb_out);
    
    assert(hb_out.sequence == 55);
    assert(hb_out.state == NODE_STATE_READY);
    assert(hb_out.fault_code == NODE_FAULT_ESTOP_BUTTON);
}

static void test_safety_heartbeat_reserved_bytes_zero(void) {
    node_heartbeat_t hb = {
        .sequence = 1,
        .state = NODE_STATE_ACTIVE,
        .fault_code = NODE_FAULT_NONE,
        .flags = 0,
    };
    uint8_t data[8];
    memset(data, 0xFF, 8);
    can_encode_heartbeat(data, &hb);
    
    // bytes 4-7 should be zeroed
    for (int i = 4; i < 8; i++) {
        assert(data[i] == 0);
    }
}

// ============================================================================
// Node heartbeat encode/decode
// ============================================================================

static void test_heartbeat_roundtrip_basic(void) {
    node_heartbeat_t hb_in = {
        .sequence = 100,
        .state = NODE_STATE_ACTIVE,
        .fault_code = NODE_FAULT_NONE,
        .flags = 0,
    };
    
    uint8_t data[8];
    can_encode_heartbeat(data, &hb_in);
    
    node_heartbeat_t hb_out = {0};
    can_decode_heartbeat(data, &hb_out);
    
    assert(hb_out.sequence == 100);
    assert(hb_out.state == NODE_STATE_ACTIVE);
    assert(hb_out.fault_code == NODE_FAULT_NONE);
    assert(hb_out.flags == 0);
}

static void test_heartbeat_roundtrip_with_fault(void) {
    node_heartbeat_t hb_in = {
        .sequence = 99,
        .state = NODE_STATE_FAULT,
        .fault_code = NODE_FAULT_PERCEPTION,
        .flags = 0,
    };
    
    uint8_t data[8];
    can_encode_heartbeat(data, &hb_in);
    
    node_heartbeat_t hb_out = {0};
    can_decode_heartbeat(data, &hb_out);
    
    assert(hb_out.sequence == 99);
    assert(hb_out.state == NODE_STATE_FAULT);
    assert(hb_out.fault_code == NODE_FAULT_PERCEPTION);
    assert(hb_out.flags == 0);
}

static void test_heartbeat_roundtrip_enable_complete(void) {
    node_heartbeat_t hb_in = {
        .sequence = 50,
        .state = NODE_STATE_ENABLING,
        .fault_code = NODE_FAULT_NONE,
        .flags = HEARTBEAT_FLAG_ENABLE_COMPLETE,
    };
    
    uint8_t data[8];
    can_encode_heartbeat(data, &hb_in);
    
    node_heartbeat_t hb_out = {0};
    can_decode_heartbeat(data, &hb_out);
    
    assert(hb_out.sequence == 50);
    assert(hb_out.state == NODE_STATE_ENABLING);
    assert(hb_out.fault_code == NODE_FAULT_NONE);
    assert(hb_out.flags == HEARTBEAT_FLAG_ENABLE_COMPLETE);
}

static void test_heartbeat_roundtrip_autonomy_request(void) {
    node_heartbeat_t hb_in = {
        .sequence = 51,
        .state = NODE_STATE_READY,
        .fault_code = NODE_FAULT_NONE,
        .flags = HEARTBEAT_FLAG_AUTONOMY_REQUEST,
    };

    uint8_t data[8];
    can_encode_heartbeat(data, &hb_in);

    node_heartbeat_t hb_out = {0};
    can_decode_heartbeat(data, &hb_out);

    assert(hb_out.sequence == 51);
    assert(hb_out.state == NODE_STATE_READY);
    assert(hb_out.fault_code == NODE_FAULT_NONE);
    assert(hb_out.flags == HEARTBEAT_FLAG_AUTONOMY_REQUEST);
}

static void test_heartbeat_reserved_bytes_zero(void) {
    node_heartbeat_t hb = {
        .sequence = 1,
        .state = NODE_STATE_READY,
        .fault_code = NODE_FAULT_NONE,
        .flags = 0,
    };
    uint8_t data[8];
    memset(data, 0xFF, 8);
    can_encode_heartbeat(data, &hb);
    
    // bytes 4-7 should be zeroed
    for (int i = 4; i < 8; i++) {
        assert(data[i] == 0);
    }
}

// ============================================================================
// String helpers
// ============================================================================

static void test_node_state_all_values(void) {
    assert(strcmp(node_state_to_string(NODE_STATE_INIT), "INIT") == 0);
    assert(strcmp(node_state_to_string(NODE_STATE_READY), "READY") == 0);
    assert(strcmp(node_state_to_string(NODE_STATE_ENABLING), "ENABLING") == 0);
    assert(strcmp(node_state_to_string(NODE_STATE_ACTIVE), "ACTIVE") == 0);
    assert(strcmp(node_state_to_string(NODE_STATE_OVERRIDE), "OVERRIDE") == 0);
    assert(strcmp(node_state_to_string(NODE_STATE_FAULT), "FAULT") == 0);
    assert(strcmp(node_state_to_string(0xFF), "UNKNOWN") == 0);
}

static void test_node_fault_all_values(void) {
    // Common
    assert(strcmp(node_fault_to_string(NODE_FAULT_NONE), "none") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_GENERAL), "general") == 0);
    // System / Safety e-stop causes (bitmask — single bits)
    assert(strcmp(node_fault_to_string(NODE_FAULT_ESTOP_BUTTON), "button") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_ESTOP_REMOTE), "remote") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_ESTOP_ULTRASONIC), "ultrasonic") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_ESTOP_PLANNER), "planner") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_ESTOP_PLANNER_TIMEOUT), "planner_timeout") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_ESTOP_CONTROL), "control") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_ESTOP_CONTROL_TIMEOUT), "control_timeout") == 0);
    // Estop bitmask combinations
    assert(strcmp(node_fault_to_string(NODE_FAULT_ESTOP_BUTTON | NODE_FAULT_ESTOP_REMOTE), "button+remote") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_ESTOP_BUTTON | NODE_FAULT_ESTOP_REMOTE | NODE_FAULT_ESTOP_ULTRASONIC),
                 "button+remote+ultrasonic") == 0);
    // Planner faults
    assert(strcmp(node_fault_to_string(NODE_FAULT_PERCEPTION), "perception") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_LOCALIZATION), "localization") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_PLANNING), "planning") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_PLANNER_HARDWARE), "planner_hardware") == 0);
    // Control faults
    assert(strcmp(node_fault_to_string(NODE_FAULT_THROTTLE_INIT), "throttle_init") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_CAN_TX), "can_tx") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_MOTOR_COMM), "motor_comm") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_SENSOR_INVALID), "sensor_invalid") == 0);
    assert(strcmp(node_fault_to_string(NODE_FAULT_RELAY_INIT), "relay_init") == 0);
    // Unknown
    assert(strcmp(node_fault_to_string(0xFE), "unknown") == 0);
}

// ============================================================================
// CAN ID constants
// ============================================================================

static void test_can_id_ranges(void) {
    // Safety IDs in 0x100-0x10F range
    assert(CAN_ID_SAFETY_HEARTBEAT >= 0x100 && CAN_ID_SAFETY_HEARTBEAT <= 0x10F);
    
    // Planner IDs in 0x110-0x11F range
    assert(CAN_ID_PLANNER_HEARTBEAT >= 0x110 && CAN_ID_PLANNER_HEARTBEAT <= 0x11F);
    assert(CAN_ID_PLANNER_COMMAND >= 0x110 && CAN_ID_PLANNER_COMMAND <= 0x11F);
    
    // Control IDs in 0x120-0x12F range
    assert(CAN_ID_CONTROL_HEARTBEAT >= 0x120 && CAN_ID_CONTROL_HEARTBEAT <= 0x12F);
}

static void test_no_id_collisions(void) {
    // All 4 message IDs should be unique
    uint16_t ids[] = {
        CAN_ID_SAFETY_HEARTBEAT,
        CAN_ID_PLANNER_HEARTBEAT,
        CAN_ID_PLANNER_COMMAND,
        CAN_ID_CONTROL_HEARTBEAT,
    };
    int n = sizeof(ids) / sizeof(ids[0]);
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            assert(ids[i] != ids[j]);
        }
    }
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
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
    
    // Planner command encode/decode (4)
    printf("\n--- Planner command encode/decode ---\n");
    TEST(test_planner_command_roundtrip);
    TEST(test_planner_command_zero);
    TEST(test_planner_command_extremes);
    TEST(test_planner_command_wire_format);
    
    // Safety heartbeat encode/decode (3)
    printf("\n--- Safety heartbeat encode/decode ---\n");
    TEST(test_safety_heartbeat_roundtrip_advancing);
    TEST(test_safety_heartbeat_roundtrip_retreating);
    TEST(test_safety_heartbeat_reserved_bytes_zero);
    
    // Node heartbeat encode/decode (5)
    printf("\n--- Heartbeat encode/decode ---\n");
    TEST(test_heartbeat_roundtrip_basic);
    TEST(test_heartbeat_roundtrip_with_fault);
    TEST(test_heartbeat_roundtrip_enable_complete);
    TEST(test_heartbeat_roundtrip_autonomy_request);
    TEST(test_heartbeat_reserved_bytes_zero);
    
    // String helpers (2)
    printf("\n--- String helpers ---\n");
    TEST(test_node_state_all_values);
    TEST(test_node_fault_all_values);
    
    // CAN ID constants (2)
    printf("\n--- CAN ID constants ---\n");
    TEST(test_can_id_ranges);
    TEST(test_no_id_collisions);
    
    printf("\n%d/%d tests passed.\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
