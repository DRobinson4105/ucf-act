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
// Orin command encode/decode
// ============================================================================

static void test_orin_command_roundtrip(void) {
    orin_command_t cmd_in = {
        .throttle = 5,
        .steering_position = 1500,
        .braking_position = -800,
        .sequence = 42,
    };
    
    uint8_t data[8] = {0};
    can_encode_orin_command(data, &cmd_in);
    
    orin_command_t cmd_out = {0};
    can_decode_orin_command(data, &cmd_out);
    
    assert(cmd_out.throttle == 5);
    assert(cmd_out.steering_position == 1500);
    assert(cmd_out.braking_position == -800);
    assert(cmd_out.sequence == 42);
}

static void test_orin_command_zero(void) {
    orin_command_t cmd_in = {0};
    
    uint8_t data[8];
    memset(data, 0xFF, 8);
    can_encode_orin_command(data, &cmd_in);
    
    orin_command_t cmd_out = {0};
    can_decode_orin_command(data, &cmd_out);
    
    assert(cmd_out.throttle == 0);
    assert(cmd_out.steering_position == 0);
    assert(cmd_out.braking_position == 0);
    assert(cmd_out.sequence == 0);
}

static void test_orin_command_extremes(void) {
    orin_command_t cmd_in = {
        .throttle = 7,
        .steering_position = INT16_MAX,
        .braking_position = INT16_MIN,
        .sequence = 255,
    };
    
    uint8_t data[8];
    can_encode_orin_command(data, &cmd_in);
    
    orin_command_t cmd_out = {0};
    can_decode_orin_command(data, &cmd_out);
    
    assert(cmd_out.throttle == 7);
    assert(cmd_out.steering_position == INT16_MAX);
    assert(cmd_out.braking_position == INT16_MIN);
    assert(cmd_out.sequence == 255);
}

static void test_orin_command_reserved_bytes(void) {
    orin_command_t cmd = {.throttle = 3, .steering_position = 100, .braking_position = -100, .sequence = 10};
    uint8_t data[8];
    can_encode_orin_command(data, &cmd);
    
    // Reserved bytes should be zero
    assert(data[1] == 0);
    assert(data[6] == 0);
}

// ============================================================================
// Safety auto allowed encode/decode
// ============================================================================

static void test_safety_auto_allowed_roundtrip(void) {
    safety_auto_allowed_t msg_in = {
        .allowed = 1,
        .block_reason = AUTO_BLOCKED_REASON_NONE,
        .estop_reason = ESTOP_REASON_NONE,
    };
    
    uint8_t data[8];
    can_encode_safety_auto_allowed(data, &msg_in);
    
    safety_auto_allowed_t msg_out = {0};
    can_decode_safety_auto_allowed(data, &msg_out);
    
    assert(msg_out.allowed == 1);
    assert(msg_out.block_reason == AUTO_BLOCKED_REASON_NONE);
    assert(msg_out.estop_reason == ESTOP_REASON_NONE);
}

static void test_safety_auto_blocked_estop(void) {
    safety_auto_allowed_t msg_in = {
        .allowed = 0,
        .block_reason = AUTO_BLOCKED_REASON_ESTOP,
        .estop_reason = ESTOP_REASON_MUSHROOM,
    };
    
    uint8_t data[8];
    can_encode_safety_auto_allowed(data, &msg_in);
    
    safety_auto_allowed_t msg_out = {0};
    can_decode_safety_auto_allowed(data, &msg_out);
    
    assert(msg_out.allowed == 0);
    assert(msg_out.block_reason == AUTO_BLOCKED_REASON_ESTOP);
    assert(msg_out.estop_reason == ESTOP_REASON_MUSHROOM);
}

static void test_safety_reserved_bytes_zero(void) {
    safety_auto_allowed_t msg = {.allowed = 1, .block_reason = 0, .estop_reason = 0};
    uint8_t data[8];
    memset(data, 0xFF, 8);
    can_encode_safety_auto_allowed(data, &msg);
    
    // bytes 3-7 should be zeroed
    for (int i = 3; i < 8; i++) {
        assert(data[i] == 0);
    }
}

// ============================================================================
// String helpers
// ============================================================================

static void test_estop_reason_strings(void) {
    assert(strcmp(estop_reason_to_string(ESTOP_REASON_NONE), "none") == 0);
    assert(strcmp(estop_reason_to_string(ESTOP_REASON_MUSHROOM), "push_button") == 0);
    assert(strcmp(estop_reason_to_string(ESTOP_REASON_REMOTE), "rf_remote") == 0);
    assert(strcmp(estop_reason_to_string(ESTOP_REASON_ULTRASONIC), "ultrasonic") == 0);
    assert(strcmp(estop_reason_to_string(0xFF), "unknown") == 0);
}

static void test_control_state_strings(void) {
    assert(strcmp(control_state_to_string(CONTROL_STATE_INIT), "INIT") == 0);
    assert(strcmp(control_state_to_string(CONTROL_STATE_READY), "READY") == 0);
    assert(strcmp(control_state_to_string(CONTROL_STATE_ACTIVE), "ACTIVE") == 0);
    assert(strcmp(control_state_to_string(CONTROL_STATE_FAULT), "FAULT") == 0);
    assert(strcmp(control_state_to_string(0xFF), "UNKNOWN") == 0);
}

static void test_override_reason_strings(void) {
    assert(strcmp(override_reason_to_string(OVERRIDE_REASON_NONE), "none") == 0);
    assert(strcmp(override_reason_to_string(OVERRIDE_REASON_PEDAL), "pedal") == 0);
    assert(strcmp(override_reason_to_string(OVERRIDE_REASON_FR_CHANGED), "fr_changed") == 0);
    assert(strcmp(override_reason_to_string(0xFF), "unknown") == 0);
}

static void test_control_fault_strings(void) {
    assert(strcmp(control_fault_to_string(CONTROL_FAULT_NONE), "none") == 0);
    assert(strcmp(control_fault_to_string(CONTROL_FAULT_THROTTLE_INIT), "throttle_init") == 0);
    assert(strcmp(control_fault_to_string(CONTROL_FAULT_CAN_TX), "can_tx") == 0);
    assert(strcmp(control_fault_to_string(CONTROL_FAULT_RELAY_INIT), "relay_init") == 0);
    assert(strcmp(control_fault_to_string(0xFF), "unknown") == 0);
}

// ============================================================================
// CAN ID constants
// ============================================================================

static void test_can_id_ranges(void) {
    // Safety IDs in 0x100-0x10F range
    assert(CAN_ID_SAFETY_AUTO_ALLOWED >= 0x100 && CAN_ID_SAFETY_AUTO_ALLOWED <= 0x10F);
    
    // Orin IDs in 0x110-0x11F range
    assert(CAN_ID_ORIN_HEARTBEAT >= 0x110 && CAN_ID_ORIN_HEARTBEAT <= 0x11F);
    assert(CAN_ID_ORIN_COMMAND >= 0x110 && CAN_ID_ORIN_COMMAND <= 0x11F);
    
    // Control IDs in 0x120-0x12F range
    assert(CAN_ID_CONTROL_HEARTBEAT >= 0x120 && CAN_ID_CONTROL_HEARTBEAT <= 0x12F);
    assert(CAN_ID_CONTROL_STATUS >= 0x120 && CAN_ID_CONTROL_STATUS <= 0x12F);
}

static void test_no_id_collisions(void) {
    // All message IDs should be unique
    uint16_t ids[] = {
        CAN_ID_SAFETY_AUTO_ALLOWED,
        CAN_ID_ORIN_HEARTBEAT,
        CAN_ID_ORIN_COMMAND,
        CAN_ID_CONTROL_HEARTBEAT,
        CAN_ID_CONTROL_STATUS,
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
    
    // LE16
    TEST(test_pack_unpack_le16_zero);
    TEST(test_pack_unpack_le16_max);
    TEST(test_pack_unpack_le16_value);
    TEST(test_pack_unpack_le16s_positive);
    TEST(test_pack_unpack_le16s_negative);
    TEST(test_pack_unpack_le16s_min);
    TEST(test_pack_unpack_le16s_max);
    
    // Orin command
    TEST(test_orin_command_roundtrip);
    TEST(test_orin_command_zero);
    TEST(test_orin_command_extremes);
    TEST(test_orin_command_reserved_bytes);
    
    // Safety auto allowed
    TEST(test_safety_auto_allowed_roundtrip);
    TEST(test_safety_auto_blocked_estop);
    TEST(test_safety_reserved_bytes_zero);
    
    // String helpers
    TEST(test_estop_reason_strings);
    TEST(test_control_state_strings);
    TEST(test_override_reason_strings);
    TEST(test_control_fault_strings);
    
    // Constants
    TEST(test_can_id_ranges);
    TEST(test_no_id_collisions);
    
    printf("\n%d/%d tests passed.\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
