/**
 * @file test_stepper_protocol.c
 * @brief Unit tests for UIM2852 stepper motor CAN protocol encoding and parsing.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <limits.h>

#include "stepper_protocol_uim2852.h"

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
// CAN ID encoding/decoding
// ============================================================================

static void test_can_id_roundtrip_node5(void) {
    uint32_t id = stepper_uim2852_make_can_id(5, 0x20);
    uint8_t producer_id = 0, cw = 0;
    assert(stepper_uim2852_parse_can_id(id, &producer_id, &cw));
    assert(producer_id == 5);
    assert(cw == 0x20);
}

static void test_can_id_roundtrip_node6(void) {
    uint32_t id = stepper_uim2852_make_can_id(6, 0x1E);
    uint8_t producer_id = 0, cw = 0;
    assert(stepper_uim2852_parse_can_id(id, &producer_id, &cw));
    assert(producer_id == 6);
    assert(cw == 0x1E);
}

static void test_can_id_with_ack_bit(void) {
    uint8_t cw_ack = stepper_uim2852_cw_with_ack(0x20);
    assert(cw_ack == 0xA0);
    uint32_t id = stepper_uim2852_make_can_id(5, cw_ack);
    uint8_t producer_id = 0, cw = 0;
    assert(stepper_uim2852_parse_can_id(id, &producer_id, &cw));
    assert(producer_id == 5);
    assert(cw == 0xA0);
    assert(stepper_uim2852_cw_base(cw) == 0x20);
}

static void test_can_id_null_outputs(void) {
    uint32_t id = stepper_uim2852_make_can_id(5, 0x15);
    // Should not crash with NULL outputs
    assert(stepper_uim2852_parse_can_id(id, NULL, NULL));
}

static void test_can_id_invalid_format(void) {
    // ID with bit 8 of SID cleared should fail validation
    // SID bits are [28:18]. Bit 8 of SID = bit 26 of can_id
    uint32_t bad_id = 0x00000001;  // SID = 0, bit 8 not set
    assert(!stepper_uim2852_parse_can_id(bad_id, NULL, NULL));
}

static void test_can_id_valid_node_range(void) {
    // SimpleCAN encoding supports node IDs 0-31 (5 bits in SID + 2 bits in EID
    // for the make direction, but parse only recovers bits within that range).
    // The UIM2852 uses IDs 0-31 in practice (default node=5, steering=5, braking=6).
    for (uint8_t node = 0; node < 32; node++) {
        for (uint8_t cw_in = 0; cw_in < 0x80; cw_in += 0x10) {
            uint32_t id = stepper_uim2852_make_can_id(node, cw_in);
            uint8_t node_out = 0xFF, cw_out = 0xFF;
            bool ok = stepper_uim2852_parse_can_id(id, &node_out, &cw_out);
            assert(ok);
            assert(node_out == node);
            assert(cw_out == cw_in);
        }
    }
}

// ============================================================================
// CW utility functions
// ============================================================================

static void test_cw_ack_bit(void) {
    assert(stepper_uim2852_cw_with_ack(0x15) == 0x95);
    assert(stepper_uim2852_cw_ack_requested(0x95));
    assert(!stepper_uim2852_cw_ack_requested(0x15));
    assert(stepper_uim2852_cw_base(0x95) == 0x15);
    assert(stepper_uim2852_cw_base(0x15) == 0x15);
}

// ============================================================================
// Frame build/parse round-trips
// ============================================================================

static void test_build_mo_enable(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_mo(data, true);
    assert(dl == 1);
    assert(data[0] == 1);
}

static void test_build_mo_disable(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_mo(data, false);
    assert(dl == 1);
    assert(data[0] == 0);
}

static void test_build_bg(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_bg(data);
    assert(dl == 0);
}

static void test_build_st(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_st(data);
    assert(dl == 0);
}

static void test_build_sd(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_sd(data, 200000);
    assert(dl == 4);
    // 200000 = 0x00030D40 LE: 0x40, 0x0D, 0x03, 0x00
    assert(data[0] == 0x40);
    assert(data[1] == 0x0D);
    assert(data[2] == 0x03);
    assert(data[3] == 0x00);
}

static void test_build_pa_positive(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_pa(data, 3200);
    assert(dl == 4);
    // 3200 = 0x0C80 LE: 0x80, 0x0C, 0x00, 0x00
    assert(data[0] == 0x80);
    assert(data[1] == 0x0C);
    assert(data[2] == 0x00);
    assert(data[3] == 0x00);
}

static void test_build_pa_negative(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_pa(data, -1600);
    assert(dl == 4);
    // -1600 = 0xFFFFF9C0 LE: 0xC0, 0xF9, 0xFF, 0xFF
    assert(data[0] == 0xC0);
    assert(data[1] == 0xF9);
    assert(data[2] == 0xFF);
    assert(data[3] == 0xFF);
}

static void test_build_sp(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_sp(data, -5000);
    assert(dl == 4);
}

static void test_build_ms_query(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_ms(data, STEPPER_UIM2852_MS_FLAGS_RELPOS);
    assert(dl == 1);
    assert(data[0] == 0);
    
    dl = stepper_uim2852_build_ms(data, STEPPER_UIM2852_MS_SPEED_ABSPOS);
    assert(dl == 1);
    assert(data[0] == 1);
}

static void test_build_ms_clear(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_ms_clear(data);
    assert(dl == 2);
    assert(data[0] == 0);
    assert(data[1] == 0);
}

static void test_build_pp_query(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_pp_query(data, STEPPER_UIM2852_PP_MICROSTEP);
    assert(dl == 1);
    assert(data[0] == 5);
}

static void test_build_pp_set_roundtrip(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_pp_set(data, 7, 42);
    assert(dl == 5);
    assert(data[0] == 7);
    
    uint8_t idx = 0;
    int32_t val = 0;
    assert(stepper_uim2852_parse_param_response(data, dl, &idx, &val));
    assert(idx == 7);
    assert(val == 42);
}

static void test_build_brake(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_brake(data, true);
    assert(dl == 2);
    assert(data[0] == STEPPER_UIM2852_MT_BRAKE);
    assert(data[1] == 1);
    
    dl = stepper_uim2852_build_brake(data, false);
    assert(data[1] == 0);
}

// ============================================================================
// MS[0] parse
// ============================================================================

static void test_parse_ms0_full(void) {
    // Construct a synthetic MS[0] response
    uint8_t data[8] = {0};
    data[0] = 0;  // index
    data[1] = 0x04 | 0x01;  // driver_on=1, mode=PTP(1)
    data[2] = 0x01 | 0x02;  // stopped=1, in_position=1
    data[3] = 0;  // reserved
    // Relative position = 1000 (0x000003E8 LE)
    data[4] = 0xE8;
    data[5] = 0x03;
    data[6] = 0x00;
    data[7] = 0x00;
    
    stepper_uim2852_status_t status = {0};
    assert(stepper_uim2852_parse_ms0(data, 8, &status));
    assert(status.mode == 1);
    assert(status.driver_on == true);
    assert(status.stopped == true);
    assert(status.in_position == true);
    assert(status.stall_detected == false);
    assert(status.relative_position == 1000);
}

static void test_parse_ms0_negative_position(void) {
    uint8_t data[8] = {0};
    data[0] = 0;
    data[1] = 0x04;  // driver_on
    data[2] = 0x00;
    // Relative position = -500 (0xFFFFFE0C LE)
    data[4] = 0x0C;
    data[5] = 0xFE;
    data[6] = 0xFF;
    data[7] = 0xFF;
    
    stepper_uim2852_status_t status = {0};
    assert(stepper_uim2852_parse_ms0(data, 8, &status));
    assert(status.relative_position == -500);
}

static void test_parse_ms0_too_short(void) {
    uint8_t data[4] = {0};
    stepper_uim2852_status_t status = {0};
    assert(!stepper_uim2852_parse_ms0(data, 4, &status));
}

static void test_parse_ms0_null(void) {
    stepper_uim2852_status_t status = {0};
    assert(!stepper_uim2852_parse_ms0(NULL, 8, &status));
    uint8_t data[8] = {0};
    assert(!stepper_uim2852_parse_ms0(data, 8, NULL));
}

// ============================================================================
// MS[1] parse
// ============================================================================

static void test_parse_ms1_full(void) {
    uint8_t data[8] = {0};
    data[0] = 1;  // index
    // Speed = 1500 in 24-bit LE: d1=0xDC, d2=0x05, d3=0x00
    data[1] = 0xDC;
    data[2] = 0x05;
    data[3] = 0x00;
    // Position = -12800 (0xFFFFCE00 LE)
    data[4] = 0x00;
    data[5] = 0xCE;
    data[6] = 0xFF;
    data[7] = 0xFF;
    
    int32_t speed = 0, pos = 0;
    assert(stepper_uim2852_parse_ms1(data, 8, &speed, &pos));
    assert(speed == 1500);
    assert(pos == -12800);
}

static void test_parse_ms1_negative_speed(void) {
    uint8_t data[8] = {0};
    data[0] = 1;
    // Speed = -200 in 24-bit signed LE: 0xFFFF38 -> d1=0x38, d2=0xFF, d3=0xFF
    data[1] = 0x38;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
    
    int32_t speed = 0;
    assert(stepper_uim2852_parse_ms1(data, 8, &speed, NULL));
    assert(speed == -200);
}

static void test_parse_ms1_null(void) {
    assert(!stepper_uim2852_parse_ms1(NULL, 8, NULL, NULL));
    uint8_t data[4] = {0};
    assert(!stepper_uim2852_parse_ms1(data, 4, NULL, NULL));
}

// ============================================================================
// Notification parse
// ============================================================================

static void test_parse_notification_ptp_complete(void) {
    uint8_t data[8] = {0};
    data[0] = STEPPER_UIM2852_STATUS_PTP_COMPLETE;
    data[1] = 0;
    // Position = 6400
    data[4] = 0x00;
    data[5] = 0x19;
    data[6] = 0x00;
    data[7] = 0x00;
    
    stepper_uim2852_notification_t notif = {0};
    assert(stepper_uim2852_parse_notification(data, 8, &notif));
    assert(!notif.is_alarm);
    assert(notif.type == STEPPER_UIM2852_STATUS_PTP_COMPLETE);
    assert(notif.position == 6400);
}

static void test_parse_notification_alarm_stall(void) {
    uint8_t data[8] = {0};
    data[0] = 0x00;  // alarm indicator
    data[1] = STEPPER_UIM2852_ALARM_STALL;
    
    stepper_uim2852_notification_t notif = {0};
    assert(stepper_uim2852_parse_notification(data, 8, &notif));
    assert(notif.is_alarm);
    assert(notif.type == STEPPER_UIM2852_ALARM_STALL);
}

static void test_parse_notification_too_short(void) {
    uint8_t data[1] = {0};
    stepper_uim2852_notification_t notif = {0};
    assert(!stepper_uim2852_parse_notification(data, 1, &notif));
}

// ============================================================================
// Error parse
// ============================================================================

static void test_parse_error(void) {
    uint8_t data[8] = {0};
    data[0] = 0;  // reserved
    data[1] = STEPPER_UIM2852_ERR_BG_DRIVER_OFF;
    data[2] = STEPPER_UIM2852_CW_BG;
    data[3] = 0;
    
    stepper_uim2852_error_t err = {0};
    assert(stepper_uim2852_parse_error(data, 8, &err));
    assert(err.error_code == STEPPER_UIM2852_ERR_BG_DRIVER_OFF);
    assert(err.related_cw == STEPPER_UIM2852_CW_BG);
    assert(err.subindex == 0);
}

static void test_parse_error_too_short(void) {
    uint8_t data[2] = {0};
    stepper_uim2852_error_t err = {0};
    assert(!stepper_uim2852_parse_error(data, 2, &err));
}

// ============================================================================
// Param response parse
// ============================================================================

static void test_parse_param_response_32bit(void) {
    uint8_t data[8] = {0};
    data[0] = 5;  // index = microstep
    // value = 16 (0x00000010 LE)
    data[1] = 16;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    
    uint8_t idx = 0;
    int32_t val = 0;
    assert(stepper_uim2852_parse_param_response(data, 5, &idx, &val));
    assert(idx == 5);
    assert(val == 16);
}

static void test_parse_param_response_16bit(void) {
    uint8_t data[8] = {0};
    data[0] = 7;
    // value = 300 (0x012C) LE: 0x2C, 0x01
    data[1] = 0x2C;
    data[2] = 0x01;
    
    uint8_t idx = 0;
    int32_t val = 0;
    assert(stepper_uim2852_parse_param_response(data, 3, &idx, &val));
    assert(idx == 7);
    assert(val == 300);
}

static void test_parse_param_response_8bit(void) {
    uint8_t data[8] = {0};
    data[0] = 1;
    data[1] = 42;
    
    uint8_t idx = 0;
    int32_t val = 0;
    assert(stepper_uim2852_parse_param_response(data, 2, &idx, &val));
    assert(idx == 1);
    assert(val == 42);
}

static void test_parse_param_response_negative(void) {
    uint8_t data[8] = {0};
    data[0] = 0;
    // value = -1000 (0xFFFFFC18 LE)
    data[1] = 0x18;
    data[2] = 0xFC;
    data[3] = 0xFF;
    data[4] = 0xFF;
    
    int32_t val = 0;
    assert(stepper_uim2852_parse_param_response(data, 5, NULL, &val));
    assert(val == -1000);
}

static void test_parse_param_too_short(void) {
    uint8_t data[1] = {0};
    assert(!stepper_uim2852_parse_param_response(data, 1, NULL, NULL));
}

// ============================================================================
// Build/parse round-trip for set param commands
// ============================================================================

static void test_ic_set_roundtrip(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_ic_set(data, 6, 1);
    assert(dl == 5);
    
    uint8_t idx;
    int32_t val;
    assert(stepper_uim2852_parse_param_response(data, dl, &idx, &val));
    assert(idx == 6);
    assert(val == 1);
}

static void test_lm_set_roundtrip(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_lm_set(data, 2, 100000);
    assert(dl == 5);
    
    uint8_t idx;
    int32_t val;
    assert(stepper_uim2852_parse_param_response(data, dl, &idx, &val));
    assert(idx == 2);
    assert(val == 100000);
}

static void test_qe_set_roundtrip(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_qe_set(data, 1, -500);
    assert(dl == 5);
    
    uint8_t idx;
    int32_t val;
    assert(stepper_uim2852_parse_param_response(data, dl, &idx, &val));
    assert(idx == 1);
    assert(val == -500);
}

// ============================================================================
// Edge cases
// ============================================================================

static void test_build_pa_zero(void) {
    uint8_t data[8];
    stepper_uim2852_build_pa(data, 0);
    assert(data[0] == 0 && data[1] == 0 && data[2] == 0 && data[3] == 0);
}

static void test_build_pa_int32_min(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_pa(data, INT32_MIN);
    assert(dl == 4);
    // INT32_MIN = 0x80000000 LE: 0x00, 0x00, 0x00, 0x80
    assert(data[0] == 0x00);
    assert(data[1] == 0x00);
    assert(data[2] == 0x00);
    assert(data[3] == 0x80);
}

static void test_build_pa_int32_max(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_pa(data, INT32_MAX);
    assert(dl == 4);
    // INT32_MAX = 0x7FFFFFFF LE: 0xFF, 0xFF, 0xFF, 0x7F
    assert(data[0] == 0xFF);
    assert(data[1] == 0xFF);
    assert(data[2] == 0xFF);
    assert(data[3] == 0x7F);
}

static void test_og_zeroes_rest_of_buffer(void) {
    uint8_t data[8];
    memset(data, 0xAA, 8);
    stepper_uim2852_build_og(data);
    for (int i = 0; i < 8; i++) {
        assert(data[i] == 0);
    }
}

// ============================================================================
// Additional frame builders (AC, DC, JV, PR)
// ============================================================================

static void test_build_ac(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_ac(data, 100000);
    assert(dl == 4);
    // 100000 = 0x000186A0 LE
    assert(data[0] == 0xA0);
    assert(data[1] == 0x86);
    assert(data[2] == 0x01);
    assert(data[3] == 0x00);
}

static void test_build_dc(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_dc(data, 50000);
    assert(dl == 4);
    // 50000 = 0x0000C350 LE
    assert(data[0] == 0x50);
    assert(data[1] == 0xC3);
    assert(data[2] == 0x00);
    assert(data[3] == 0x00);
}

static void test_build_jv(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_jv(data, -3200);
    assert(dl == 4);
    // -3200 = 0xFFFFF380 LE
    assert(data[0] == 0x80);
    assert(data[1] == 0xF3);
    assert(data[2] == 0xFF);
    assert(data[3] == 0xFF);
}

static void test_build_pr_positive(void) {
    uint8_t data[8];
    uint8_t dl = stepper_uim2852_build_pr(data, 1600);
    assert(dl == 4);
    // 1600 = 0x00000640 LE
    assert(data[0] == 0x40);
    assert(data[1] == 0x06);
    assert(data[2] == 0x00);
    assert(data[3] == 0x00);
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    printf("stepper_protocol_uim2852 tests:\n");

    // CAN ID
    TEST(test_can_id_roundtrip_node5);
    TEST(test_can_id_roundtrip_node6);
    TEST(test_can_id_with_ack_bit);
    TEST(test_can_id_null_outputs);
    TEST(test_can_id_invalid_format);
    TEST(test_can_id_valid_node_range);

    // CW utility
    TEST(test_cw_ack_bit);

    // Frame builders
    TEST(test_build_mo_enable);
    TEST(test_build_mo_disable);
    TEST(test_build_bg);
    TEST(test_build_st);
    TEST(test_build_sd);
    TEST(test_build_pa_positive);
    TEST(test_build_pa_negative);
    TEST(test_build_sp);
    TEST(test_build_ms_query);
    TEST(test_build_ms_clear);
    TEST(test_build_pp_query);
    TEST(test_build_pp_set_roundtrip);
    TEST(test_build_brake);
    TEST(test_build_ac);
    TEST(test_build_dc);
    TEST(test_build_jv);
    TEST(test_build_pr_positive);

    // MS[0] parse
    TEST(test_parse_ms0_full);
    TEST(test_parse_ms0_negative_position);
    TEST(test_parse_ms0_too_short);
    TEST(test_parse_ms0_null);

    // MS[1] parse
    TEST(test_parse_ms1_full);
    TEST(test_parse_ms1_negative_speed);
    TEST(test_parse_ms1_null);

    // Notification parse
    TEST(test_parse_notification_ptp_complete);
    TEST(test_parse_notification_alarm_stall);
    TEST(test_parse_notification_too_short);

    // Error parse
    TEST(test_parse_error);
    TEST(test_parse_error_too_short);

    // Param response parse
    TEST(test_parse_param_response_32bit);
    TEST(test_parse_param_response_16bit);
    TEST(test_parse_param_response_8bit);
    TEST(test_parse_param_response_negative);
    TEST(test_parse_param_too_short);

    // Round-trip set param
    TEST(test_ic_set_roundtrip);
    TEST(test_lm_set_roundtrip);
    TEST(test_qe_set_roundtrip);

    // Edge cases
    TEST(test_build_pa_zero);
    TEST(test_build_pa_int32_min);
    TEST(test_build_pa_int32_max);
    TEST(test_og_zeroes_rest_of_buffer);

    printf("\n%d/%d tests passed.\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
