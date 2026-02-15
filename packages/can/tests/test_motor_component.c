/**
 * @file test_motor_component.c
 * @brief Unit tests for stepper_motor_uim2852 motor component.
 *
 * Compiled with host gcc against mock ESP-IDF infrastructure.
 * Tests the high-level motor control API including init, enable/disable,
 * motion control, status queries, CAN frame processing, and parameter access.
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "esp_idf_mock.h"
#include "stepper_motor_uim2852.h"

// ============================================================================
// Test harness
// ============================================================================

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  [TEST] %-55s ", #name); \
    name(); \
    tests_passed++; \
    printf("PASS\n"); \
} while (0)

// ============================================================================
// Helpers
// ============================================================================

/**
 * Initialize a motor with defaults for use in tests.
 * Returns ESP_OK and the motor is ready to use.
 */
static esp_err_t init_default_motor(stepper_motor_uim2852_t *motor) {
    mock_reset_all();
    return stepper_motor_uim2852_init(motor, NULL);
}

/**
 * Build a mock twai_message_t as if it came from motor node_id with given cw.
 * The motor responds using its own node_id as the producer and echoes the CW
 * (with ACK bit if it was requested).
 */
static twai_message_t make_motor_response(uint8_t node_id, uint8_t cw,
                                           const uint8_t *data, uint8_t dl) {
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.extd = 1;
    msg.identifier = stepper_uim2852_make_can_id(node_id, cw);
    msg.data_length_code = dl;
    if (data && dl > 0) memcpy(msg.data, data, dl > 8 ? 8 : dl);
    return msg;
}

/**
 * Extract the base CW from a sent frame's CAN ID.
 */
static uint8_t get_sent_cw_base(int frame_index) {
    uint8_t producer_id = 0, cw = 0;
    stepper_uim2852_parse_can_id(mock_sent_frames[frame_index].id, &producer_id, &cw);
    return stepper_uim2852_cw_base(cw);
}

// Callback tracking for notification tests
static int callback_call_count = 0;
static stepper_uim2852_notification_t last_callback_notif;

static void test_notify_callback(stepper_motor_uim2852_t *motor,
                                  const stepper_uim2852_notification_t *notif) {
    (void)motor;
    callback_call_count++;
    if (notif) last_callback_notif = *notif;
}

// ============================================================================
// Init tests
// ============================================================================

static void test_init_with_defaults(void) {
    mock_reset_all();
    stepper_motor_uim2852_t motor;
    esp_err_t err = stepper_motor_uim2852_init(&motor, NULL);
    assert(err == ESP_OK);
    assert(motor.initialized == true);
    assert(motor.config.node_id == 5);
    assert(motor.config.producer_id == 4);
    assert(motor.config.request_ack == true);
    assert(motor.config.default_speed == 3200);
    assert(motor.config.default_accel == 50000);
    assert(motor.config.default_decel == 50000);
    assert(motor.config.stop_decel == 200000);
    assert(motor.microstep_resolution == 16);
    assert(motor.pulses_per_rev == 3200);
}

static void test_init_with_custom_config(void) {
    mock_reset_all();
    stepper_motor_uim2852_config_t cfg = {
        .node_id = 7,
        .producer_id = 3,
        .default_speed = 6400,
        .default_accel = 100000,
        .default_decel = 80000,
        .stop_decel = 500000,
        .request_ack = false,
    };
    stepper_motor_uim2852_t motor;
    esp_err_t err = stepper_motor_uim2852_init(&motor, &cfg);
    assert(err == ESP_OK);
    assert(motor.initialized == true);
    assert(motor.config.node_id == 7);
    assert(motor.config.producer_id == 3);
    assert(motor.config.default_speed == 6400);
    assert(motor.config.default_accel == 100000);
    assert(motor.config.default_decel == 80000);
    assert(motor.config.stop_decel == 500000);
    assert(motor.config.request_ack == false);
}

static void test_init_semaphore_fail(void) {
    mock_reset_all();
    mock_sem_create_fail = 1;
    stepper_motor_uim2852_t motor;
    esp_err_t err = stepper_motor_uim2852_init(&motor, NULL);
    assert(err == ESP_ERR_NO_MEM);
    assert(motor.initialized == false);
    assert(motor.query_sem == NULL);
}

// ============================================================================
// Enable/disable tests
// ============================================================================

static void test_enable_sends_mo_frame(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_enable(&motor);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_MO);
    assert(mock_sent_frames[0].data[0] == 1);
    assert(motor.driver_enabled == true);
}

static void test_disable_sends_mo_frame(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.driver_enabled = true;
    motor.motion_in_progress = true;

    esp_err_t err = stepper_motor_uim2852_disable(&motor);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_MO);
    assert(mock_sent_frames[0].data[0] == 0);
    assert(motor.driver_enabled == false);
    assert(motor.motion_in_progress == false);
}

static void test_enable_tx_failure(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    mock_twai_send_result = ESP_FAIL;

    bool was_enabled = motor.driver_enabled;
    esp_err_t err = stepper_motor_uim2852_enable(&motor);
    assert(err == ESP_FAIL);
    assert(motor.driver_enabled == was_enabled);
}

static void test_enable_uninitialized(void) {
    mock_reset_all();
    stepper_motor_uim2852_t motor;
    memset(&motor, 0, sizeof(motor));
    motor.initialized = false;

    esp_err_t err = stepper_motor_uim2852_enable(&motor);
    assert(err == ESP_ERR_INVALID_STATE);
}

// ============================================================================
// Stop tests
// ============================================================================

static void test_stop_sends_st(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.motion_in_progress = true;

    esp_err_t err = stepper_motor_uim2852_stop(&motor);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_ST);
    assert(motor.motion_in_progress == false);
}

static void test_emergency_stop_sends_sd_then_st(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.motion_in_progress = true;

    esp_err_t err = stepper_motor_uim2852_emergency_stop(&motor);
    assert(err == ESP_OK);
    assert(mock_sent_count == 2);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_SD);
    assert(get_sent_cw_base(1) == STEPPER_UIM2852_CW_ST);
    assert(motor.motion_in_progress == false);
}

// ============================================================================
// go_absolute tests
// ============================================================================

static void test_go_absolute_sends_pa_then_bg(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_go_absolute(&motor, 3200);
    assert(err == ESP_OK);
    assert(mock_sent_count == 2);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_PA);
    assert(get_sent_cw_base(1) == STEPPER_UIM2852_CW_BG);
    assert(motor.motion_in_progress == true);

    // Verify PA data: 3200 = 0x00000C80 LE
    assert(mock_sent_frames[0].data[0] == 0x80);
    assert(mock_sent_frames[0].data[1] == 0x0C);
    assert(mock_sent_frames[0].data[2] == 0x00);
    assert(mock_sent_frames[0].data[3] == 0x00);
}

static void test_go_absolute_negative(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_go_absolute(&motor, -1600);
    assert(err == ESP_OK);
    assert(mock_sent_count == 2);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_PA);

    // -1600 = 0xFFFFF9C0 LE
    assert(mock_sent_frames[0].data[0] == 0xC0);
    assert(mock_sent_frames[0].data[1] == 0xF9);
    assert(mock_sent_frames[0].data[2] == 0xFF);
    assert(mock_sent_frames[0].data[3] == 0xFF);
}

static void test_go_absolute_pa_failure(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    mock_twai_send_result = ESP_FAIL;

    esp_err_t err = stepper_motor_uim2852_go_absolute(&motor, 3200);
    assert(err == ESP_FAIL);
    assert(mock_sent_count == 0);
    assert(motor.motion_in_progress == false);
}

static void test_go_absolute_bg_failure(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    // Fail on the 2nd call to can_twai_send_extended (the BG send)
    g_mock_send_ext_fail_after = 2;
    esp_err_t err = stepper_motor_uim2852_go_absolute(&motor, 6400);
    assert(err == ESP_FAIL);
    assert(mock_sent_count == 1);  // only PA succeeded
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_PA);
    assert(motor.motion_in_progress == false);
    g_mock_send_ext_fail_after = 0;
}

// ============================================================================
// go_relative tests
// ============================================================================

static void test_go_relative_sends_pr_then_bg(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_go_relative(&motor, 800);
    assert(err == ESP_OK);
    assert(mock_sent_count == 2);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_PR);
    assert(get_sent_cw_base(1) == STEPPER_UIM2852_CW_BG);

    // 800 = 0x00000320 LE
    assert(mock_sent_frames[0].data[0] == 0x20);
    assert(mock_sent_frames[0].data[1] == 0x03);
    assert(mock_sent_frames[0].data[2] == 0x00);
    assert(mock_sent_frames[0].data[3] == 0x00);
}

static void test_go_relative_negative(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_go_relative(&motor, -400);
    assert(err == ESP_OK);
    assert(mock_sent_count == 2);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_PR);

    // -400 = 0xFFFFFE70 LE
    assert(mock_sent_frames[0].data[0] == 0x70);
    assert(mock_sent_frames[0].data[1] == 0xFE);
    assert(mock_sent_frames[0].data[2] == 0xFF);
    assert(mock_sent_frames[0].data[3] == 0xFF);
}

static void test_go_relative_motion_flag(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    assert(motor.motion_in_progress == false);

    esp_err_t err = stepper_motor_uim2852_go_relative(&motor, 100);
    assert(err == ESP_OK);
    assert(motor.motion_in_progress == true);
}

// ============================================================================
// set_origin tests
// ============================================================================

static void test_set_origin_sends_og(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_set_origin(&motor);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_OG);
    assert(motor.absolute_position == 0);
}

static void test_set_origin_clears_position(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.absolute_position = 5000;

    esp_err_t err = stepper_motor_uim2852_set_origin(&motor);
    assert(err == ESP_OK);
    assert(motor.absolute_position == 0);
}

// ============================================================================
// set_speed / set_accel / set_decel tests
// ============================================================================

static void test_set_speed(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_set_speed(&motor, 5000);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_SP);

    // 5000 = 0x00001388 LE
    assert(mock_sent_frames[0].data[0] == 0x88);
    assert(mock_sent_frames[0].data[1] == 0x13);
    assert(mock_sent_frames[0].data[2] == 0x00);
    assert(mock_sent_frames[0].data[3] == 0x00);
}

static void test_set_accel(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_set_accel(&motor, 100000);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_AC);

    // 100000 = 0x000186A0 LE
    assert(mock_sent_frames[0].data[0] == 0xA0);
    assert(mock_sent_frames[0].data[1] == 0x86);
    assert(mock_sent_frames[0].data[2] == 0x01);
    assert(mock_sent_frames[0].data[3] == 0x00);
}

static void test_set_decel(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_set_decel(&motor, 100000);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_DC);

    // 100000 = 0x000186A0 LE
    assert(mock_sent_frames[0].data[0] == 0xA0);
    assert(mock_sent_frames[0].data[1] == 0x86);
    assert(mock_sent_frames[0].data[2] == 0x01);
    assert(mock_sent_frames[0].data[3] == 0x00);
}

// ============================================================================
// query_status / query_position / clear_status tests
// ============================================================================

static void test_query_status(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_query_status(&motor);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_MS);
    assert(mock_sent_frames[0].data[0] == STEPPER_UIM2852_MS_FLAGS_RELPOS);
}

static void test_query_position(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_query_position(&motor);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_MS);
    assert(mock_sent_frames[0].data[0] == STEPPER_UIM2852_MS_SPEED_ABSPOS);
}

static void test_clear_status(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_clear_status(&motor);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_MS);
    // MS clear sends index=0 with value=0 (dl=2)
    assert(mock_sent_frames[0].data[0] == 0);
    assert(mock_sent_frames[0].data[1] == 0);
    assert(mock_sent_frames[0].len == 2);
}

// ============================================================================
// process_frame — filtering tests
// ============================================================================

static void test_process_frame_standard_frame(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.extd = 0;  // standard frame, not extended
    assert(stepper_motor_uim2852_process_frame(&motor, &msg) == false);
}

static void test_process_frame_wrong_node(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    // Motor is node 5. Build a response from node 6.
    uint8_t data[8] = {0};
    twai_message_t msg = make_motor_response(6, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MS), data, 8);
    assert(stepper_motor_uim2852_process_frame(&motor, &msg) == false);
}

// ============================================================================
// process_frame — MS[0] tests
// ============================================================================

static void test_process_ms0_updates_status(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    uint8_t data[8] = {0};
    data[0] = STEPPER_UIM2852_MS_FLAGS_RELPOS;  // index = 0
    data[1] = 0x04 | 0x01;  // driver_on=1, mode=PTP(1)
    data[2] = 0x01 | 0x02;  // stopped=1, in_position=1
    data[3] = 0;
    // relative position = 1000 LE
    data[4] = 0xE8; data[5] = 0x03; data[6] = 0x00; data[7] = 0x00;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MS), data, 8);
    bool processed = stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(processed == true);
    assert(motor.status.driver_on == true);
    assert(motor.status.stopped == true);
    assert(motor.status.in_position == true);
    assert(motor.status.relative_position == 1000);
}

static void test_process_ms0_clears_motion_on_inposition(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.motion_in_progress = true;

    uint8_t data[8] = {0};
    data[0] = STEPPER_UIM2852_MS_FLAGS_RELPOS;
    data[1] = 0x04;  // driver_on=1
    data[2] = 0x02;  // in_position=1
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MS), data, 8);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(motor.motion_in_progress == false);
}

static void test_process_ms0_clears_motion_on_stopped(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.motion_in_progress = true;

    uint8_t data[8] = {0};
    data[0] = STEPPER_UIM2852_MS_FLAGS_RELPOS;
    data[1] = 0x04;  // driver_on=1
    data[2] = 0x01;  // stopped=1
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MS), data, 8);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(motor.motion_in_progress == false);
}

static void test_process_ms0_updates_driver_enabled(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.driver_enabled = true;

    uint8_t data[8] = {0};
    data[0] = STEPPER_UIM2852_MS_FLAGS_RELPOS;
    data[1] = 0x00;  // driver_on=0
    data[2] = 0x01;  // stopped=1
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MS), data, 8);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(motor.driver_enabled == false);
}

static void test_process_ms0_negative_position(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    uint8_t data[8] = {0};
    data[0] = STEPPER_UIM2852_MS_FLAGS_RELPOS;
    data[1] = 0x04;  // driver_on
    data[2] = 0x00;
    // relative position = -500 = 0xFFFFFE0C LE
    data[4] = 0x0C; data[5] = 0xFE; data[6] = 0xFF; data[7] = 0xFF;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MS), data, 8);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(motor.status.relative_position == -500);
}

// ============================================================================
// process_frame — MS[1] tests
// ============================================================================

static void test_process_ms1_updates_speed_and_position(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    uint8_t data[8] = {0};
    data[0] = STEPPER_UIM2852_MS_SPEED_ABSPOS;  // index = 1
    // speed = 1500 in 24-bit LE: 0xDC, 0x05, 0x00
    data[1] = 0xDC; data[2] = 0x05; data[3] = 0x00;
    // position = -12800 = 0xFFFFCE00 LE
    data[4] = 0x00; data[5] = 0xCE; data[6] = 0xFF; data[7] = 0xFF;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MS), data, 8);
    bool processed = stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(processed == true);
    assert(motor.current_speed == 1500);
    assert(motor.absolute_position == -12800);
}

static void test_process_ms1_zero_values(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.current_speed = 999;
    motor.absolute_position = 999;

    uint8_t data[8] = {0};
    data[0] = STEPPER_UIM2852_MS_SPEED_ABSPOS;
    // speed=0, position=0 (all zeros)

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MS), data, 8);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(motor.current_speed == 0);
    assert(motor.absolute_position == 0);
}

static void test_process_ms1_negative_speed(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    uint8_t data[8] = {0};
    data[0] = STEPPER_UIM2852_MS_SPEED_ABSPOS;
    // speed = -200 in 24-bit signed LE: 0x38, 0xFF, 0xFF
    data[1] = 0x38; data[2] = 0xFF; data[3] = 0xFF;
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MS), data, 8);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(motor.current_speed == -200);
}

// ============================================================================
// process_frame — NOTIFY PTP tests
// ============================================================================

static void test_process_notify_ptp_complete(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.motion_in_progress = true;
    motor.status.in_position = false;

    uint8_t data[8] = {0};
    data[0] = STEPPER_UIM2852_STATUS_PTP_COMPLETE;  // 0x29
    data[1] = 0;
    // position = 6400 = 0x00001900 LE
    data[4] = 0x00; data[5] = 0x19; data[6] = 0x00; data[7] = 0x00;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_NOTIFY), data, 8);
    bool processed = stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(processed == true);
    assert(motor.motion_in_progress == false);
    assert(motor.status.in_position == true);
    assert(motor.absolute_position == 6400);
}

static void test_process_notify_ptp_fires_callback(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    callback_call_count = 0;
    memset(&last_callback_notif, 0, sizeof(last_callback_notif));
    stepper_motor_uim2852_set_notify_callback(&motor, test_notify_callback);

    uint8_t data[8] = {0};
    data[0] = STEPPER_UIM2852_STATUS_PTP_COMPLETE;
    data[1] = 0;
    data[4] = 0x00; data[5] = 0x19; data[6] = 0x00; data[7] = 0x00;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_NOTIFY), data, 8);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(callback_call_count == 1);
    assert(last_callback_notif.type == STEPPER_UIM2852_STATUS_PTP_COMPLETE);
    assert(last_callback_notif.is_alarm == false);
    assert(last_callback_notif.position == 6400);
}

static void test_process_notify_ptp_no_callback(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    // No callback set — should not crash

    uint8_t data[8] = {0};
    data[0] = STEPPER_UIM2852_STATUS_PTP_COMPLETE;
    data[1] = 0;
    data[4] = 0x00; data[5] = 0x19; data[6] = 0x00; data[7] = 0x00;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_NOTIFY), data, 8);
    bool processed = stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(processed == true);
    // No crash is the success condition
}

// ============================================================================
// process_frame — NOTIFY stall tests
// ============================================================================

static void test_process_notify_stall(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.motion_in_progress = true;
    motor.status.stall_detected = false;

    uint8_t data[8] = {0};
    data[0] = 0x00;  // alarm indicator
    data[1] = STEPPER_UIM2852_ALARM_STALL;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_NOTIFY), data, 8);
    bool processed = stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(processed == true);
    assert(motor.status.stall_detected == true);
    assert(motor.motion_in_progress == false);
}

static void test_process_notify_stall_fires_callback(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    callback_call_count = 0;
    memset(&last_callback_notif, 0, sizeof(last_callback_notif));
    stepper_motor_uim2852_set_notify_callback(&motor, test_notify_callback);

    uint8_t data[8] = {0};
    data[0] = 0x00;
    data[1] = STEPPER_UIM2852_ALARM_STALL;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_NOTIFY), data, 8);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(callback_call_count == 1);
    assert(last_callback_notif.is_alarm == true);
    assert(last_callback_notif.type == STEPPER_UIM2852_ALARM_STALL);
}

// ============================================================================
// process_frame — ER tests
// ============================================================================

static void test_process_error_sets_flag(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.status.error_detected = false;

    uint8_t data[8] = {0};
    data[0] = 0;  // reserved
    data[1] = STEPPER_UIM2852_ERR_BG_DRIVER_OFF;
    data[2] = STEPPER_UIM2852_CW_BG;
    data[3] = 0;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_ER), data, 4);
    bool processed = stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(processed == true);
    assert(motor.status.error_detected == true);
}

static void test_process_error_short_data(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.status.error_detected = false;

    uint8_t data[8] = {0};
    data[0] = 0;
    data[1] = STEPPER_UIM2852_ERR_SYNTAX;

    // dl < 4, parse_error returns false, error_detected should stay false
    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_ER), data, 2);
    bool processed = stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(processed == true);  // Frame was from our motor, so it's "processed"
    assert(motor.status.error_detected == false);
}

// ============================================================================
// process_frame — MO ACK tests
// ============================================================================

static void test_process_mo_ack_enable(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.driver_enabled = false;

    uint8_t data[8] = {0};
    data[0] = 1;  // enable

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MO), data, 1);
    bool processed = stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(processed == true);
    assert(motor.driver_enabled == true);
}

static void test_process_mo_ack_disable(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.driver_enabled = true;

    uint8_t data[8] = {0};
    data[0] = 0;  // disable

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MO), data, 1);
    bool processed = stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(processed == true);
    assert(motor.driver_enabled == false);
}

// ============================================================================
// process_frame — param response tests
// ============================================================================

static void test_process_param_response_unblocks_query(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    // Simulate pending query for PP[5]
    motor.query_pending_cw = STEPPER_UIM2852_CW_PP;
    motor.query_pending_idx = 5;
    motor.query_result = 0;
    int initial_give_count = mock_sem_give_count;

    uint8_t data[8] = {0};
    data[0] = 5;   // index = 5
    data[1] = 16;  // value = 16 LE
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_PP), data, 5);
    bool processed = stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(processed == true);
    assert(motor.query_result == 16);
    assert(motor.query_pending_cw == 0);
    assert(mock_sem_give_count == initial_give_count + 1);
}

static void test_process_param_response_wrong_idx_ignored(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    motor.query_pending_cw = STEPPER_UIM2852_CW_PP;
    motor.query_pending_idx = 5;
    motor.query_result = 0;
    int initial_give_count = mock_sem_give_count;

    uint8_t data[8] = {0};
    data[0] = 7;   // wrong index
    data[1] = 42;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_PP), data, 5);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(motor.query_pending_cw == STEPPER_UIM2852_CW_PP);  // still pending
    assert(mock_sem_give_count == initial_give_count);  // not signaled
}

static void test_process_param_response_wrong_cw_ignored(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    motor.query_pending_cw = STEPPER_UIM2852_CW_PP;
    motor.query_pending_idx = 5;
    int initial_give_count = mock_sem_give_count;

    uint8_t data[8] = {0};
    data[0] = 5;
    data[1] = 16;

    // Send IC response instead of PP
    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_IC), data, 5);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(motor.query_pending_cw == STEPPER_UIM2852_CW_PP);  // still pending
    assert(mock_sem_give_count == initial_give_count);
}

static void test_process_param_response_no_pending(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    motor.query_pending_cw = 0;  // no pending query
    int initial_give_count = mock_sem_give_count;

    uint8_t data[8] = {0};
    data[0] = 5;
    data[1] = 16;

    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_PP), data, 5);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(mock_sem_give_count == initial_give_count);
}

// ============================================================================
// process_frame — general tests
// ============================================================================

static void test_process_frame_updates_timestamp(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    mock_tick_count = 42;
    motor.last_response_tick = 0;

    // Send any valid frame from the motor (e.g. MO ACK)
    uint8_t data[8] = {0};
    data[0] = 1;
    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MO), data, 1);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(motor.last_response_tick == 42);
}

static void test_process_frame_clears_ack_pending(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.ack_pending = true;

    uint8_t data[8] = {0};
    data[0] = 1;
    twai_message_t msg = make_motor_response(5, stepper_uim2852_cw_with_ack(STEPPER_UIM2852_CW_MO), data, 1);
    stepper_motor_uim2852_process_frame(&motor, &msg);
    assert(motor.ack_pending == false);
}

// ============================================================================
// query_param tests
// ============================================================================

static void test_query_param_sends_and_blocks(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    // mock_sem_take_result=pdTRUE means both the drain take and the blocking take succeed.
    // query_param will read motor->query_result which was reset to 0, so value=0.
    mock_sem_take_result = pdTRUE;
    motor.query_result = 0;

    int32_t value = -1;
    esp_err_t err = stepper_motor_uim2852_query_param(&motor, STEPPER_UIM2852_CW_PP, 5, &value);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_PP);
    assert(mock_sent_frames[0].data[0] == 5);  // index
    assert(value == 0);  // query_result was 0
}

static void test_query_param_timeout(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    mock_sem_take_result = pdFALSE;

    int32_t value = -1;
    esp_err_t err = stepper_motor_uim2852_query_param(&motor, STEPPER_UIM2852_CW_PP, 5, &value);
    assert(err == ESP_ERR_TIMEOUT);
    assert(mock_sent_count == 1);
    assert(motor.query_pending_cw == 0);  // cleared on timeout
}

// ============================================================================
// set_param tests
// ============================================================================

static void test_set_param_pp_sends_dl2(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    // PP uses DL=2: d0=index, d1=u8 value
    esp_err_t err = stepper_motor_uim2852_set_param(&motor, STEPPER_UIM2852_CW_PP, 7, 42);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_PP);
    assert(mock_sent_frames[0].data[0] == 7);   // index
    assert(mock_sent_frames[0].data[1] == 42);  // u8 value
    assert(mock_sent_frames[0].len == 2);
}

static void test_set_param_ic_sends_dl3(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    // IC uses DL=3: d0=index, d1-d2=u16 LE value
    esp_err_t err = stepper_motor_uim2852_set_param(&motor, STEPPER_UIM2852_CW_IC, 0, 0x0102);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_IC);
    assert(mock_sent_frames[0].data[0] == 0);      // index
    assert(mock_sent_frames[0].data[1] == 0x02);   // low byte
    assert(mock_sent_frames[0].data[2] == 0x01);   // high byte
    assert(mock_sent_frames[0].len == 3);
}

static void test_set_param_lm_sends_dl5(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    // LM uses DL=5: d0=index, d1-d4=s32 LE value
    esp_err_t err = stepper_motor_uim2852_set_param(&motor, STEPPER_UIM2852_CW_LM, 1, -500);
    assert(err == ESP_OK);
    assert(mock_sent_count == 1);
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_LM);
    assert(mock_sent_frames[0].data[0] == 1);      // index
    // -500 = 0xFFFFFE0C LE
    assert(mock_sent_frames[0].data[1] == 0x0C);
    assert(mock_sent_frames[0].data[2] == 0xFE);
    assert(mock_sent_frames[0].data[3] == 0xFF);
    assert(mock_sent_frames[0].data[4] == 0xFF);
    assert(mock_sent_frames[0].len == 5);
}

// ============================================================================
// configure tests
// ============================================================================

static void test_configure_timeout_uses_defaults(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    mock_sem_take_result = pdFALSE;  // query_param will time out

    esp_err_t err = stepper_motor_uim2852_configure(&motor);
    assert(err == ESP_ERR_TIMEOUT);
    // Microstep stays at default since configure exits on query timeout
    assert(motor.microstep_resolution == 16);
    assert(motor.pulses_per_rev == 3200);
    // Only the MT query frame is sent before returning timeout
    assert(mock_sent_count == 1);
}

// Helper: simulate process_frame setting query_result during the blocking semaphore wait
static stepper_motor_uim2852_t *s_configure_motor_ptr = NULL;
static void configure_sem_callback(int take_count) {
    // take_count 1 = drain call, take_count 2 = blocking wait for query response
    if (take_count == 2 && s_configure_motor_ptr) {
        s_configure_motor_ptr->query_result = 32;
    }
}

static void test_configure_success_updates_microstep(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    mock_sem_take_result = pdTRUE;
    // Use callback to simulate process_frame setting query_result during sem wait
    s_configure_motor_ptr = &motor;
    g_mock_sem_take_callback = configure_sem_callback;
    
    esp_err_t err = stepper_motor_uim2852_configure(&motor);
    assert(err == ESP_OK);
    assert(motor.microstep_resolution == 32);
    assert(motor.pulses_per_rev == 6400);  // 32 * 200
    
    g_mock_sem_take_callback = NULL;
    s_configure_motor_ptr = NULL;
}

// ============================================================================
// notify callback tests
// ============================================================================

static void test_set_notify_callback(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    stepper_motor_uim2852_set_notify_callback(&motor, test_notify_callback);
    assert(motor.notify_callback != NULL);
}

// ============================================================================
// go_relative failure paths
// ============================================================================

static void test_go_relative_pr_failure(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    mock_twai_send_result = ESP_FAIL;

    esp_err_t err = stepper_motor_uim2852_go_relative(&motor, 800);
    assert(err == ESP_FAIL);
    assert(mock_sent_count == 0);
    assert(motor.motion_in_progress == false);
}

static void test_go_relative_bg_failure(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    // Fail on the 2nd call to can_twai_send_extended (the BG send)
    g_mock_send_ext_fail_after = 2;
    esp_err_t err = stepper_motor_uim2852_go_relative(&motor, 800);
    assert(err == ESP_FAIL);
    assert(mock_sent_count == 1);  // only PR succeeded
    assert(get_sent_cw_base(0) == STEPPER_UIM2852_CW_PR);
    assert(motor.motion_in_progress == false);
    g_mock_send_ext_fail_after = 0;
}

// ============================================================================
// Status accessor tests — target_position and position_error
// ============================================================================

static void test_target_position_zero_after_init(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    assert(stepper_motor_uim2852_get_target_position(&motor) == 0);
}

static void test_target_position_set_by_go_absolute(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);

    esp_err_t err = stepper_motor_uim2852_go_absolute(&motor, 3200);
    assert(err == ESP_OK);
    assert(stepper_motor_uim2852_get_target_position(&motor) == 3200);
}

static void test_position_error_zero_when_at_target(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.target_position = 3200;
    motor.absolute_position = 3200;
    assert(stepper_motor_uim2852_position_error(&motor) == 0);
}

static void test_position_error_positive_delta(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.target_position = 3200;
    motor.absolute_position = 3000;
    assert(stepper_motor_uim2852_position_error(&motor) == 200);
}

static void test_position_error_negative_delta(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    motor.target_position = 3000;
    motor.absolute_position = 3200;
    // Absolute value: |3000 - 3200| = 200
    assert(stepper_motor_uim2852_position_error(&motor) == 200);
}

static void test_target_position_not_set_on_bg_failure(void) {
    stepper_motor_uim2852_t motor;
    init_default_motor(&motor);
    assert(stepper_motor_uim2852_get_target_position(&motor) == 0);

    // Fail on BG (2nd send)
    g_mock_send_ext_fail_after = 2;
    esp_err_t err = stepper_motor_uim2852_go_absolute(&motor, 6400);
    assert(err == ESP_FAIL);
    // target_position should NOT have been updated
    assert(stepper_motor_uim2852_get_target_position(&motor) == 0);
    g_mock_send_ext_fail_after = 0;
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    printf("stepper_motor_uim2852 component tests:\n");

    // Init tests
    printf("\n  --- init ---\n");
    TEST(test_init_with_defaults);
    TEST(test_init_with_custom_config);
    TEST(test_init_semaphore_fail);

    // Enable/disable tests
    printf("\n  --- enable/disable ---\n");
    TEST(test_enable_sends_mo_frame);
    TEST(test_disable_sends_mo_frame);
    TEST(test_enable_tx_failure);
    TEST(test_enable_uninitialized);

    // Stop tests
    printf("\n  --- stop ---\n");
    TEST(test_stop_sends_st);
    TEST(test_emergency_stop_sends_sd_then_st);

    // go_absolute tests
    printf("\n  --- go_absolute ---\n");
    TEST(test_go_absolute_sends_pa_then_bg);
    TEST(test_go_absolute_negative);
    TEST(test_go_absolute_pa_failure);
    TEST(test_go_absolute_bg_failure);

    // go_relative tests
    printf("\n  --- go_relative ---\n");
    TEST(test_go_relative_sends_pr_then_bg);
    TEST(test_go_relative_negative);
    TEST(test_go_relative_motion_flag);
    TEST(test_go_relative_pr_failure);
    TEST(test_go_relative_bg_failure);

    // set_origin tests
    printf("\n  --- set_origin ---\n");
    TEST(test_set_origin_sends_og);
    TEST(test_set_origin_clears_position);

    // set_speed / set_accel / set_decel tests
    printf("\n  --- set_speed/accel/decel ---\n");
    TEST(test_set_speed);
    TEST(test_set_accel);
    TEST(test_set_decel);

    // query_status / query_position / clear_status tests
    printf("\n  --- query_status/position/clear ---\n");
    TEST(test_query_status);
    TEST(test_query_position);
    TEST(test_clear_status);

    // process_frame — filtering
    printf("\n  --- process_frame filtering ---\n");
    TEST(test_process_frame_standard_frame);
    TEST(test_process_frame_wrong_node);

    // process_frame — MS[0]
    printf("\n  --- process_frame MS[0] ---\n");
    TEST(test_process_ms0_updates_status);
    TEST(test_process_ms0_clears_motion_on_inposition);
    TEST(test_process_ms0_clears_motion_on_stopped);
    TEST(test_process_ms0_updates_driver_enabled);
    TEST(test_process_ms0_negative_position);

    // process_frame — MS[1]
    printf("\n  --- process_frame MS[1] ---\n");
    TEST(test_process_ms1_updates_speed_and_position);
    TEST(test_process_ms1_zero_values);
    TEST(test_process_ms1_negative_speed);

    // process_frame — NOTIFY PTP
    printf("\n  --- process_frame NOTIFY PTP ---\n");
    TEST(test_process_notify_ptp_complete);
    TEST(test_process_notify_ptp_fires_callback);
    TEST(test_process_notify_ptp_no_callback);

    // process_frame — NOTIFY stall
    printf("\n  --- process_frame NOTIFY stall ---\n");
    TEST(test_process_notify_stall);
    TEST(test_process_notify_stall_fires_callback);

    // process_frame — ER
    printf("\n  --- process_frame ER ---\n");
    TEST(test_process_error_sets_flag);
    TEST(test_process_error_short_data);

    // process_frame — MO ACK
    printf("\n  --- process_frame MO ACK ---\n");
    TEST(test_process_mo_ack_enable);
    TEST(test_process_mo_ack_disable);

    // process_frame — param response
    printf("\n  --- process_frame param response ---\n");
    TEST(test_process_param_response_unblocks_query);
    TEST(test_process_param_response_wrong_idx_ignored);
    TEST(test_process_param_response_wrong_cw_ignored);
    TEST(test_process_param_response_no_pending);

    // process_frame — general
    printf("\n  --- process_frame general ---\n");
    TEST(test_process_frame_updates_timestamp);
    TEST(test_process_frame_clears_ack_pending);

    // query_param
    printf("\n  --- query_param ---\n");
    TEST(test_query_param_sends_and_blocks);
    TEST(test_query_param_timeout);

    // set_param
    printf("\n  --- set_param ---\n");
    TEST(test_set_param_pp_sends_dl2);
    TEST(test_set_param_ic_sends_dl3);
    TEST(test_set_param_lm_sends_dl5);

    // configure
    printf("\n  --- configure ---\n");
    TEST(test_configure_timeout_uses_defaults);
    TEST(test_configure_success_updates_microstep);

    // notify callback
    printf("\n  --- notify callback ---\n");
    TEST(test_set_notify_callback);

    // status accessors
    printf("\n  --- status accessors ---\n");
    TEST(test_target_position_zero_after_init);
    TEST(test_target_position_set_by_go_absolute);
    TEST(test_position_error_zero_when_at_target);
    TEST(test_position_error_positive_delta);
    TEST(test_position_error_negative_delta);
    TEST(test_target_position_not_set_on_bg_failure);

    printf("\n%d/%d tests passed.\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
