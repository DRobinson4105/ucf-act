/**
 * @file test_motor_uim2852_protocol.cpp
 * @brief Unit tests for motor_uim2852_protocol codec, command builders, and RX
 *        parsing.
 *
 * Replaces the deleted test_motor_component.cpp with focused coverage of the
 * pure-protocol layer (no component/driver mocks needed).
 */

#include "test_harness.h"
#include "motor_protocol.h"
#include "motor_codec.h"
#include "motor_rx.h"
#include <string.h>
#include <assert.h>
#include <limits.h>

/* Base code constants duplicated here for assertion clarity.  Keep in sync with
 * the private #defines inside motor_cmd.cpp / motor_rx.cpp. */
#define BASE_PP   0x01U
#define BASE_IC   0x06U
#define BASE_IE   0x07U
#define BASE_ER   0x0FU
#define BASE_MT   0x10U
#define BASE_MO   0x15U
#define BASE_BG   0x16U
#define BASE_ST   0x17U
#define BASE_SD   0x1CU
#define BASE_PA   0x20U
#define BASE_OG   0x21U
#define BASE_MP   0x22U
#define BASE_PV   0x23U
#define BASE_PT   0x24U
#define BASE_LM   0x2CU
#define BASE_BL   0x2DU
#define BASE_DV   0x2EU
#define BASE_IL   0x34U
#define BASE_QE   0x3DU
#define BASE_NOTIFY 0x5AU

/* Default motor node and host IDs used throughout tests. */
#define MOTOR_NODE  6U
#define HOST_ID     0x04U

/* ========================================================================== */
/*  Helpers                                                                   */
/* ========================================================================== */

/**
 * Build a fake twai_message_t that looks like a response from a motor node.
 *
 * @param producer_id  Node that sent the frame (motor node for responses).
 * @param consumer_id  Target that the frame is addressed to (host, usually 0).
 * @param cw_raw       Command word byte (base | 0x80 for ACK).
 * @param dlc          Data-length code.
 * @param data         Payload bytes (may be NULL if dlc == 0).
 */
static twai_message_t make_rx_frame(uint8_t producer_id,
                                    uint8_t consumer_id,
                                    uint8_t cw_raw,
                                    uint8_t dlc,
                                    const uint8_t *data)
{
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.extd = 1;
    msg.identifier = motor_codec_build_ext_id_endpoints(producer_id, consumer_id, cw_raw);
    msg.data_length_code = dlc;
    if (data != NULL && dlc > 0) {
        memcpy(msg.data, data, dlc);
    }
    return msg;
}

/* Convenience: ACK frame from MOTOR_NODE, consumer=0. */
static twai_message_t make_ack_frame(uint8_t base_code,
                                     uint8_t dlc,
                                     const uint8_t *data)
{
    return make_rx_frame(MOTOR_NODE, 0, base_code, dlc, data);
}

/* ========================================================================== */
/*  Codec tests                                                               */
/* ========================================================================== */

static void test_codec_compose_cw_no_ack(void)
{
    uint8_t cw = motor_codec_compose_cw(BASE_PA, false);
    assert(cw == BASE_PA);
    assert(motor_codec_base_code(cw) == BASE_PA);
    assert(!motor_codec_ack_requested(cw));
}

static void test_codec_compose_cw_with_ack(void)
{
    uint8_t cw = motor_codec_compose_cw(BASE_PA, true);
    assert(cw == (BASE_PA | 0x80U));
    assert(motor_codec_base_code(cw) == BASE_PA);
    assert(motor_codec_ack_requested(cw));
}

static void test_codec_compose_cw_ack_bit_is_bit7(void)
{
    uint8_t cw = motor_codec_compose_cw(0x01, true);
    assert((cw & 0x80U) == 0x80U);
}

static void test_codec_build_ext_id_roundtrip(void)
{
    uint8_t cw = motor_codec_compose_cw(BASE_MO, true);
    uint32_t id = motor_codec_build_ext_id(MOTOR_NODE, cw);

    assert(motor_codec_decode_producer_id(id) == HOST_ID);
    assert(motor_codec_decode_consumer_id(id) == MOTOR_NODE);
    assert(motor_codec_decode_cw(id) == cw);
}

static void test_codec_build_ext_id_endpoints_roundtrip(void)
{
    uint8_t cw = motor_codec_compose_cw(BASE_BG, false);
    uint32_t id = motor_codec_build_ext_id_endpoints(MOTOR_NODE, 0, cw);

    assert(motor_codec_decode_producer_id(id) == MOTOR_NODE);
    assert(motor_codec_decode_consumer_id(id) == 0);
    assert(motor_codec_decode_cw(id) == cw);
}

static void test_codec_ext_id_29bit_mask(void)
{
    uint32_t id = motor_codec_build_ext_id(MOTOR_NODE, 0xFF);
    assert((id & 0xE0000000U) == 0);
}

static void test_codec_decode_id_is_consumer(void)
{
    uint32_t id = motor_codec_build_ext_id(MOTOR_NODE, BASE_PA);
    assert(motor_codec_decode_id(id) == MOTOR_NODE);
}

static void test_codec_pack_i32_le_positive(void)
{
    uint8_t buf[4];
    motor_codec_pack_i32_le(buf, 12345);
    assert(motor_codec_unpack_i32_le(buf) == 12345);
}

static void test_codec_pack_i32_le_negative(void)
{
    uint8_t buf[4];
    motor_codec_pack_i32_le(buf, -99999);
    assert(motor_codec_unpack_i32_le(buf) == -99999);
}

static void test_codec_pack_i32_le_zero(void)
{
    uint8_t buf[4];
    motor_codec_pack_i32_le(buf, 0);
    assert(motor_codec_unpack_i32_le(buf) == 0);
}

static void test_codec_pack_i32_le_min(void)
{
    uint8_t buf[4];
    motor_codec_pack_i32_le(buf, INT32_MIN);
    assert(motor_codec_unpack_i32_le(buf) == INT32_MIN);
}

static void test_codec_pack_i32_le_max(void)
{
    uint8_t buf[4];
    motor_codec_pack_i32_le(buf, INT32_MAX);
    assert(motor_codec_unpack_i32_le(buf) == INT32_MAX);
}

static void test_codec_pack_u16_le_roundtrip(void)
{
    uint8_t buf[2];
    motor_codec_pack_u16_le(buf, 0xABCDU);
    assert(motor_codec_unpack_u16_le(buf) == 0xABCDU);
}

static void test_codec_pack_u32_le_roundtrip(void)
{
    uint8_t buf[4];
    motor_codec_pack_u32_le(buf, 0xDEADBEEFU);
    assert(motor_codec_unpack_u32_le(buf) == 0xDEADBEEFU);
}

static void test_codec_pack_u16_le_byte_order(void)
{
    uint8_t buf[2];
    motor_codec_pack_u16_le(buf, 0x0102U);
    assert(buf[0] == 0x02U);
    assert(buf[1] == 0x01U);
}

/* ========================================================================== */
/*  Command builder tests                                                     */
/* ========================================================================== */

static void test_cmd_mo_set_enable(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_mo_set(MOTOR_NODE, true, MOTOR_MO_STATE_ENABLE, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_MO);
    assert(cmd.section == MOTOR_SECTION_MOTOR_DRIVER);
    assert(cmd.base_code == BASE_MO);
    assert(cmd.ack_requested == true);
    assert(cmd.msg.data_length_code == 1);
    assert(cmd.msg.data[0] == (uint8_t)MOTOR_MO_STATE_ENABLE);
    assert(cmd.msg.extd == 1);
}

static void test_cmd_mo_set_disable(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_mo_set(MOTOR_NODE, false, MOTOR_MO_STATE_DISABLE, &cmd) == ESP_OK);
    assert(cmd.msg.data[0] == (uint8_t)MOTOR_MO_STATE_DISABLE);
    assert(cmd.ack_requested == false);
}

static void test_cmd_bg_begin(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_bg_begin(MOTOR_NODE, true, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_BG);
    assert(cmd.section == MOTOR_SECTION_MOTION_CONTROL);
    assert(cmd.base_code == BASE_BG);
    assert(cmd.msg.data_length_code == 0);
}

static void test_cmd_st_stop(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_st_stop(MOTOR_NODE, true, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_ST);
    assert(cmd.section == MOTOR_SECTION_MOTION_CONTROL);
    assert(cmd.base_code == BASE_ST);
    assert(cmd.msg.data_length_code == 0);
}

static void test_cmd_pa_set_positive(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_pa_set(MOTOR_NODE, true, 50000, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_PA);
    assert(cmd.section == MOTOR_SECTION_MOTION_CONTROL);
    assert(cmd.base_code == BASE_PA);
    assert(cmd.msg.data_length_code == 4);
    assert(motor_codec_unpack_i32_le(cmd.msg.data) == 50000);
}

static void test_cmd_pa_set_negative(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_pa_set(MOTOR_NODE, true, -12345, &cmd) == ESP_OK);
    assert(motor_codec_unpack_i32_le(cmd.msg.data) == -12345);
}

static void test_cmd_pt_set(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_pt_set(MOTOR_NODE, true, 7, 100000, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_PT);
    assert(cmd.section == MOTOR_SECTION_INTERPOLATED_MOTION);
    assert(cmd.msg.data_length_code == 8);
    assert(motor_codec_unpack_u16_le(&cmd.msg.data[0]) == 7);
    assert(motor_codec_unpack_i32_le(&cmd.msg.data[2]) == 100000);
    assert(cmd.has_index == true);
    assert(cmd.index == 7);
}

static void test_cmd_lm_set_i32(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_lm_set_i32(MOTOR_NODE, true, MOTOR_LM_INDEX_MAX_SPEED, 5000, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_LM);
    assert(cmd.section == MOTOR_SECTION_MOTION_CONTROL);
    assert(cmd.msg.data_length_code == 5);
    assert(cmd.msg.data[0] == (uint8_t)MOTOR_LM_INDEX_MAX_SPEED);
    assert(motor_codec_unpack_i32_le(&cmd.msg.data[1]) == 5000);
    assert(cmd.has_index == true);
    assert(cmd.index == (uint16_t)MOTOR_LM_INDEX_MAX_SPEED);
}

static void test_cmd_sd_set_u32(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_sd_set_u32(MOTOR_NODE, true, 999, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_SD);
    assert(cmd.section == MOTOR_SECTION_MOTION_CONTROL);
    assert(cmd.msg.data_length_code == 4);
    assert(motor_codec_unpack_u32_le(cmd.msg.data) == 999);
}

static void test_cmd_mp_set_u16(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_mp_set_u16(MOTOR_NODE, true, MOTOR_MP_INDEX_PT_MOTION_TIME, 100, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_MP);
    assert(cmd.section == MOTOR_SECTION_INTERPOLATED_MOTION);
    assert(cmd.msg.data_length_code == 3);
    assert(cmd.msg.data[0] == (uint8_t)MOTOR_MP_INDEX_PT_MOTION_TIME);
    assert(motor_codec_unpack_u16_le(&cmd.msg.data[1]) == 100);
    assert(cmd.has_index == true);
}

static void test_cmd_ic_set_u16(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_ic_set_u16(MOTOR_NODE, true, MOTOR_IC_INDEX_USE_CLOSED_LOOP, 1, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_IC);
    assert(cmd.section == MOTOR_SECTION_SYSTEM);
    assert(cmd.msg.data_length_code == 3);
    assert(cmd.msg.data[0] == (uint8_t)MOTOR_IC_INDEX_USE_CLOSED_LOOP);
    assert(motor_codec_unpack_u16_le(&cmd.msg.data[1]) == 1);
    assert(cmd.has_index == true);
}

static void test_cmd_ie_set_u16(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_ie_set_u16(MOTOR_NODE, true, MOTOR_IE_INDEX_PTP_POSITIONING_FINISH_NOTIFICATION,
                                MOTOR_IE_STATE_ENABLE, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_IE);
    assert(cmd.section == MOTOR_SECTION_SYSTEM);
    assert(cmd.msg.data_length_code == 3);
    assert(cmd.has_index == true);
}

static void test_cmd_er_clear_all(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_er_clear_all(MOTOR_NODE, true, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_ER);
    assert(cmd.section == MOTOR_SECTION_SYSTEM);
    assert(cmd.base_code == BASE_ER);
    assert(cmd.msg.data_length_code == 2);
    assert(cmd.msg.data[0] == (uint8_t)MOTOR_ER_INDEX_CLEAR_ALL);
    assert(cmd.msg.data[1] == 0);
    assert(cmd.has_index == true);
}

static void test_cmd_null_output_returns_error(void)
{
    assert(motor_cmd_mo_set(MOTOR_NODE, true, MOTOR_MO_STATE_ENABLE, NULL) == ESP_ERR_INVALID_ARG);
    assert(motor_cmd_bg_begin(MOTOR_NODE, true, NULL) == ESP_ERR_INVALID_ARG);
    assert(motor_cmd_pa_set(MOTOR_NODE, true, 0, NULL) == ESP_ERR_INVALID_ARG);
}

static void test_cmd_invalid_target_id_returns_error(void)
{
    motor_cmd_t cmd;
    /* target_id 1 is in the invalid range (1-4) */
    assert(motor_cmd_mo_set(1, true, MOTOR_MO_STATE_ENABLE, &cmd) == ESP_ERR_INVALID_ARG);
    assert(motor_cmd_bg_begin(2, true, &cmd) == ESP_ERR_INVALID_ARG);
    assert(motor_cmd_pa_set(3, true, 0, &cmd) == ESP_ERR_INVALID_ARG);
    assert(motor_cmd_sd_set_u32(4, true, 0, &cmd) == ESP_ERR_INVALID_ARG);
}

static void test_cmd_target_id_zero_allowed(void)
{
    motor_cmd_t cmd;
    /* target_id 0 is allowed (broadcast). */
    assert(motor_cmd_mo_set(0, true, MOTOR_MO_STATE_ENABLE, &cmd) == ESP_OK);
}

static void test_cmd_target_id_127_invalid(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_mo_set(127, true, MOTOR_MO_STATE_ENABLE, &cmd) == ESP_ERR_INVALID_ARG);
}

static void test_cmd_pa_get_dlc_zero(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_pa_get(MOTOR_NODE, true, &cmd) == ESP_OK);
    assert(cmd.msg.data_length_code == 0);
    assert(cmd.object == MOTOR_OBJECT_PA);
}

static void test_cmd_og_set_origin(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_og_set_origin(MOTOR_NODE, true, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_OG);
    assert(cmd.section == MOTOR_SECTION_MOTION_CONTROL);
    assert(cmd.msg.data_length_code == 0);
}

static void test_cmd_bl_set_u32(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_bl_set_u32(MOTOR_NODE, true, 12345678, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_BL);
    assert(cmd.msg.data_length_code == 4);
    assert(motor_codec_unpack_u32_le(cmd.msg.data) == 12345678);
}

static void test_cmd_pv_set(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_pv_set(MOTOR_NODE, true, 42, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_PV);
    assert(cmd.section == MOTOR_SECTION_INTERPOLATED_MOTION);
    assert(cmd.msg.data_length_code == 2);
    assert(motor_codec_unpack_u16_le(cmd.msg.data) == 42);
}

static void test_cmd_sy_reboot(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_sy_reboot(MOTOR_NODE, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_SY);
    assert(cmd.section == MOTOR_SECTION_SYSTEM);
    assert(cmd.msg.data[0] == (uint8_t)MOTOR_SY_OP_REBOOT);
}

static void test_cmd_extd_flag_always_set(void)
{
    motor_cmd_t cmd;
    motor_cmd_bg_begin(MOTOR_NODE, true, &cmd);
    assert(cmd.msg.extd == 1);
    motor_cmd_pa_set(MOTOR_NODE, false, 0, &cmd);
    assert(cmd.msg.extd == 1);
}

static void test_cmd_mt_set_u16(void)
{
    motor_cmd_t cmd;
    assert(motor_cmd_mt_set_u16(MOTOR_NODE, true, MOTOR_MT_INDEX_WORKING_CURRENT, 40, &cmd) == ESP_OK);
    assert(cmd.object == MOTOR_OBJECT_MT);
    assert(cmd.section == MOTOR_SECTION_MOTOR_DRIVER);
    assert(cmd.msg.data_length_code == 3);
    assert(cmd.msg.data[0] == (uint8_t)MOTOR_MT_INDEX_WORKING_CURRENT);
    assert(motor_codec_unpack_u16_le(&cmd.msg.data[1]) == 40);
    assert(cmd.has_index == true);
}

/* ========================================================================== */
/*  RX parsing tests                                                          */
/* ========================================================================== */

static void test_rx_parse_ack_frame(void)
{
    uint8_t data[] = { (uint8_t)MOTOR_MO_STATE_ENABLE };
    twai_message_t msg = make_ack_frame(BASE_MO, 1, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    assert(motor_rx_is_ack(&rx));
    assert(!motor_rx_is_error(&rx));
    assert(!motor_rx_is_notification(&rx));
}

static void test_rx_parse_notification_frame(void)
{
    /* status notification: d0 != 0 (non-alarm), 6 bytes total */
    uint8_t data[6] = { 0x08, 0x00, 0xD2, 0x04, 0x00, 0x00 }; /* d0=8, pos=1234 */
    twai_message_t msg = make_rx_frame(MOTOR_NODE, 0, BASE_NOTIFY, 6, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    assert(motor_rx_is_notification(&rx));
    assert(!motor_rx_is_ack(&rx));
}

static void test_rx_parse_error_frame(void)
{
    /* ER error: slot=0, code=5, related_cw=0x20, subindex=0 */
    uint8_t data[4] = { 0x00, 0x05, 0x20, 0x00 };
    twai_message_t msg = make_rx_frame(MOTOR_NODE, 0, BASE_ER, 4, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    assert(motor_rx_is_error(&rx));
    assert(!motor_rx_is_ack(&rx));
}

static void test_rx_parse_standard_frame_rejected(void)
{
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.extd = 0; /* standard frame */
    msg.data_length_code = 1;
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_ERR_INVALID_ARG);
}

static void test_rx_parse_rtr_frame_rejected(void)
{
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.extd = 1;
    msg.rtr = 1;
    msg.data_length_code = 0;
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_ERR_INVALID_ARG);
}

static void test_rx_matches_cmd_correct_node(void)
{
    /* Build a command */
    motor_cmd_t cmd;
    motor_cmd_mo_set(MOTOR_NODE, true, MOTOR_MO_STATE_ENABLE, &cmd);

    /* Build matching ACK from the motor node */
    uint8_t data[] = { (uint8_t)MOTOR_MO_STATE_ENABLE };
    twai_message_t msg = make_rx_frame(MOTOR_NODE, 0, BASE_MO, 1, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    assert(motor_rx_matches_cmd(&rx, &cmd));
}

static void test_rx_matches_cmd_wrong_node(void)
{
    motor_cmd_t cmd;
    motor_cmd_mo_set(MOTOR_NODE, true, MOTOR_MO_STATE_ENABLE, &cmd);

    /* ACK from a different motor node */
    uint8_t data[] = { (uint8_t)MOTOR_MO_STATE_ENABLE };
    twai_message_t msg = make_rx_frame(7, 0, BASE_MO, 1, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    assert(!motor_rx_matches_cmd(&rx, &cmd));
}

static void test_rx_matches_cmd_wrong_cw(void)
{
    motor_cmd_t cmd;
    motor_cmd_mo_set(MOTOR_NODE, true, MOTOR_MO_STATE_ENABLE, &cmd);

    /* ACK with a different base code (BG instead of MO) */
    twai_message_t msg = make_rx_frame(MOTOR_NODE, 0, BASE_BG, 0, NULL);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    assert(!motor_rx_matches_cmd(&rx, &cmd));
}

static void test_rx_ack_mo_enable(void)
{
    uint8_t data[] = { (uint8_t)MOTOR_MO_STATE_ENABLE };
    twai_message_t msg = make_ack_frame(BASE_MO, 1, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    uint8_t state = 0xFF;
    assert(motor_rx_ack_mo(&rx, &state));
    assert(state == (uint8_t)MOTOR_MO_STATE_ENABLE);
}

static void test_rx_ack_mo_disable(void)
{
    uint8_t data[] = { (uint8_t)MOTOR_MO_STATE_DISABLE };
    twai_message_t msg = make_ack_frame(BASE_MO, 1, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    uint8_t state = 0xFF;
    assert(motor_rx_ack_mo(&rx, &state));
    assert(state == (uint8_t)MOTOR_MO_STATE_DISABLE);
}

static void test_rx_ack_pa_set_extracts_position(void)
{
    /* PA SET response uses DV family base=0x2E, DLC 5 */
    uint8_t data[5];
    data[0] = 4; /* DV subindex for PA absolute */
    motor_codec_pack_i32_le(&data[1], -5000);
    twai_message_t msg = make_ack_frame(BASE_DV, 5, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    int32_t pos = 0;
    assert(motor_rx_ack_pa_set(&rx, &pos));
    assert(pos == -5000);
}

static void test_rx_ack_pa_get_extracts_position(void)
{
    uint8_t data[4];
    motor_codec_pack_i32_le(data, 77777);
    twai_message_t msg = make_ack_frame(BASE_PA, 4, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    int32_t pos = 0;
    assert(motor_rx_ack_pa_get(&rx, &pos));
    assert(pos == 77777);
}

static void test_rx_ack_pt_extracts_row_and_position(void)
{
    uint8_t data[8];
    motor_codec_pack_u16_le(&data[0], 3);       /* row index */
    motor_codec_pack_i32_le(&data[2], 200000);   /* position  */
    data[6] = 0;
    data[7] = 0;
    twai_message_t msg = make_ack_frame(BASE_PT, 8, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    uint16_t row = 0;
    int32_t pos = 0;
    assert(motor_rx_ack_pt(&rx, &row, &pos));
    assert(row == 3);
    assert(pos == 200000);
}

static void test_rx_notification_current_position(void)
{
    /* Status notification: d0 != 0 (status, not alarm), position in d2..d5 */
    uint8_t data[6];
    data[0] = 0x08; /* non-zero d0 -> status notification */
    data[1] = 0x00;
    motor_codec_pack_i32_le(&data[2], 42000);
    twai_message_t msg = make_rx_frame(MOTOR_NODE, 0, BASE_NOTIFY, 6, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    int32_t pos = 0;
    assert(motor_rx_notification_current_position(&rx, &pos));
    assert(pos == 42000);
}

static void test_rx_error_code_extraction(void)
{
    uint8_t data[4] = { 0x00, 0x07, 0x20, 0x01 };
    twai_message_t msg = make_rx_frame(MOTOR_NODE, 0, BASE_ER, 4, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    uint8_t code = 0;
    assert(motor_rx_error_code(&rx, &code));
    assert(code == 0x07);
}

static void test_rx_er_thrown_error_vs_history(void)
{
    /* Thrown error: slot=0 (MOTOR_ER_INDEX_CLEAR_ALL), non-zero payload */
    uint8_t thrown_data[4] = { 0x00, 0x05, 0x15, 0x00 };
    twai_message_t msg1 = make_rx_frame(MOTOR_NODE, 0, BASE_ER, 4, thrown_data);
    motor_rx_t rx1;
    assert(motor_rx_parse(&msg1, &rx1) == ESP_OK);
    assert(motor_rx_er_is_thrown_error(&rx1));
    assert(!motor_rx_er_is_history_response(&rx1));

    /* History response: slot=10 (MOTOR_ER_INDEX_HISTORY_1) */
    uint8_t hist_data[6] = { 10, 0x05, 0x15, 0x00, 0x00, 0x00 };
    twai_message_t msg2 = make_rx_frame(MOTOR_NODE, 0, BASE_ER, 6, hist_data);
    motor_rx_t rx2;
    assert(motor_rx_parse(&msg2, &rx2) == ESP_OK);
    assert(motor_rx_er_is_history_response(&rx2));
    assert(!motor_rx_er_is_thrown_error(&rx2));
}

static void test_rx_er_clear_all_response(void)
{
    /* Clear-all response: slot=0, all zeros, DLC 6 */
    uint8_t data[6] = { 0, 0, 0, 0, 0, 0 };
    twai_message_t msg = make_rx_frame(MOTOR_NODE, 0, BASE_ER, 6, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    assert(motor_rx_er_is_clear_all_response(&rx));
    assert(!motor_rx_er_is_thrown_error(&rx));
}

static void test_rx_producer_consumer_decode(void)
{
    twai_message_t msg = make_rx_frame(MOTOR_NODE, 0, BASE_MO, 1,
                                       (const uint8_t[]){ 1 });
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    assert(rx.producer_id == MOTOR_NODE);
    assert(rx.consumer_id == 0);
}

static void test_rx_ack_bg(void)
{
    twai_message_t msg = make_ack_frame(BASE_BG, 0, NULL);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    assert(motor_rx_ack_bg(&rx));
}

static void test_rx_ack_st(void)
{
    twai_message_t msg = make_ack_frame(BASE_ST, 0, NULL);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    assert(motor_rx_ack_st(&rx));
}

static void test_rx_ack_sd(void)
{
    uint8_t data[4];
    motor_codec_pack_u32_le(data, 500);
    twai_message_t msg = make_ack_frame(BASE_SD, 4, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    uint32_t val = 0;
    assert(motor_rx_ack_sd(&rx, &val));
    assert(val == 500);
}

static void test_rx_ack_mp(void)
{
    uint8_t data[3];
    data[0] = (uint8_t)MOTOR_MP_INDEX_PT_MOTION_TIME;
    motor_codec_pack_u16_le(&data[1], 100);
    twai_message_t msg = make_ack_frame(BASE_MP, 3, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    motor_mp_index_t idx;
    uint16_t val = 0;
    assert(motor_rx_ack_mp(&rx, &idx, &val));
    assert(idx == MOTOR_MP_INDEX_PT_MOTION_TIME);
    assert(val == 100);
}

static void test_rx_notification_alarm(void)
{
    /* Alarm: d0==0, d1==alarm code */
    uint8_t data[2] = { 0x00, 0x03 };
    twai_message_t msg = make_rx_frame(MOTOR_NODE, 0, BASE_NOTIFY, 2, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    assert(motor_rx_notification_is_alarm(&rx));
    assert(!motor_rx_notification_is_status(&rx));
    uint8_t code = 0;
    assert(motor_rx_notification_code(&rx, &code));
    assert(code == 0x03);
}

static void test_rx_null_args(void)
{
    motor_rx_t rx;
    assert(motor_rx_parse(NULL, &rx) == ESP_ERR_INVALID_ARG);

    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));
    msg.extd = 1;
    assert(motor_rx_parse(&msg, NULL) == ESP_ERR_INVALID_ARG);
}

static void test_rx_ack_ic(void)
{
    uint8_t data[3];
    data[0] = (uint8_t)MOTOR_IC_INDEX_USE_CLOSED_LOOP;
    motor_codec_pack_u16_le(&data[1], 1);
    twai_message_t msg = make_ack_frame(BASE_IC, 3, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    motor_ic_index_t idx;
    uint16_t val = 0;
    assert(motor_rx_ack_ic(&rx, &idx, &val));
    assert(idx == MOTOR_IC_INDEX_USE_CLOSED_LOOP);
    assert(val == 1);
}

static void test_rx_ack_lm_get(void)
{
    uint8_t data[5];
    data[0] = (uint8_t)MOTOR_LM_INDEX_MAX_SPEED;
    motor_codec_pack_i32_le(&data[1], 10000);
    twai_message_t msg = make_ack_frame(BASE_LM, 5, data);
    motor_rx_t rx;
    assert(motor_rx_parse(&msg, &rx) == ESP_OK);
    motor_lm_index_t idx;
    int32_t val = 0;
    assert(motor_rx_ack_lm_get(&rx, &idx, &val));
    assert(idx == MOTOR_LM_INDEX_MAX_SPEED);
    assert(val == 10000);
}

/* ========================================================================== */
/*  main                                                                      */
/* ========================================================================== */

int main(void)
{
    printf("\n=== motor_uim2852_protocol unit tests ===\n\n");

    /* Codec tests */
    TEST(test_codec_compose_cw_no_ack);
    TEST(test_codec_compose_cw_with_ack);
    TEST(test_codec_compose_cw_ack_bit_is_bit7);
    TEST(test_codec_build_ext_id_roundtrip);
    TEST(test_codec_build_ext_id_endpoints_roundtrip);
    TEST(test_codec_ext_id_29bit_mask);
    TEST(test_codec_decode_id_is_consumer);
    TEST(test_codec_pack_i32_le_positive);
    TEST(test_codec_pack_i32_le_negative);
    TEST(test_codec_pack_i32_le_zero);
    TEST(test_codec_pack_i32_le_min);
    TEST(test_codec_pack_i32_le_max);
    TEST(test_codec_pack_u16_le_roundtrip);
    TEST(test_codec_pack_u32_le_roundtrip);
    TEST(test_codec_pack_u16_le_byte_order);

    /* Command builder tests */
    TEST(test_cmd_mo_set_enable);
    TEST(test_cmd_mo_set_disable);
    TEST(test_cmd_bg_begin);
    TEST(test_cmd_st_stop);
    TEST(test_cmd_pa_set_positive);
    TEST(test_cmd_pa_set_negative);
    TEST(test_cmd_pt_set);
    TEST(test_cmd_lm_set_i32);
    TEST(test_cmd_sd_set_u32);
    TEST(test_cmd_mp_set_u16);
    TEST(test_cmd_ic_set_u16);
    TEST(test_cmd_ie_set_u16);
    TEST(test_cmd_er_clear_all);
    TEST(test_cmd_null_output_returns_error);
    TEST(test_cmd_invalid_target_id_returns_error);
    TEST(test_cmd_target_id_zero_allowed);
    TEST(test_cmd_target_id_127_invalid);
    TEST(test_cmd_pa_get_dlc_zero);
    TEST(test_cmd_og_set_origin);
    TEST(test_cmd_bl_set_u32);
    TEST(test_cmd_pv_set);
    TEST(test_cmd_sy_reboot);
    TEST(test_cmd_extd_flag_always_set);
    TEST(test_cmd_mt_set_u16);

    /* RX parsing tests */
    TEST(test_rx_parse_ack_frame);
    TEST(test_rx_parse_notification_frame);
    TEST(test_rx_parse_error_frame);
    TEST(test_rx_parse_standard_frame_rejected);
    TEST(test_rx_parse_rtr_frame_rejected);
    TEST(test_rx_matches_cmd_correct_node);
    TEST(test_rx_matches_cmd_wrong_node);
    TEST(test_rx_matches_cmd_wrong_cw);
    TEST(test_rx_ack_mo_enable);
    TEST(test_rx_ack_mo_disable);
    TEST(test_rx_ack_pa_set_extracts_position);
    TEST(test_rx_ack_pa_get_extracts_position);
    TEST(test_rx_ack_pt_extracts_row_and_position);
    TEST(test_rx_notification_current_position);
    TEST(test_rx_error_code_extraction);
    TEST(test_rx_er_thrown_error_vs_history);
    TEST(test_rx_er_clear_all_response);
    TEST(test_rx_producer_consumer_decode);
    TEST(test_rx_ack_bg);
    TEST(test_rx_ack_st);
    TEST(test_rx_ack_sd);
    TEST(test_rx_ack_mp);
    TEST(test_rx_notification_alarm);
    TEST(test_rx_null_args);
    TEST(test_rx_ack_ic);
    TEST(test_rx_ack_lm_get);

    TEST_REPORT();
    TEST_EXIT();
}
