/*
 * Responsibility:
 * Unity coverage for the incoming motor RX parsing and matching layer.
 */

#include <string.h>

#include "unity.h"

#include "esp_err.h"
#include "motor_codec.h"
#include "motor_protocol.h"

static twai_message_t make_ext_frame(uint8_t producer_id,
                                     uint8_t cw_raw,
                                     uint8_t dlc,
                                     const uint8_t *data)
{
    twai_message_t msg = {
        .extd = 1,
        .rtr = 0,
        .ss = 0,
        .self = 0,
        .dlc_non_comp = 0,
        .identifier = motor_codec_build_ext_id(producer_id, cw_raw),
        .data_length_code = dlc,
    };

    if (data != NULL && dlc > 0U) {
        memcpy(msg.data, data, dlc);
    }

    return msg;
}

TEST_CASE("motor rx rejects structurally invalid frames", "[motor_protocol]")
{
    motor_rx_t rx;
    twai_message_t standard = {
        .extd = 0,
        .rtr = 0,
        .identifier = 0x123,
        .data_length_code = 0,
    };
    twai_message_t remote = {
        .extd = 1,
        .rtr = 1,
        .identifier = motor_codec_build_ext_id(0x05, 0x01),
        .data_length_code = 0,
    };
    twai_message_t bad_dlc = {
        .extd = 1,
        .rtr = 0,
        .identifier = motor_codec_build_ext_id(0x05, 0x01),
        .data_length_code = 9,
    };

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_rx_parse(NULL, &rx));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_rx_parse(&standard, &rx));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_rx_parse(&remote, &rx));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_rx_parse(&bad_dlc, &rx));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_rx_parse(&standard, NULL));
}

TEST_CASE("motor rx parses PP ACK", "[motor_protocol]")
{
    static const uint8_t data[] = {MOTOR_PP_INDEX_NODE_ID, 0x2A};
    twai_message_t msg = make_ext_frame(0x05, 0x01, 2U, data);
    motor_rx_t rx;
    motor_pp_index_t index;
    uint8_t value;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_is_ack(&rx));
    TEST_ASSERT_EQUAL(MOTOR_SECTION_PROTOCOL, rx.section);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_PP, rx.object);
    TEST_ASSERT_TRUE(motor_rx_ack_pp(&rx, &index, &value));
    TEST_ASSERT_EQUAL(MOTOR_PP_INDEX_NODE_ID, index);
    TEST_ASSERT_EQUAL_HEX8(0x2A, value);
}

TEST_CASE("motor rx parses IC ACK", "[motor_protocol]")
{
    static const uint8_t data[] = {MOTOR_IC_INDEX_USE_CLOSED_LOOP, 0x34, 0x12};
    twai_message_t msg = make_ext_frame(0x22, 0x06, 3U, data);
    motor_rx_t rx;
    motor_ic_index_t index;
    uint16_t value;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_ic(&rx, &index, &value));
    TEST_ASSERT_EQUAL(MOTOR_IC_INDEX_USE_CLOSED_LOOP, index);
    TEST_ASSERT_EQUAL_UINT16(0x1234, value);
}

TEST_CASE("motor rx parses IE QE IL and MP indexed ACKs", "[motor_protocol]")
{
    motor_rx_t rx;
    uint16_t value;
    motor_ie_index_t ie_index;
    motor_qe_index_t qe_index;
    motor_il_index_t il_index;
    motor_mp_index_t mp_index;
    twai_message_t ie_msg = make_ext_frame(0x22, 0x07, 3U,
                                           (const uint8_t[]){MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, 0x01, 0x00});
    twai_message_t qe_msg = make_ext_frame(0x22, 0x3D, 3U,
                                           (const uint8_t[]){MOTOR_QE_INDEX_BATTERY_VOLTAGE, 0xC0, 0x5D});
    twai_message_t il_msg = make_ext_frame(0x22, 0x34, 3U,
                                           (const uint8_t[]){MOTOR_IL_INDEX_STALL_BEHAVIOR, 0x02, 0x00});
    twai_message_t mp_msg = make_ext_frame(0x22, 0x22, 3U,
                                           (const uint8_t[]){MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE, 0x03, 0x00});

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&ie_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_ie(&rx, &ie_index, &value));
    TEST_ASSERT_EQUAL(MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, ie_index);
    TEST_ASSERT_EQUAL_UINT16(1U, value);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&qe_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_qe(&rx, &qe_index, &value));
    TEST_ASSERT_EQUAL(MOTOR_QE_INDEX_BATTERY_VOLTAGE, qe_index);
    TEST_ASSERT_EQUAL_UINT16(24000U, value);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&il_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_il(&rx, &il_index, &value));
    TEST_ASSERT_EQUAL(MOTOR_IL_INDEX_STALL_BEHAVIOR, il_index);
    TEST_ASSERT_EQUAL_UINT16(2U, value);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&mp_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_mp(&rx, &mp_index, &value));
    TEST_ASSERT_EQUAL(MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE, mp_index);
    TEST_ASSERT_EQUAL_UINT16(3U, value);
}

TEST_CASE("motor rx parses ER error frame and helpers", "[motor_protocol]")
{
    static const uint8_t data[] = {
        MOTOR_ER_INDEX_HISTORY_4, 0x91, 0xA4, 0x07, 0x00, 0x00
    };
    twai_message_t msg = make_ext_frame(0x7E, 0x0F, 6U, data);
    motor_rx_t rx;
    motor_er_index_t slot;
    uint8_t code;
    uint8_t related_cw;
    uint8_t related_base;
    uint8_t related_subindex;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_is_error(&rx));
    TEST_ASSERT_TRUE(motor_rx_ack_er(&rx, &slot, &code, &related_cw, &related_subindex));
    TEST_ASSERT_EQUAL(MOTOR_ER_INDEX_HISTORY_4, slot);
    TEST_ASSERT_EQUAL_HEX8(0x91, code);
    TEST_ASSERT_EQUAL_HEX8(0xA4, related_cw);
    TEST_ASSERT_EQUAL_HEX8(0x07, related_subindex);
    TEST_ASSERT_TRUE(motor_rx_error_slot(&rx, &code));
    TEST_ASSERT_EQUAL_HEX8(MOTOR_ER_INDEX_HISTORY_4, code);
    TEST_ASSERT_TRUE(motor_rx_error_code(&rx, &code));
    TEST_ASSERT_EQUAL_HEX8(0x91, code);
    TEST_ASSERT_TRUE(motor_rx_error_related_cw(&rx, &related_cw));
    TEST_ASSERT_TRUE(motor_rx_error_related_base_code(&rx, &related_base));
    TEST_ASSERT_TRUE(motor_rx_error_related_subindex(&rx, &related_subindex));
    TEST_ASSERT_EQUAL_HEX8(0x24, related_base);
    TEST_ASSERT_EQUAL_HEX8(0x07, related_subindex);
}

TEST_CASE("motor rx models ER responses as error kind with ER family accessor", "[motor_protocol]")
{
    static const uint8_t data[] = {
        MOTOR_ER_INDEX_HISTORY_1, 0x11, 0x8F, 0x00, 0x00, 0x00
    };
    twai_message_t msg = make_ext_frame(0x7E, 0x0F, 6U, data);
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_is_error(&rx));
    TEST_ASSERT_FALSE(motor_rx_is_ack(&rx));
    TEST_ASSERT_EQUAL(MOTOR_SECTION_SYSTEM, rx.section);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_ER, rx.object);
    TEST_ASSERT_FALSE(motor_rx_er_is_thrown_error(&rx));
    TEST_ASSERT_TRUE(motor_rx_er_is_history_response(&rx));
    TEST_ASSERT_TRUE(motor_rx_ack_er(&rx, NULL, NULL, NULL, NULL));
}

TEST_CASE("motor rx rejects ACK payloads with invalid returned family indices", "[motor_protocol]")
{
    static const uint8_t pp_data[] = {0x06, 0x2A};
    static const uint8_t ic_data[] = {0x02, 0x34, 0x12};
    static const uint8_t mt_data[] = {0x04, 0x60, 0xEA};
    uint8_t lm_data[5] = {0x05};
    twai_message_t pp_msg = make_ext_frame(0x05, 0x01, 2U, pp_data);
    twai_message_t ic_msg = make_ext_frame(0x22, 0x06, 3U, ic_data);
    twai_message_t mt_msg = make_ext_frame(0x35, 0x10, 3U, mt_data);
    twai_message_t lm_msg;
    motor_rx_t rx;

    motor_codec_pack_i32_le(&lm_data[1], 3333);
    lm_msg = make_ext_frame(0x2F, 0x2C, 5U, lm_data);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pp_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_pp(&rx, NULL, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&ic_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_ic(&rx, NULL, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&mt_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_mt(&rx, NULL, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&lm_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_lm_get(&rx, NULL, NULL));
}

TEST_CASE("motor rx parses SD and BL scalar ACKs", "[motor_protocol]")
{
    uint8_t sd_data[4];
    uint8_t bl_data[4];
    twai_message_t sd_msg;
    twai_message_t bl_msg;
    motor_rx_t rx;
    uint32_t value;

    motor_codec_pack_u32_le(sd_data, 123456U);
    motor_codec_pack_u32_le(bl_data, 65535U);
    sd_msg = make_ext_frame(0x22, 0x1C, 4U, sd_data);
    bl_msg = make_ext_frame(0x22, 0x2D, 4U, bl_data);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&sd_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_sd(&rx, &value));
    TEST_ASSERT_EQUAL_UINT32(123456U, value);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&bl_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_bl(&rx, &value));
    TEST_ASSERT_EQUAL_UINT32(65535U, value);
}

TEST_CASE("motor rx parses PV and PT ACK payloads with documented don't-care handling", "[motor_protocol]")
{
    twai_message_t pv_get_msg = make_ext_frame(0x06, 0x23, 8U,
                                               (const uint8_t[]){0x21, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF});
    twai_message_t pv_set_msg = make_ext_frame(0x06, 0x23, 2U,
                                               (const uint8_t[]){0x02, 0x00});
    twai_message_t pt_msg = make_ext_frame(0x06, 0x24, 8U,
                                           (const uint8_t[]){0x0A, 0x01, 0xA0, 0x86, 0x01, 0x00, 0x12, 0x34});
    motor_rx_t rx;
    uint16_t row;
    int32_t position;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pv_get_msg, &rx));
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_PV, rx.object);
    TEST_ASSERT_TRUE(motor_rx_ack_pv(&rx, &row));
    TEST_ASSERT_EQUAL_UINT16(33U, row);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pv_set_msg, &rx));
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_PV, rx.object);
    TEST_ASSERT_TRUE(motor_rx_ack_pv(&rx, &row));
    TEST_ASSERT_EQUAL_UINT16(2U, row);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pt_msg, &rx));
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_PT, rx.object);
    TEST_ASSERT_TRUE(motor_rx_ack_pt(&rx, &row, &position));
    TEST_ASSERT_EQUAL_UINT16(266U, row);
    TEST_ASSERT_EQUAL_INT32(100000, position);
}

TEST_CASE("motor rx disambiguates 0x24 ACK payloads between PT and LM using DLC", "[motor_protocol]")
{
    twai_message_t pt_msg = make_ext_frame(0x06, 0x24, 8U,
                                           (const uint8_t[]){0x0A, 0x01, 0xA0, 0x86, 0x01, 0x00, 0x12, 0x34});
    twai_message_t lm_msg = make_ext_frame(0x06, 0x24, 5U,
                                           (const uint8_t[]){MOTOR_LM_INDEX_MAX_ACCEL_DECEL, 0xD2, 0x1E, 0x00, 0x00});
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pt_msg, &rx));
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_PT, rx.object);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&lm_msg, &rx));
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_LM, rx.object);
}

TEST_CASE("motor rx keeps new ACK helpers strict on DLC base and index", "[motor_protocol]")
{
    twai_message_t ie_wrong_dlc_msg = make_ext_frame(0x22, 0x07, 2U,
                                                     (const uint8_t[]){MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, 0x01});
    twai_message_t ie_invalid_index_msg = make_ext_frame(0x22, 0x07, 3U,
                                                         (const uint8_t[]){0x09, 0x01, 0x00});
    twai_message_t qe_wrong_base_msg = make_ext_frame(0x22, 0x06, 3U,
                                                      (const uint8_t[]){MOTOR_QE_INDEX_BATTERY_VOLTAGE, 0xC0, 0x5D});
    twai_message_t il_invalid_index_msg = make_ext_frame(0x22, 0x34, 3U,
                                                         (const uint8_t[]){0x04, 0x02, 0x00});
    twai_message_t mp_wrong_dlc_msg = make_ext_frame(0x22, 0x22, 2U,
                                                     (const uint8_t[]){MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE, 0x01});
    twai_message_t sd_wrong_dlc_msg = make_ext_frame(0x22, 0x1C, 3U,
                                                     (const uint8_t[]){0xA8, 0xE2, 0x04});
    twai_message_t bl_wrong_base_msg = make_ext_frame(0x22, 0x1C, 4U,
                                                      (const uint8_t[]){0xFF, 0xFF, 0x00, 0x00});
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&ie_wrong_dlc_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_ie(&rx, NULL, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&ie_invalid_index_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_ie(&rx, NULL, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&qe_wrong_base_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_qe(&rx, NULL, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&il_invalid_index_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_il(&rx, NULL, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&mp_wrong_dlc_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_mp(&rx, NULL, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&sd_wrong_dlc_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_sd(&rx, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&bl_wrong_base_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_bl(&rx, NULL));
}

TEST_CASE("motor rx rejects ER helper on invalid ER slot", "[motor_protocol]")
{
    static const uint8_t data[] = {0x09, 0x91, 0xA4, 0x07, 0x00, 0x00};
    twai_message_t msg = make_ext_frame(0x7E, 0x0F, 6U, data);
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_is_error(&rx));
    TEST_ASSERT_FALSE(motor_rx_ack_er(&rx, NULL, NULL, NULL, NULL));
}

TEST_CASE("motor rx parses MT ACK", "[motor_protocol]")
{
    static const uint8_t data[] = {MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS, 0x60, 0xEA};
    twai_message_t msg = make_ext_frame(0x35, 0x10, 3U, data);
    motor_rx_t rx;
    motor_mt_index_t index;
    uint16_t value;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_mt(&rx, &index, &value));
    TEST_ASSERT_EQUAL(MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS, index);
    TEST_ASSERT_EQUAL_UINT16(60000U, value);
}

TEST_CASE("motor rx parses MO BG ST and OG ACKs", "[motor_protocol]")
{
    static const uint8_t mo_data[] = {MOTOR_MO_STATE_ENABLE};
    static const uint8_t bg_data[] = {0x00, 0x00, 0x00, 0x00};
    twai_message_t mo_msg = make_ext_frame(0x2A, 0x15, 1U, mo_data);
    twai_message_t bg_msg = make_ext_frame(0x09, 0x16, 4U, bg_data);
    twai_message_t st_msg = make_ext_frame(0x09, 0x17, 0U, NULL);
    twai_message_t og_msg = make_ext_frame(0x0A, 0x21, 0U, NULL);
    motor_rx_t rx;
    uint8_t state;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&mo_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_mo(&rx, &state));
    TEST_ASSERT_EQUAL_HEX8(MOTOR_MO_STATE_ENABLE, state);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&bg_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_bg(&rx));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&st_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_st(&rx));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&og_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_og(&rx));
}

TEST_CASE("motor rx rejects semantically invalid MO ACK state", "[motor_protocol]")
{
    static const uint8_t invalid_mo_data[] = {0x02};
    twai_message_t msg = make_ext_frame(0x2A, 0x15, 1U, invalid_mo_data);
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_mo(&rx, NULL));
}

TEST_CASE("motor rx parses PA GET and SET ACKs", "[motor_protocol]")
{
    uint8_t get_data[4];
    uint8_t set_data[5] = {4U};
    twai_message_t get_msg;
    twai_message_t set_msg;
    motor_rx_t rx;
    int32_t position;

    motor_codec_pack_i32_le(get_data, 123456);
    motor_codec_pack_i32_le(&set_data[1], -2222);
    get_msg = make_ext_frame(0x0A, 0x20, 4U, get_data);
    set_msg = make_ext_frame(0x0A, 0x2E, 5U, set_data);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&get_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_pa_get(&rx, &position));
    TEST_ASSERT_EQUAL_INT32(123456, position);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&set_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_pa_set(&rx, &position));
    TEST_ASSERT_EQUAL_INT32(-2222, position);
}

TEST_CASE("motor rx rejects bad PA set ACK shapes", "[motor_protocol]")
{
    uint8_t good_data[5] = {4U};
    uint8_t wrong_dlc_data[4] = {4U, 0x52, 0xF7, 0xFF};
    uint8_t wrong_subindex_data[5] = {3U};
    twai_message_t wrong_dlc_msg;
    twai_message_t wrong_subindex_msg;
    twai_message_t wrong_base_msg;
    motor_rx_t rx;

    motor_codec_pack_i32_le(&good_data[1], -2222);
    motor_codec_pack_i32_le(&wrong_subindex_data[1], -2222);
    wrong_dlc_msg = make_ext_frame(0x0A, 0x2E, 4U, wrong_dlc_data);
    wrong_subindex_msg = make_ext_frame(0x0A, 0x2E, 5U, wrong_subindex_data);
    wrong_base_msg = make_ext_frame(0x0A, 0x20, 5U, good_data);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&wrong_dlc_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_pa_set(&rx, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&wrong_subindex_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_pa_set(&rx, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&wrong_base_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_pa_set(&rx, NULL));
}

TEST_CASE("motor rx parses LM GET and SET ACKs", "[motor_protocol]")
{
    uint8_t get_data[5] = {MOTOR_LM_INDEX_MAX_ACCEL_DECEL};
    uint8_t set_data_legacy[5] = {MOTOR_LM_INDEX_MAX_ACCEL_DECEL};
    uint8_t set_data_actual[5] = {MOTOR_LM_INDEX_RESET_LIMITS};
    twai_message_t get_msg;
    twai_message_t set_msg_legacy;
    twai_message_t set_msg_actual;
    motor_rx_t rx;
    motor_lm_index_t index;
    int32_t value;

    motor_codec_pack_i32_le(&get_data[1], 3333);
    motor_codec_pack_i32_le(&set_data_legacy[1], -4444);
    motor_codec_pack_i32_le(&set_data_actual[1], 1);
    get_msg = make_ext_frame(0x2F, 0x2C, 5U, get_data);
    set_msg_legacy = make_ext_frame(0x2F, 0x24, 5U, set_data_legacy);
    set_msg_actual = make_ext_frame(0x2F, 0x2C, 5U, set_data_actual);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&get_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_lm_get(&rx, &index, &value));
    TEST_ASSERT_EQUAL(MOTOR_LM_INDEX_MAX_ACCEL_DECEL, index);
    TEST_ASSERT_EQUAL_INT32(3333, value);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&set_msg_legacy, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_lm_set(&rx, &index, &value));
    TEST_ASSERT_EQUAL(MOTOR_LM_INDEX_MAX_ACCEL_DECEL, index);
    TEST_ASSERT_EQUAL_INT32(-4444, value);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&set_msg_actual, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_lm_set(&rx, &index, &value));
    TEST_ASSERT_EQUAL(MOTOR_LM_INDEX_RESET_LIMITS, index);
    TEST_ASSERT_EQUAL_INT32(1, value);
}

TEST_CASE("motor rx rejects bad LM set ACK shapes", "[motor_protocol]")
{
    uint8_t good_data[5] = {MOTOR_LM_INDEX_MAX_ACCEL_DECEL};
    uint8_t wrong_dlc_data[4] = {MOTOR_LM_INDEX_MAX_ACCEL_DECEL, 0x5C, 0x11, 0x00};
    uint8_t invalid_index_data[5] = {0x05};
    twai_message_t wrong_dlc_msg;
    twai_message_t invalid_index_msg;
    twai_message_t wrong_base_msg;
    motor_rx_t rx;

    motor_codec_pack_i32_le(&good_data[1], 4444);
    motor_codec_pack_i32_le(&invalid_index_data[1], 4444);
    wrong_dlc_msg = make_ext_frame(0x2F, 0x24, 4U, wrong_dlc_data);
    invalid_index_msg = make_ext_frame(0x2F, 0x24, 5U, invalid_index_data);
    wrong_base_msg = make_ext_frame(0x2F, 0x34, 5U, good_data);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&wrong_dlc_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_lm_set(&rx, NULL, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&invalid_index_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_lm_set(&rx, NULL, NULL));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&wrong_base_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_ack_lm_set(&rx, NULL, NULL));
}

TEST_CASE("motor rx parses completion-style notification helpers and current position", "[motor_protocol]")
{
    uint8_t status_data[6] = {0x44, 0x02, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t alarm_data[] = {0x00, 0x99};
    twai_message_t status_msg;
    twai_message_t alarm_msg = make_ext_frame(0x11, 0x5A, 2U, alarm_data);
    motor_rx_t rx;
    uint8_t code;
    int32_t position;

    motor_codec_pack_i32_le(&status_data[2], 654321);
    status_msg = make_ext_frame(0x11, 0x5A, 6U, status_data);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&status_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_is_notification(&rx));
    TEST_ASSERT_TRUE(motor_rx_notification_is_status(&rx));
    TEST_ASSERT_TRUE(motor_rx_notification_code(&rx, &code));
    TEST_ASSERT_EQUAL_HEX8(0x44, code);
    TEST_ASSERT_TRUE(motor_rx_notification_current_position(&rx, &position));
    TEST_ASSERT_EQUAL_INT32(654321, position);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&alarm_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_notification_is_alarm(&rx));
    TEST_ASSERT_TRUE(motor_rx_notification_code(&rx, &code));
    TEST_ASSERT_EQUAL_HEX8(0x99, code);
    TEST_ASSERT_FALSE(motor_rx_notification_current_position(&rx, &position));
}

TEST_CASE("motor rx notification helpers stay strict on weak payload shapes", "[motor_protocol]")
{
    uint8_t long_alarm_data[6] = {0x00, 0x55, 0x78, 0x56, 0x34, 0x12};
    static const uint8_t short_status_data[] = {0x44, 0x02, 0xAA, 0xBB};
    static const uint8_t partial_status_data[] = {0x44};
    twai_message_t long_alarm_msg = make_ext_frame(0x11, 0x5A, 6U, long_alarm_data);
    twai_message_t short_status_msg = make_ext_frame(0x11, 0x5A, 4U, short_status_data);
    twai_message_t partial_status_msg = make_ext_frame(0x11, 0x5A, 1U, partial_status_data);
    motor_rx_t rx;
    uint8_t byte;
    int32_t position;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&long_alarm_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_notification_is_alarm(&rx));
    TEST_ASSERT_FALSE(motor_rx_notification_current_position(&rx, &position));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&short_status_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_notification_is_status(&rx));
    TEST_ASSERT_FALSE(motor_rx_notification_current_position(&rx, &position));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&partial_status_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_notification_d0(&rx, &byte));
    TEST_ASSERT_EQUAL_HEX8(0x44, byte);
    TEST_ASSERT_FALSE(motor_rx_notification_d1(&rx, &byte));
    TEST_ASSERT_TRUE(motor_rx_notification_is_status(&rx));
    TEST_ASSERT_TRUE(motor_rx_notification_code(&rx, &byte));
    TEST_ASSERT_EQUAL_HEX8(0x44, byte);
    TEST_ASSERT_FALSE(motor_rx_notification_current_position(&rx, &position));
}

TEST_CASE("motor rx matches symmetric ACK commands", "[motor_protocol]")
{
    static const uint8_t pp_data[] = {MOTOR_PP_INDEX_BITRATE, MOTOR_PP_BITRATE_250K};
    static const uint8_t ic_data[] = {MOTOR_IC_INDEX_USE_CLOSED_LOOP, 0x01, 0x00};
    twai_message_t pp_msg = make_ext_frame(0x05, 0x01, 2U, pp_data);
    twai_message_t ic_msg = make_ext_frame(0x22, 0x06, 3U, ic_data);
    motor_cmd_t pp_cmd;
    motor_cmd_t ic_cmd;
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_set_u8(0x05, false, MOTOR_PP_INDEX_BITRATE, MOTOR_PP_BITRATE_250K, &pp_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_ic_set_u16(0x22, true, MOTOR_IC_INDEX_USE_CLOSED_LOOP, 1U, &ic_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pp_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &pp_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&ic_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &ic_cmd));
}

TEST_CASE("motor rx matches asymmetric ACK commands", "[motor_protocol]")
{
    uint8_t pa_set_data[5] = {4U};
    uint8_t lm_set_data[5] = {MOTOR_LM_INDEX_MAX_ACCEL_DECEL};
    twai_message_t pa_msg;
    twai_message_t lm_msg;
    motor_cmd_t pa_cmd;
    motor_cmd_t lm_cmd;
    motor_rx_t rx;

    motor_codec_pack_i32_le(&pa_set_data[1], -123456);
    motor_codec_pack_i32_le(&lm_set_data[1], 7890);
    pa_msg = make_ext_frame(0x0A, 0x2E, 5U, pa_set_data);
    lm_msg = make_ext_frame(0x2F, 0x2C, 5U, lm_set_data);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pa_set(0x0A, true, -123456, &pa_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_lm_set_i32(0x2F, false, MOTOR_LM_INDEX_MAX_ACCEL_DECEL, 7890, &lm_cmd));
    TEST_ASSERT_EQUAL_UINT8(4U, pa_cmd.msg.data_length_code);
    TEST_ASSERT_EQUAL_UINT8(5U, lm_cmd.msg.data_length_code);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pa_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &pa_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&lm_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &lm_cmd));
}

TEST_CASE("motor rx matches new indexed and scalar command families", "[motor_protocol]")
{
    twai_message_t ie_msg = make_ext_frame(0x05, 0x07, 3U, (const uint8_t[]){0x08, 0x01, 0x00});
    twai_message_t qe_msg = make_ext_frame(0x05, 0x3D, 3U, (const uint8_t[]){0x03, 0xC0, 0x5D});
    twai_message_t il_msg = make_ext_frame(0x05, 0x34, 3U, (const uint8_t[]){0x03, 0x34, 0x12});
    twai_message_t mp_msg = make_ext_frame(0x05, 0x22, 3U, (const uint8_t[]){0x03, 0x01, 0x00});
    uint8_t sd_data[4];
    uint8_t bl_data[4];
    twai_message_t sd_msg;
    twai_message_t bl_msg;
    motor_cmd_t ie_cmd;
    motor_cmd_t qe_cmd;
    motor_cmd_t il_cmd;
    motor_cmd_t mp_cmd;
    motor_cmd_t sd_cmd;
    motor_cmd_t bl_cmd;
    motor_rx_t rx;

    motor_codec_pack_u32_le(sd_data, 321000U);
    motor_codec_pack_u32_le(bl_data, 70000U);
    sd_msg = make_ext_frame(0x05, 0x1C, 4U, sd_data);
    bl_msg = make_ext_frame(0x05, 0x2D, 4U, bl_data);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_ie_set_u16(0x05, true, MOTOR_IE_INDEX_PTP_POSITIONING_FINISH_NOTIFICATION, 1U, &ie_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_qe_set_u16(0x05, true, MOTOR_QE_INDEX_BATTERY_VOLTAGE, 24000U, &qe_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_il_set_u16(0x05, true, MOTOR_IL_INDEX_STALL_BEHAVIOR, 0x1234U, &il_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mp_set_u16(0x05, true, MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE, 1U, &mp_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_sd_set_u32(0x05, true, 321000U, &sd_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bl_set_u32(0x05, true, 70000U, &bl_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&ie_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &ie_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&qe_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &qe_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&il_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &il_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&mp_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &mp_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&sd_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &sd_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&bl_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &bl_cmd));
}

TEST_CASE("motor rx matches PT and PV commands using only the contractually meaningful bytes", "[motor_protocol]")
{
    twai_message_t pv_get_msg = make_ext_frame(0x06, 0x23, 8U,
                                               (const uint8_t[]){0x21, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF});
    twai_message_t pv_set_msg = make_ext_frame(0x06, 0x23, 8U,
                                               (const uint8_t[]){0x02, 0x00, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF});
    twai_message_t pt_get_msg = make_ext_frame(0x06, 0x24, 8U,
                                               (const uint8_t[]){0x0A, 0x01, 0xA0, 0x86, 0x01, 0x00, 0x12, 0x34});
    twai_message_t pt_set_msg = make_ext_frame(0x06, 0x24, 8U,
                                               (const uint8_t[]){0x0A, 0x01, 0xA0, 0x86, 0x01, 0x00, 0x56, 0x78});
    motor_cmd_t pv_get_cmd;
    motor_cmd_t pv_set_cmd;
    motor_cmd_t pt_get_cmd;
    motor_cmd_t pt_set_cmd;
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pv_get(0x06, true, &pv_get_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pv_set(0x06, true, 2U, &pv_set_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pt_get(0x06, true, 266U, &pt_get_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pt_set(0x06, true, 266U, 100000, &pt_set_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pv_get_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &pv_get_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pv_set_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &pv_set_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pt_get_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &pt_get_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pt_set_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &pt_set_cmd));
}

TEST_CASE("motor rx rejects PT and PV mismatches on meaningful bytes only", "[motor_protocol]")
{
    twai_message_t pv_wrong_value_msg = make_ext_frame(0x06, 0x23, 2U, (const uint8_t[]){0x03, 0x00});
    twai_message_t pt_wrong_row_msg = make_ext_frame(0x06, 0x24, 8U,
                                                     (const uint8_t[]){0x0B, 0x01, 0xA0, 0x86, 0x01, 0x00, 0x00, 0x00});
    twai_message_t pt_wrong_position_msg = make_ext_frame(0x06, 0x24, 8U,
                                                          (const uint8_t[]){0x0A, 0x01, 0xA1, 0x86, 0x01, 0x00, 0x00, 0x00});
    motor_cmd_t pv_set_cmd;
    motor_cmd_t pt_set_cmd;
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pv_set(0x06, true, 2U, &pv_set_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pt_set(0x06, true, 266U, 100000, &pt_set_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pv_wrong_value_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &pv_set_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pt_wrong_row_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &pt_set_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pt_wrong_position_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &pt_set_cmd));
}

TEST_CASE("motor rx keeps PT GET strict on row match while relaxing PT SET row echo", "[motor_protocol]")
{
    twai_message_t pt_wrong_row_msg = make_ext_frame(0x06, 0x24, 8U,
                                                     (const uint8_t[]){0x0B, 0x01, 0xA0, 0x86, 0x01, 0x00, 0x00, 0x00});
    motor_cmd_t pt_get_cmd;
    motor_cmd_t pt_set_cmd;
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pt_get(0x06, true, 266U, &pt_get_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pt_set(0x06, true, 266U, 100000, &pt_set_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pt_wrong_row_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &pt_get_cmd));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &pt_set_cmd));
}

TEST_CASE("motor rx matches ER queried-history responses using exact d0 semantics", "[motor_protocol]")
{
    twai_message_t er_thrown_msg = make_ext_frame(0x7E, 0x0F, 6U,
                                                  (const uint8_t[]){MOTOR_ER_INDEX_CLEAR_ALL, 0x11, 0x8F, 0x00, 0x00, 0x00});
    twai_message_t er_10_msg = make_ext_frame(0x7E, 0x0F, 6U,
                                              (const uint8_t[]){MOTOR_ER_INDEX_HISTORY_1, 0x11, 0x8F, 0x00, 0x00, 0x00});
    twai_message_t er_11_msg = make_ext_frame(0x7E, 0x0F, 6U,
                                              (const uint8_t[]){MOTOR_ER_INDEX_HISTORY_2, 0x22, 0x8F, 0x00, 0x00, 0x00});
    twai_message_t er_18_msg = make_ext_frame(0x7E, 0x0F, 6U,
                                              (const uint8_t[]){MOTOR_ER_INDEX_HISTORY_9, 0x33, 0x8F, 0x00, 0x00, 0x00});
    motor_cmd_t er_get_invalid_cmd;
    motor_cmd_t er_get_10_cmd;
    motor_cmd_t er_get_11_cmd;
    motor_cmd_t er_get_18_cmd;
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      motor_cmd_er_get(0x7E, true, MOTOR_ER_INDEX_CLEAR_ALL, &er_get_invalid_cmd));
    TEST_ASSERT_EQUAL(ESP_OK,
                      motor_cmd_er_get(0x7E, true, MOTOR_ER_INDEX_HISTORY_1, &er_get_10_cmd));
    TEST_ASSERT_EQUAL(ESP_OK,
                      motor_cmd_er_get(0x7E, true, MOTOR_ER_INDEX_HISTORY_2, &er_get_11_cmd));
    TEST_ASSERT_EQUAL(ESP_OK,
                      motor_cmd_er_get(0x7E, true, MOTOR_ER_INDEX_HISTORY_9, &er_get_18_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&er_thrown_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_er_is_thrown_error(&rx));
    TEST_ASSERT_FALSE(motor_rx_er_is_history_response(&rx));
    TEST_ASSERT_FALSE(motor_rx_ack_er(&rx, NULL, NULL, NULL, NULL));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &er_get_10_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&er_10_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_er_is_thrown_error(&rx));
    TEST_ASSERT_TRUE(motor_rx_er_is_history_response(&rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &er_get_10_cmd));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &er_get_11_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&er_11_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &er_get_11_cmd));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &er_get_10_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&er_18_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &er_get_18_cmd));
}

TEST_CASE("motor rx matches ER clear-all completion only for the exact zero ACK shape", "[motor_protocol]")
{
    twai_message_t clear_ack_msg = make_ext_frame(0x7E, 0x0F, 6U,
                                                  (const uint8_t[]){0x00, 0x00, 0x00, 0x00, 0x00, 0x00});
    twai_message_t thrown_error_msg = make_ext_frame(0x7E, 0x0F, 6U,
                                                     (const uint8_t[]){0x00, 0x33, 0x8F, 0x00, 0x00, 0x00});
    motor_cmd_t clear_cmd;
    motor_cmd_t get_cmd;
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_er_clear_all(0x7E, true, &clear_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_er_get(0x7E, true, MOTOR_ER_INDEX_HISTORY_1, &get_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&clear_ack_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_er_is_clear_all_response(&rx));
    TEST_ASSERT_FALSE(motor_rx_er_is_thrown_error(&rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &clear_cmd));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &get_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&thrown_error_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_er_is_clear_all_response(&rx));
    TEST_ASSERT_TRUE(motor_rx_er_is_thrown_error(&rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &clear_cmd));
}

TEST_CASE("motor rx rejects symmetric and asymmetric command-response mismatches", "[motor_protocol]")
{
    static const uint8_t pp_wrong_producer_data[] = {MOTOR_PP_INDEX_BITRATE, MOTOR_PP_BITRATE_250K};
    static const uint8_t pp_wrong_index_data[] = {MOTOR_PP_INDEX_NODE_ID, MOTOR_PP_BITRATE_250K};
    static const uint8_t pp_wrong_value_data[] = {MOTOR_PP_INDEX_BITRATE, MOTOR_PP_BITRATE_1M};
    uint8_t pa_wrong_subindex_data[5] = {3U};
    uint8_t lm_wrong_index_data[5] = {MOTOR_LM_INDEX_MAX_POSITION_ERROR};
    uint8_t lm_wrong_value_data[5] = {MOTOR_LM_INDEX_MAX_ACCEL_DECEL};
    twai_message_t pp_wrong_producer_msg = make_ext_frame(0x06, 0x01, 2U, pp_wrong_producer_data);
    twai_message_t pp_wrong_index_msg = make_ext_frame(0x05, 0x01, 2U, pp_wrong_index_data);
    twai_message_t pp_wrong_value_msg = make_ext_frame(0x05, 0x01, 2U, pp_wrong_value_data);
    twai_message_t pa_wrong_subindex_msg;
    twai_message_t lm_wrong_index_msg;
    twai_message_t lm_wrong_value_msg;
    motor_cmd_t pp_cmd;
    motor_cmd_t pa_cmd;
    motor_cmd_t lm_cmd;
    motor_rx_t rx;

    motor_codec_pack_i32_le(&pa_wrong_subindex_data[1], -123456);
    motor_codec_pack_i32_le(&lm_wrong_index_data[1], 7890);
    motor_codec_pack_i32_le(&lm_wrong_value_data[1], 7891);
    pa_wrong_subindex_msg = make_ext_frame(0x0A, 0x2E, 5U, pa_wrong_subindex_data);
    lm_wrong_index_msg = make_ext_frame(0x2F, 0x24, 5U, lm_wrong_index_data);
    lm_wrong_value_msg = make_ext_frame(0x2F, 0x24, 5U, lm_wrong_value_data);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_set_u8(0x05, false, MOTOR_PP_INDEX_BITRATE, MOTOR_PP_BITRATE_250K, &pp_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pa_set(0x0A, true, -123456, &pa_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_lm_set_i32(0x2F, false, MOTOR_LM_INDEX_MAX_ACCEL_DECEL, 7890, &lm_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pp_wrong_producer_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &pp_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pp_wrong_index_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &pp_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pp_wrong_value_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &pp_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pa_wrong_subindex_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &pa_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&lm_wrong_index_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &lm_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&lm_wrong_value_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &lm_cmd));
}

TEST_CASE("motor rx rejects mismatches for new indexed and scalar command families", "[motor_protocol]")
{
    twai_message_t ie_wrong_index_msg = make_ext_frame(0x05, 0x07, 3U, (const uint8_t[]){0x0A, 0x01, 0x00});
    twai_message_t ie_wrong_value_msg = make_ext_frame(0x05, 0x07, 3U, (const uint8_t[]){0x08, 0x00, 0x00});
    twai_message_t qe_wrong_value_msg = make_ext_frame(0x05, 0x3D, 3U, (const uint8_t[]){0x03, 0x00, 0x00});
    twai_message_t il_wrong_value_msg = make_ext_frame(0x05, 0x34, 3U, (const uint8_t[]){0x03, 0x35, 0x12});
    twai_message_t mp_wrong_value_msg = make_ext_frame(0x05, 0x22, 3U, (const uint8_t[]){0x03, 0x00, 0x00});
    twai_message_t sd_wrong_producer_msg;
    twai_message_t sd_wrong_value_msg;
    twai_message_t bl_wrong_value_msg;
    uint8_t sd_wrong_producer_data[4];
    uint8_t sd_wrong_value_data[4];
    uint8_t bl_wrong_value_data[4];
    motor_cmd_t ie_cmd;
    motor_cmd_t qe_cmd;
    motor_cmd_t il_cmd;
    motor_cmd_t mp_cmd;
    motor_cmd_t sd_cmd;
    motor_cmd_t bl_cmd;
    motor_rx_t rx;

    motor_codec_pack_u32_le(sd_wrong_producer_data, 321000U);
    motor_codec_pack_u32_le(sd_wrong_value_data, 321001U);
    motor_codec_pack_u32_le(bl_wrong_value_data, 70001U);
    sd_wrong_producer_msg = make_ext_frame(0x06, 0x1C, 4U, sd_wrong_producer_data);
    sd_wrong_value_msg = make_ext_frame(0x05, 0x1C, 4U, sd_wrong_value_data);
    bl_wrong_value_msg = make_ext_frame(0x05, 0x2D, 4U, bl_wrong_value_data);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_ie_set_u16(0x05, true, MOTOR_IE_INDEX_PTP_POSITIONING_FINISH_NOTIFICATION, 1U, &ie_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_qe_set_u16(0x05, true, MOTOR_QE_INDEX_BATTERY_VOLTAGE, 24000U, &qe_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_il_set_u16(0x05, true, MOTOR_IL_INDEX_STALL_BEHAVIOR, 0x1234U, &il_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mp_set_u16(0x05, true, MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE, 1U, &mp_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_sd_set_u32(0x05, true, 321000U, &sd_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bl_set_u32(0x05, true, 70000U, &bl_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&ie_wrong_index_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &ie_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&ie_wrong_value_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &ie_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&qe_wrong_value_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &qe_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&il_wrong_value_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &il_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&mp_wrong_value_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &mp_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&sd_wrong_producer_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &sd_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&sd_wrong_value_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &sd_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&bl_wrong_value_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &bl_cmd));
}

TEST_CASE("motor rx matches global-addressed PP GET ACK from actual producer", "[motor_protocol]")
{
    static const uint8_t pp_data[] = {MOTOR_PP_INDEX_NODE_ID, 0x2A};
    twai_message_t pp_msg = make_ext_frame(0x01, 0x01, 2U, pp_data);
    motor_cmd_t pp_cmd;
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_get(0x00, true, MOTOR_PP_INDEX_NODE_ID, &pp_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pp_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_matches_cmd(&rx, &pp_cmd));
}

TEST_CASE("motor rx keeps direct-addressed producer matching strict", "[motor_protocol]")
{
    static const uint8_t pp_data[] = {MOTOR_PP_INDEX_NODE_ID, 0x2A};
    twai_message_t pp_msg = make_ext_frame(0x06, 0x01, 2U, pp_data);
    motor_cmd_t pp_cmd;
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_get(0x05, true, MOTOR_PP_INDEX_NODE_ID, &pp_cmd));
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pp_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &pp_cmd));
}

TEST_CASE("motor rx keeps global-addressed matching strict on response shape", "[motor_protocol]")
{
    static const uint8_t pp_wrong_index_data[] = {MOTOR_PP_INDEX_GROUP_ID, 0x2A};
    static const uint8_t pp_wrong_family_data[] = {MOTOR_IC_INDEX_USE_CLOSED_LOOP, 0x01, 0x00};
    twai_message_t pp_wrong_index_msg = make_ext_frame(0x01, 0x01, 2U, pp_wrong_index_data);
    twai_message_t pp_wrong_family_msg = make_ext_frame(0x01, 0x06, 3U, pp_wrong_family_data);
    motor_cmd_t pp_cmd;
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_get(0x00, true, MOTOR_PP_INDEX_NODE_ID, &pp_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pp_wrong_index_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &pp_cmd));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&pp_wrong_family_msg, &rx));
    TEST_ASSERT_FALSE(motor_rx_matches_cmd(&rx, &pp_cmd));
}

TEST_CASE("motor rx parses manual PP ACK examples", "[motor_protocol]")
{
    typedef struct {
        motor_pp_index_t index;
        uint8_t value;
    } pp_case_t;
    static const pp_case_t cases[] = {
        {MOTOR_PP_INDEX_BITRATE, 0x03},
        {MOTOR_PP_INDEX_NODE_ID, 0x05},
        {MOTOR_PP_INDEX_GROUP_ID, 0x0A},
        {MOTOR_PP_INDEX_BITRATE, 0x02},
        {MOTOR_PP_INDEX_NODE_ID, 0x0B},
        {MOTOR_PP_INDEX_GROUP_ID, 0x14},
    };
    size_t i;

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        twai_message_t msg = make_ext_frame(0x05, 0x01, 2U,
                                            (const uint8_t[]){(uint8_t)cases[i].index, cases[i].value});
        motor_rx_t rx;
        motor_pp_index_t index;
        uint8_t value;

        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
        TEST_ASSERT_TRUE(motor_rx_ack_pp(&rx, &index, &value));
        TEST_ASSERT_EQUAL(cases[i].index, index);
        TEST_ASSERT_EQUAL_HEX8(cases[i].value, value);
    }
}

TEST_CASE("motor rx parses manual IC ACK examples", "[motor_protocol]")
{
    typedef struct {
        motor_ic_index_t index;
        uint16_t value;
    } ic_case_t;
    static const ic_case_t cases[] = {
        {MOTOR_IC_INDEX_AUTO_ENABLE_AFTER_POWERUP, 0x0000},
        {MOTOR_IC_INDEX_POSITIVE_MOTOR_DIRECTION, 0x0001},
        {MOTOR_IC_INDEX_ENABLE_LOCKDOWN_SYSTEM, 0x0001},
        {MOTOR_IC_INDEX_AC_DC_UNITS, 0x0001},
        {MOTOR_IC_INDEX_USE_CLOSED_LOOP, 0x0001},
        {MOTOR_IC_INDEX_SOFTWARE_LIMIT_ENABLE, 0x0001},
        {MOTOR_IC_INDEX_INTERNAL_BRAKE_LOGIC_ENABLE, 0x0001},
        {MOTOR_IC_INDEX_USE_INTERNAL_BRAKE, 0x0001},
    };
    size_t i;

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        twai_message_t msg = make_ext_frame(0x05, 0x06, 3U,
                                            (const uint8_t[]){(uint8_t)cases[i].index,
                                                              (uint8_t)(cases[i].value & 0xFFU),
                                                              (uint8_t)(cases[i].value >> 8)});
        motor_rx_t rx;
        motor_ic_index_t index;
        uint16_t value;

        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
        TEST_ASSERT_TRUE(motor_rx_ack_ic(&rx, &index, &value));
        TEST_ASSERT_EQUAL(cases[i].index, index);
        TEST_ASSERT_EQUAL_UINT16(cases[i].value, value);
    }
}

TEST_CASE("motor rx parses manual ER ACK examples", "[motor_protocol]")
{
    twai_message_t history_msg = make_ext_frame(0x05, 0x0F, 6U,
                                                (const uint8_t[]){0x0A, 0x33, 0x81, 0x05, 0x00, 0x00});
    twai_message_t clear_ack_msg = make_ext_frame(0x05, 0x0F, 6U,
                                                  (const uint8_t[]){MOTOR_ER_INDEX_CLEAR_ALL, 0x00, 0x00, 0x00, 0x00, 0x00});
    twai_message_t thrown_msg = make_ext_frame(0x05, 0x0F, 6U,
                                               (const uint8_t[]){MOTOR_ER_INDEX_CLEAR_ALL, 0x33, 0x81, 0x05, 0x00, 0x00});
    motor_rx_t rx;
    motor_er_index_t slot;
    uint8_t code;
    uint8_t related_cw;
    uint8_t related_subindex;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&history_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_er(&rx, &slot, &code, &related_cw, &related_subindex));
    TEST_ASSERT_EQUAL(MOTOR_ER_INDEX_HISTORY_1, slot);
    TEST_ASSERT_EQUAL_HEX8(0x33, code);
    TEST_ASSERT_EQUAL_HEX8(0x81, related_cw);
    TEST_ASSERT_EQUAL_HEX8(0x05, related_subindex);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&clear_ack_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_er_is_clear_all_response(&rx));
    TEST_ASSERT_FALSE(motor_rx_er_is_thrown_error(&rx));
    TEST_ASSERT_FALSE(motor_rx_ack_er(&rx, &slot, &code, &related_cw, &related_subindex));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&thrown_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_er_is_thrown_error(&rx));
    TEST_ASSERT_FALSE(motor_rx_ack_er(&rx, &slot, &code, &related_cw, &related_subindex));
}

TEST_CASE("motor rx parses manual MT MO BG ST PA OG and LM ACK examples", "[motor_protocol]")
{
    twai_message_t msg;
    motor_rx_t rx;
    motor_mt_index_t mt_index;
    motor_lm_index_t lm_index;
    uint16_t mt_value;
    uint8_t mo_state;
    int32_t i32_value;

    msg = make_ext_frame(0x05, 0x10, 3U, (const uint8_t[]){0x00, 0x10, 0x00});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_mt(&rx, &mt_index, &mt_value));
    TEST_ASSERT_EQUAL(MOTOR_MT_INDEX_MICROSTEP, mt_index);
    TEST_ASSERT_EQUAL_UINT16(16U, mt_value);

    msg = make_ext_frame(0x05, 0x10, 3U, (const uint8_t[]){0x01, 0x1C, 0x00});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_mt(&rx, &mt_index, &mt_value));
    TEST_ASSERT_EQUAL(MOTOR_MT_INDEX_WORKING_CURRENT, mt_index);
    TEST_ASSERT_EQUAL_UINT16(28U, mt_value);

    msg = make_ext_frame(0x05, 0x10, 3U, (const uint8_t[]){0x02, 0x32, 0x00});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_mt(&rx, &mt_index, &mt_value));
    TEST_ASSERT_EQUAL(MOTOR_MT_INDEX_IDLE_CURRENT_PERCENT, mt_index);
    TEST_ASSERT_EQUAL_UINT16(50U, mt_value);

    msg = make_ext_frame(0x05, 0x10, 3U, (const uint8_t[]){0x03, 0xE8, 0x03});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_mt(&rx, &mt_index, &mt_value));
    TEST_ASSERT_EQUAL(MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS, mt_index);
    TEST_ASSERT_EQUAL_UINT16(1000U, mt_value);

    msg = make_ext_frame(0x05, 0x10, 3U, (const uint8_t[]){0x05, 0x01, 0x00});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_mt(&rx, &mt_index, &mt_value));
    TEST_ASSERT_EQUAL(MOTOR_MT_INDEX_BRAKE_STATE, mt_index);
    TEST_ASSERT_EQUAL_UINT16(1U, mt_value);

    msg = make_ext_frame(0x05, 0x15, 1U, (const uint8_t[]){0x01});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_mo(&rx, &mo_state));
    TEST_ASSERT_EQUAL_HEX8(0x01, mo_state);

    msg = make_ext_frame(0x05, 0x15, 1U, (const uint8_t[]){0x00});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_mo(&rx, &mo_state));
    TEST_ASSERT_EQUAL_HEX8(0x00, mo_state);

    msg = make_ext_frame(0x05, 0x16, 0U, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_bg(&rx));

    msg = make_ext_frame(0x05, 0x17, 0U, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_ack_st(&rx));

    {
        uint8_t pa_get_data[4];
        uint8_t pa_set_data[5] = {0x04};
        uint8_t lm_get_data[5] = {0x01};
        uint8_t lm_set_data[5] = {0x02};
        uint8_t lm_enable_data[5] = {0xFF};

        motor_codec_pack_i32_le(pa_get_data, -1000);
        motor_codec_pack_i32_le(&pa_set_data[1], 1000);
        motor_codec_pack_i32_le(&lm_get_data[1], 4124);
        motor_codec_pack_i32_le(&lm_set_data[1], 10000);
        motor_codec_pack_i32_le(&lm_enable_data[1], 0);

        msg = make_ext_frame(0x05, 0x20, 4U, pa_get_data);
        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
        TEST_ASSERT_TRUE(motor_rx_ack_pa_get(&rx, &i32_value));
        TEST_ASSERT_EQUAL_INT32(-1000, i32_value);

        msg = make_ext_frame(0x05, 0x2E, 5U, pa_set_data);
        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
        TEST_ASSERT_TRUE(motor_rx_ack_pa_set(&rx, &i32_value));
        TEST_ASSERT_EQUAL_INT32(1000, i32_value);

        msg = make_ext_frame(0x05, 0x21, 0U, NULL);
        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
        TEST_ASSERT_TRUE(motor_rx_ack_og(&rx));

        msg = make_ext_frame(0x05, 0x2C, 5U, lm_get_data);
        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
        TEST_ASSERT_TRUE(motor_rx_ack_lm_get(&rx, &lm_index, &i32_value));
        TEST_ASSERT_EQUAL(MOTOR_LM_INDEX_LOWER_WORKING_LIMIT, lm_index);
        TEST_ASSERT_EQUAL_INT32(4124, i32_value);

        msg = make_ext_frame(0x05, 0x2C, 5U, lm_set_data);
        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
        TEST_ASSERT_TRUE(motor_rx_ack_lm_set(&rx, &lm_index, &i32_value));
        TEST_ASSERT_EQUAL(MOTOR_LM_INDEX_UPPER_WORKING_LIMIT, lm_index);
        TEST_ASSERT_EQUAL_INT32(10000, i32_value);

        msg = make_ext_frame(0x05, 0x2C, 5U, lm_enable_data);
        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
        TEST_ASSERT_TRUE(motor_rx_ack_lm_set(&rx, &lm_index, &i32_value));
        TEST_ASSERT_EQUAL(MOTOR_LM_INDEX_ENABLE_DISABLE, lm_index);
        TEST_ASSERT_EQUAL_INT32(0, i32_value);
    }
}

TEST_CASE("motor rx parses RT alarm input edge and completion notifications from manual families", "[motor_protocol]")
{
    twai_message_t alarm_msg = make_ext_frame(0x05, 0x5A, 2U, (const uint8_t[]){0x00, 0x01});
    twai_message_t falling_msg = make_ext_frame(0x05, 0x5A, 2U, (const uint8_t[]){0x11, 0x00});
    twai_message_t rising_msg = make_ext_frame(0x05, 0x5A, 2U, (const uint8_t[]){0x12, 0x00});
    uint8_t completion_data[6] = {0x29, 0x00, 0x00, 0x00, 0x00, 0x00};
    motor_rx_t rx;
    uint8_t code;
    int32_t position;

    motor_codec_pack_i32_le(&completion_data[2], 1000);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&alarm_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_notification_is_alarm(&rx));
    TEST_ASSERT_TRUE(motor_rx_notification_code(&rx, &code));
    TEST_ASSERT_EQUAL_HEX8(0x01, code);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&falling_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_notification_is_status(&rx));
    TEST_ASSERT_TRUE(motor_rx_notification_code(&rx, &code));
    TEST_ASSERT_EQUAL_HEX8(0x11, code);
    TEST_ASSERT_FALSE(motor_rx_notification_current_position(&rx, &position));

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&rising_msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_notification_is_status(&rx));
    TEST_ASSERT_TRUE(motor_rx_notification_code(&rx, &code));
    TEST_ASSERT_EQUAL_HEX8(0x12, code);

    {
        twai_message_t completion_msg = make_ext_frame(0x05, 0x5A, 6U, completion_data);
        TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&completion_msg, &rx));
    }
    TEST_ASSERT_TRUE(motor_rx_notification_is_status(&rx));
    TEST_ASSERT_TRUE(motor_rx_notification_code(&rx, &code));
    TEST_ASSERT_EQUAL_HEX8(0x29, code);
    TEST_ASSERT_TRUE(motor_rx_notification_current_position(&rx, &position));
    TEST_ASSERT_EQUAL_INT32(1000, position);
}

TEST_CASE("motor rx parses unknown valid extended frame as unclassified", "[motor_protocol]")
{
    static const uint8_t data[] = {0xAA, 0x55};
    twai_message_t msg = make_ext_frame(0x33, 0x33, 2U, data);
    motor_rx_t rx;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_rx_is_unclassified(&rx));
    TEST_ASSERT_EQUAL(MOTOR_SECTION_NONE, rx.section);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_NONE, rx.object);
}
