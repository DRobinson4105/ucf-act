/*
 * Responsibility:
 * Unity coverage for the outgoing motor command builder layer.
 */

#include <inttypes.h>
#include "unity.h"

#include "esp_err.h"
#include "motor_codec.h"
#include "motor_protocol.h"
#include "test_log.h"

static void assert_frame_core(const motor_cmd_t *cmd,
                              uint8_t target_id,
                              uint8_t expected_base_code,
                              bool expected_ack,
                              uint8_t expected_dlc)
{
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL_HEX8(expected_dlc, cmd->msg.data_length_code);
    TEST_ASSERT_EQUAL_HEX8(target_id, cmd->target_id);
    TEST_ASSERT_EQUAL_HEX8(expected_base_code, motor_codec_base_code(cmd->cw_raw));
    TEST_ASSERT_EQUAL(expected_ack, motor_codec_ack_requested(cmd->cw_raw));
    TEST_ASSERT_EQUAL_HEX8(expected_base_code, cmd->base_code);
    TEST_ASSERT_EQUAL(expected_ack, cmd->ack_requested);
    TEST_ASSERT_EQUAL_HEX8(target_id, motor_codec_decode_id(cmd->msg.identifier));
    TEST_ASSERT_EQUAL_HEX8(cmd->cw_raw, motor_codec_decode_cw(cmd->msg.identifier));
    TEST_ASSERT_EQUAL_HEX32(motor_codec_build_ext_id(target_id, cmd->cw_raw), cmd->msg.identifier);
}

TEST_CASE("motor cmd builds PP get frame", "[motor_protocol]")
{
    LOG_SECTION("motor cmd PP get");
    motor_cmd_t cmd;
    esp_err_t err = motor_cmd_pp_get(0x05, true, MOTOR_PP_INDEX_NODE_ID, &cmd);

    LOG_INPUT("target_id=0x%02X ack_requested=%d index=%u", 0x05, true, MOTOR_PP_INDEX_NODE_ID);
    LOG_OUTPUT("err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u data=[%02X]",
               esp_err_to_name(err), cmd.cw_raw, cmd.msg.identifier, cmd.msg.data_length_code, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    assert_frame_core(&cmd, 0x05, 0x01, true, 1U);
    TEST_ASSERT_TRUE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_PP_INDEX_NODE_ID, cmd.index);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_PP_INDEX_NODE_ID, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_STRING("PP_GET", cmd.name);
}

TEST_CASE("motor cmd builds PP set u8 frame", "[motor_protocol]")
{
    LOG_SECTION("motor cmd PP set u8");
    motor_cmd_t cmd;
    esp_err_t err = motor_cmd_pp_set_u8(0x05, false, MOTOR_PP_INDEX_BITRATE, MOTOR_PP_BITRATE_250K, &cmd);

    LOG_INPUT("target_id=0x%02X ack_requested=%d index=%u value=%u",
              0x05, false, MOTOR_PP_INDEX_BITRATE, MOTOR_PP_BITRATE_250K);
    LOG_OUTPUT("err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u data=[%02X %02X]",
               esp_err_to_name(err), cmd.cw_raw, cmd.msg.identifier, cmd.msg.data_length_code,
               cmd.msg.data[0], cmd.msg.data[1]);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    assert_frame_core(&cmd, 0x05, 0x01, false, 2U);
    TEST_ASSERT_TRUE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_PP_INDEX_BITRATE, cmd.index);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_PP_INDEX_BITRATE, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_PP_BITRATE_250K, cmd.msg.data[1]);
    TEST_ASSERT_EQUAL_STRING("PP_SET_U8", cmd.name);
}

TEST_CASE("motor cmd rejects invalid PP values", "[motor_protocol]")
{
    LOG_SECTION("motor cmd PP validation");
    motor_cmd_t cmd;
    esp_err_t err_bad_bitrate = motor_cmd_pp_set_u8(0x05, true, MOTOR_PP_INDEX_BITRATE, 5U, &cmd);
    esp_err_t err_bad_target = motor_cmd_pp_get(0x04, true, MOTOR_PP_INDEX_NODE_ID, &cmd);

    LOG_INPUT("invalid PP bitrate value=5");
    LOG_ERROR("err=%s", esp_err_to_name(err_bad_bitrate));
    LOG_INPUT("invalid target_id=0x04");
    LOG_ERROR("err=%s", esp_err_to_name(err_bad_target));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err_bad_bitrate);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err_bad_target);
}

TEST_CASE("motor cmd builds IC set u16 frame", "[motor_protocol]")
{
    LOG_SECTION("motor cmd IC set u16");
    motor_cmd_t cmd;
    esp_err_t err = motor_cmd_ic_set_u16(0x22, true, MOTOR_IC_INDEX_USE_CLOSED_LOOP, 1U, &cmd);

    LOG_INPUT("target_id=0x%02X ack_requested=%d index=%u value=%u",
              0x22, true, MOTOR_IC_INDEX_USE_CLOSED_LOOP, 1U);
    LOG_OUTPUT("err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u data=[%02X %02X %02X]",
               esp_err_to_name(err), cmd.cw_raw, cmd.msg.identifier, cmd.msg.data_length_code,
               cmd.msg.data[0], cmd.msg.data[1], cmd.msg.data[2]);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    assert_frame_core(&cmd, 0x22, 0x06, true, 3U);
    TEST_ASSERT_TRUE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_IC_INDEX_USE_CLOSED_LOOP, cmd.index);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_IC_INDEX_USE_CLOSED_LOOP, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_UINT16(1U, motor_codec_unpack_u16_le(&cmd.msg.data[1]));
    TEST_ASSERT_EQUAL_STRING("IC_SET_U16", cmd.name);
}

TEST_CASE("motor cmd rejects invalid IC values", "[motor_protocol]")
{
    LOG_SECTION("motor cmd IC validation");
    motor_cmd_t cmd;
    esp_err_t err = motor_cmd_ic_set_u16(0x22, true, MOTOR_IC_INDEX_USE_CLOSED_LOOP, 2U, &cmd);

    LOG_INPUT("invalid IC boolean-like value=2");
    LOG_ERROR("err=%s", esp_err_to_name(err));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

TEST_CASE("motor cmd builds IE QE IL and MP indexed u16 frames", "[motor_protocol]")
{
    motor_cmd_t cmd;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_ie_get(0x05, true, MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, &cmd));
    assert_frame_core(&cmd, 0x05, 0x07, true, 1U);
    TEST_ASSERT_TRUE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, cmd.index);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_STRING("IE_GET", cmd.name);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_ie_set_u16(0x05, true, MOTOR_IE_INDEX_IN1_CHANGE_NOTIFICATION, 1U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x07, true, 3U);
    TEST_ASSERT_EQUAL_UINT16(1U, motor_codec_unpack_u16_le(&cmd.msg.data[1]));

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_qe_set_u16(0x05, true, MOTOR_QE_INDEX_BATTERY_VOLTAGE, 24000U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x3D, true, 3U);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_QE_INDEX_BATTERY_VOLTAGE, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_UINT16(24000U, motor_codec_unpack_u16_le(&cmd.msg.data[1]));

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_il_set_u16(0x05, true, MOTOR_IL_INDEX_STALL_BEHAVIOR, 2U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x34, true, 3U);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_IL_INDEX_STALL_BEHAVIOR, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_UINT16(2U, motor_codec_unpack_u16_le(&cmd.msg.data[1]));

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mp_set_u16(0x05, true, MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE, MOTOR_MP_MODE_LOOP, &cmd));
    assert_frame_core(&cmd, 0x05, 0x22, true, 3U);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_MP_MODE_LOOP, motor_codec_unpack_u16_le(&cmd.msg.data[1]));
}

TEST_CASE("motor cmd builds SD and BL scalar u32 frames and enforces documented validation", "[motor_protocol]")
{
    motor_cmd_t cmd;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_sd_get(0x05, true, &cmd));
    assert_frame_core(&cmd, 0x05, 0x1C, true, 0U);
    TEST_ASSERT_FALSE(cmd.has_index);
    TEST_ASSERT_EQUAL_STRING("SD_GET", cmd.name);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_sd_set_u32(0x05, true, 123456U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x1C, true, 4U);
    TEST_ASSERT_EQUAL_UINT32(123456U, motor_codec_unpack_u32_le(&cmd.msg.data[0]));
    TEST_ASSERT_EQUAL_STRING("SD_SET_U32", cmd.name);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bl_set_u32(0x05, true, 65535U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x2D, true, 4U);
    TEST_ASSERT_EQUAL_UINT32(65535U, motor_codec_unpack_u32_le(&cmd.msg.data[0]));
    TEST_ASSERT_EQUAL_STRING("BL_SET_U32", cmd.name);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bl_set_u32(0x05, true, 70000U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x2D, true, 4U);
    TEST_ASSERT_EQUAL_UINT32(70000U, motor_codec_unpack_u32_le(&cmd.msg.data[0]));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_cmd_ie_set_u16(0x05, true, MOTOR_IE_INDEX_IN1_CHANGE_NOTIFICATION, 2U, &cmd));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_cmd_mp_set_u16(0x05, true, MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE, 2U, &cmd));
}

TEST_CASE("motor cmd composes literal ACK and no-ACK CW values for new families", "[motor_protocol]")
{
    motor_cmd_t cmd;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_ie_get(0x05, false, MOTOR_IE_INDEX_IN1_CHANGE_NOTIFICATION, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0x07, cmd.cw_raw);
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_ie_get(0x05, true, MOTOR_IE_INDEX_IN1_CHANGE_NOTIFICATION, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0x87, cmd.cw_raw);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_qe_get(0x05, false, MOTOR_QE_INDEX_LINES_PER_REVOLUTION, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0x3D, cmd.cw_raw);
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_qe_get(0x05, true, MOTOR_QE_INDEX_LINES_PER_REVOLUTION, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0xBD, cmd.cw_raw);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_il_get(0x05, false, MOTOR_IL_INDEX_IN1_TRIGGER_ACTION, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0x34, cmd.cw_raw);
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_il_get(0x05, true, MOTOR_IL_INDEX_IN1_TRIGGER_ACTION, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0xB4, cmd.cw_raw);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mp_get(0x05, false, MOTOR_MP_INDEX_FIRST_VALID_ROW, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0x22, cmd.cw_raw);
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mp_get(0x05, true, MOTOR_MP_INDEX_FIRST_VALID_ROW, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0xA2, cmd.cw_raw);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_sd_get(0x05, false, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0x1C, cmd.cw_raw);
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_sd_get(0x05, true, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0x9C, cmd.cw_raw);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bl_get(0x05, false, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0x2D, cmd.cw_raw);
    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bl_get(0x05, true, &cmd));
    TEST_ASSERT_EQUAL_HEX8(0xAD, cmd.cw_raw);
}

TEST_CASE("motor cmd builds PV and PT frames exactly from the documented wire contracts", "[motor_protocol]")
{
    motor_cmd_t cmd;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pv_get(0x05, true, &cmd));
    assert_frame_core(&cmd, 0x05, 0x23, true, 0U);
    TEST_ASSERT_FALSE(cmd.has_index);
    TEST_ASSERT_EQUAL_STRING("PV_GET", cmd.name);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pv_set(0x05, true, 2U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x23, true, 2U);
    TEST_ASSERT_FALSE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(2U, motor_codec_unpack_u16_le(&cmd.msg.data[0]));
    TEST_ASSERT_EQUAL_STRING("PV_SET", cmd.name);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pt_get(0x05, true, 266U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x24, true, 2U);
    TEST_ASSERT_TRUE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(266U, cmd.index);
    TEST_ASSERT_EQUAL_UINT16(266U, motor_codec_unpack_u16_le(&cmd.msg.data[0]));
    TEST_ASSERT_EQUAL_STRING("PT_GET", cmd.name);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pt_set(0x05, true, 266U, 100000, &cmd));
    assert_frame_core(&cmd, 0x05, 0x24, true, 8U);
    TEST_ASSERT_TRUE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(266U, cmd.index);
    TEST_ASSERT_EQUAL_UINT16(266U, motor_codec_unpack_u16_le(&cmd.msg.data[0]));
    TEST_ASSERT_EQUAL_INT32(100000, motor_codec_unpack_i32_le(&cmd.msg.data[2]));
    TEST_ASSERT_EQUAL_HEX8(0x00, cmd.msg.data[6]);
    TEST_ASSERT_EQUAL_HEX8(0x00, cmd.msg.data[7]);
    TEST_ASSERT_EQUAL_STRING("PT_SET", cmd.name);
}

TEST_CASE("motor cmd builds manual IE QE IL MP SD and BL example frames exactly", "[motor_protocol]")
{
    motor_cmd_t cmd;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_ie_get(0x05, true, MOTOR_IE_INDEX_PTP_POSITIONING_FINISH_NOTIFICATION, &cmd));
    assert_frame_core(&cmd, 0x05, 0x07, true, 1U);
    TEST_ASSERT_EQUAL_HEX8(0x08, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_ie_set_u16(0x05, true, MOTOR_IE_INDEX_FIFO_LOW_WARNING_NOTIFICATION, 0U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x07, true, 3U);
    TEST_ASSERT_EQUAL_HEX8(0x0B, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_UINT16(0U, motor_codec_unpack_u16_le(&cmd.msg.data[1]));

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_qe_get(0x05, true, MOTOR_QE_INDEX_COUNTS_PER_REVOLUTION, &cmd));
    assert_frame_core(&cmd, 0x05, 0x3D, true, 1U);
    TEST_ASSERT_EQUAL_HEX8(0x04, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_qe_set_u16(0x05, true, MOTOR_QE_INDEX_LINES_PER_REVOLUTION, 1000U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x3D, true, 3U);
    TEST_ASSERT_EQUAL_HEX8(0x00, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_UINT16(1000U, motor_codec_unpack_u16_le(&cmd.msg.data[1]));

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_il_get(0x05, true, MOTOR_IL_INDEX_IN2_TRIGGER_ACTION, &cmd));
    assert_frame_core(&cmd, 0x05, 0x34, true, 1U);
    TEST_ASSERT_EQUAL_HEX8(0x01, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_il_set_u16(0x05, true, MOTOR_IL_INDEX_IN3_TRIGGER_ACTION, 0x1234U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x34, true, 3U);
    TEST_ASSERT_EQUAL_HEX8(0x02, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_UINT16(0x1234U, motor_codec_unpack_u16_le(&cmd.msg.data[1]));

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mp_get(0x05, true, MOTOR_MP_INDEX_PT_MOTION_TIME, &cmd));
    assert_frame_core(&cmd, 0x05, 0x22, true, 1U);
    TEST_ASSERT_EQUAL_HEX8(0x04, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mp_set_u16(0x05, true, MOTOR_MP_INDEX_QUEUE_LOW_THRESHOLD, 9U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x22, true, 3U);
    TEST_ASSERT_EQUAL_HEX8(0x05, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_UINT16(9U, motor_codec_unpack_u16_le(&cmd.msg.data[1]));

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_sd_get(0x05, true, &cmd));
    assert_frame_core(&cmd, 0x05, 0x1C, true, 0U);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_sd_set_u32(0x05, true, 321000U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x1C, true, 4U);
    TEST_ASSERT_EQUAL_UINT32(321000U, motor_codec_unpack_u32_le(&cmd.msg.data[0]));

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bl_get(0x05, true, &cmd));
    assert_frame_core(&cmd, 0x05, 0x2D, true, 0U);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bl_set_u32(0x05, true, 123U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x2D, true, 4U);
    TEST_ASSERT_EQUAL_UINT32(123U, motor_codec_unpack_u32_le(&cmd.msg.data[0]));
}

TEST_CASE("motor cmd rejects invalid indexes and builder arguments across supported families", "[motor_protocol]")
{
    motor_cmd_t cmd;

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_cmd_er_get(0x05, true, (motor_er_index_t)0x00, &cmd));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_cmd_er_get(0x05, true, (motor_er_index_t)0x09, &cmd));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_cmd_ie_get(0x05, true, (motor_ie_index_t)0x09, &cmd));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_cmd_qe_get(0x05, true, (motor_qe_index_t)0x02, &cmd));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_cmd_il_get(0x05, true, (motor_il_index_t)0x04, &cmd));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_cmd_mp_get(0x05, true, (motor_mp_index_t)0x07, &cmd));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_cmd_sd_get(0x04, true, &cmd));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_cmd_bl_get(0x05, true, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, motor_cmd_sy_reboot(0x04, &cmd));
}

TEST_CASE("motor cmd builds ER get frame", "[motor_protocol]")
{
    LOG_SECTION("motor cmd ER get");
    motor_cmd_t cmd;
    esp_err_t err = motor_cmd_er_get(0x7E, false, MOTOR_ER_INDEX_HISTORY_4, &cmd);

    LOG_INPUT("target_id=0x%02X ack_requested=%d index=%u", 0x7E, false, MOTOR_ER_INDEX_HISTORY_4);
    LOG_OUTPUT("err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u data=[%02X]",
               esp_err_to_name(err), cmd.cw_raw, cmd.msg.identifier, cmd.msg.data_length_code, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    assert_frame_core(&cmd, 0x7E, 0x0F, false, 1U);
    TEST_ASSERT_TRUE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_ER_INDEX_HISTORY_4, cmd.index);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_ER_INDEX_HISTORY_4, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_STRING("ER_GET", cmd.name);
}

TEST_CASE("motor cmd builds ER clear all frame", "[motor_protocol]")
{
    LOG_SECTION("motor cmd ER clear all");
    motor_cmd_t cmd;
    esp_err_t err = motor_cmd_er_clear_all(0x7E, true, &cmd);

    LOG_INPUT("target_id=0x%02X ack_requested=%d", 0x7E, true);
    LOG_OUTPUT("err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u data=[%02X %02X]",
               esp_err_to_name(err), cmd.cw_raw, cmd.msg.identifier, cmd.msg.data_length_code,
               cmd.msg.data[0], cmd.msg.data[1]);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    assert_frame_core(&cmd, 0x7E, 0x0F, true, 2U);
    TEST_ASSERT_TRUE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_ER_INDEX_CLEAR_ALL, cmd.index);
    TEST_ASSERT_EQUAL_HEX8(0x00, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, cmd.msg.data[1]);
    TEST_ASSERT_EQUAL_STRING("ER_CLEAR_ALL", cmd.name);
}

TEST_CASE("motor cmd builds SY reboot frame", "[motor_protocol]")
{
    LOG_SECTION("motor cmd SY reboot");
    motor_cmd_t cmd;
    esp_err_t err = motor_cmd_sy_reboot(0x00, &cmd);

    LOG_INPUT("target_id=0x%02X", 0x00);
    LOG_OUTPUT("err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u data=[%02X]",
               esp_err_to_name(err), cmd.cw_raw, cmd.msg.identifier, cmd.msg.data_length_code, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    assert_frame_core(&cmd, 0x00, 0x7E, false, 1U);
    TEST_ASSERT_TRUE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_SY_OP_REBOOT, cmd.index);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_SY_OP_REBOOT, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_STRING("SY_REBOOT", cmd.name);
}

TEST_CASE("motor cmd builds SY restore defaults frame", "[motor_protocol]")
{
    LOG_SECTION("motor cmd SY restore defaults");
    motor_cmd_t cmd;
    esp_err_t err = motor_cmd_sy_restore_factory_defaults(0x05, &cmd);

    LOG_INPUT("target_id=0x%02X", 0x05);
    LOG_OUTPUT("err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u data=[%02X]",
               esp_err_to_name(err), cmd.cw_raw, cmd.msg.identifier, cmd.msg.data_length_code, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    assert_frame_core(&cmd, 0x05, 0x7E, false, 1U);
    TEST_ASSERT_TRUE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_SY_OP_RESTORE_FACTORY_DEFAULTS, cmd.index);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_SY_OP_RESTORE_FACTORY_DEFAULTS, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_STRING("SY_RESTORE_FACTORY_DEFAULTS", cmd.name);
}

TEST_CASE("motor cmd builds MT set u16 frame", "[motor_protocol]")
{
    LOG_SECTION("motor cmd MT set u16");
    motor_cmd_t cmd;
    esp_err_t err = motor_cmd_mt_set_u16(0x35, false, MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS, 60000U, &cmd);

    LOG_INPUT("target_id=0x%02X ack_requested=%d index=%u value=%u",
              0x35, false, MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS, 60000U);
    LOG_OUTPUT("err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u data=[%02X %02X %02X]",
               esp_err_to_name(err), cmd.cw_raw, cmd.msg.identifier, cmd.msg.data_length_code,
               cmd.msg.data[0], cmd.msg.data[1], cmd.msg.data[2]);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    assert_frame_core(&cmd, 0x35, 0x10, false, 3U);
    TEST_ASSERT_TRUE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS, cmd.index);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_UINT16(60000U, motor_codec_unpack_u16_le(&cmd.msg.data[1]));
    TEST_ASSERT_EQUAL_STRING("MT_SET_U16", cmd.name);
}

TEST_CASE("motor cmd rejects invalid MT values", "[motor_protocol]")
{
    LOG_SECTION("motor cmd MT validation");
    motor_cmd_t cmd;
    esp_err_t err = motor_cmd_mt_set_u16(0x35, false, MOTOR_MT_INDEX_MICROSTEP, 3U, &cmd);

    LOG_INPUT("invalid MT microstep value=3");
    LOG_ERROR("err=%s", esp_err_to_name(err));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

TEST_CASE("motor cmd builds MO set frame", "[motor_protocol]")
{
    LOG_SECTION("motor cmd MO set");
    motor_cmd_t cmd;
    esp_err_t err = motor_cmd_mo_set(0x2A, true, MOTOR_MO_STATE_ENABLE, &cmd);

    LOG_INPUT("target_id=0x%02X ack_requested=%d state=%u", 0x2A, true, MOTOR_MO_STATE_ENABLE);
    LOG_OUTPUT("err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u data=[%02X]",
               esp_err_to_name(err), cmd.cw_raw, cmd.msg.identifier, cmd.msg.data_length_code, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    assert_frame_core(&cmd, 0x2A, 0x15, true, 1U);
    TEST_ASSERT_FALSE(cmd.has_index);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_MO_STATE_ENABLE, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_STRING("MO_SET", cmd.name);
}

TEST_CASE("motor cmd builds BG and ST no payload frames", "[motor_protocol]")
{
    LOG_SECTION("motor cmd BG and ST");
    motor_cmd_t bg_cmd;
    motor_cmd_t st_cmd;
    esp_err_t err_bg = motor_cmd_bg_begin(0x09, true, &bg_cmd);
    esp_err_t err_st = motor_cmd_st_stop(0x09, false, &st_cmd);

    LOG_INPUT("BG target_id=0x%02X ack_requested=%d", 0x09, true);
    LOG_OUTPUT("BG err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u",
               esp_err_to_name(err_bg), bg_cmd.cw_raw, bg_cmd.msg.identifier, bg_cmd.msg.data_length_code);
    LOG_INPUT("ST target_id=0x%02X ack_requested=%d", 0x09, false);
    LOG_OUTPUT("ST err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u",
               esp_err_to_name(err_st), st_cmd.cw_raw, st_cmd.msg.identifier, st_cmd.msg.data_length_code);

    TEST_ASSERT_EQUAL(ESP_OK, err_bg);
    TEST_ASSERT_EQUAL(ESP_OK, err_st);

    assert_frame_core(&bg_cmd, 0x09, 0x16, true, 0U);
    TEST_ASSERT_FALSE(bg_cmd.has_index);
    TEST_ASSERT_EQUAL_STRING("BG_BEGIN", bg_cmd.name);
    TEST_ASSERT_EACH_EQUAL_HEX8(0x00, bg_cmd.msg.data, 8);

    assert_frame_core(&st_cmd, 0x09, 0x17, false, 0U);
    TEST_ASSERT_FALSE(st_cmd.has_index);
    TEST_ASSERT_EQUAL_STRING("ST_STOP", st_cmd.name);
    TEST_ASSERT_EACH_EQUAL_HEX8(0x00, st_cmd.msg.data, 8);
}

TEST_CASE("motor cmd builds PA set i32 frame", "[motor_protocol]")
{
    LOG_SECTION("motor cmd PA set i32");
    motor_cmd_t cmd;
    int32_t position = -123456;
    esp_err_t err = motor_cmd_pa_set(0x0A, true, position, &cmd);

    LOG_INPUT("target_id=0x%02X ack_requested=%d position=%ld", 0x0A, true, (long)position);
    LOG_OUTPUT("err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u data=[%02X %02X %02X %02X]",
               esp_err_to_name(err), cmd.cw_raw, cmd.msg.identifier, cmd.msg.data_length_code,
               cmd.msg.data[0], cmd.msg.data[1], cmd.msg.data[2], cmd.msg.data[3]);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    assert_frame_core(&cmd, 0x0A, 0x20, true, 4U);
    TEST_ASSERT_FALSE(cmd.has_index);
    TEST_ASSERT_EQUAL_INT32(position, motor_codec_unpack_i32_le(&cmd.msg.data[0]));
    TEST_ASSERT_EQUAL_STRING("PA_SET", cmd.name);
}

TEST_CASE("motor cmd builds LM set i32 frame", "[motor_protocol]")
{
    LOG_SECTION("motor cmd LM set i32");
    motor_cmd_t cmd;
    int32_t value = 1000000;
    esp_err_t err = motor_cmd_lm_set_i32(0x2F, false, MOTOR_LM_INDEX_MAX_ACCEL_DECEL, value, &cmd);

    LOG_INPUT("target_id=0x%02X ack_requested=%d index=%u value=%ld",
              0x2F, false, MOTOR_LM_INDEX_MAX_ACCEL_DECEL, (long)value);
    LOG_OUTPUT("err=%s cw=0x%02X ext_id=0x%08" PRIX32 " dlc=%u data=[%02X %02X %02X %02X %02X]",
               esp_err_to_name(err), cmd.cw_raw, cmd.msg.identifier, cmd.msg.data_length_code,
               cmd.msg.data[0], cmd.msg.data[1], cmd.msg.data[2], cmd.msg.data[3], cmd.msg.data[4]);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    assert_frame_core(&cmd, 0x2F, 0x2C, false, 5U);
    TEST_ASSERT_TRUE(cmd.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_LM_INDEX_MAX_ACCEL_DECEL, cmd.index);
    TEST_ASSERT_EQUAL_HEX8(MOTOR_LM_INDEX_MAX_ACCEL_DECEL, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_INT32(value, motor_codec_unpack_i32_le(&cmd.msg.data[1]));
    TEST_ASSERT_EQUAL_STRING("LM_SET_I32", cmd.name);
}

TEST_CASE("motor cmd rejects invalid LM values", "[motor_protocol]")
{
    LOG_SECTION("motor cmd LM validation");
    motor_cmd_t cmd;
    esp_err_t err = motor_cmd_lm_set_i32(0x2F, false, MOTOR_LM_INDEX_RESET_LIMITS, 0, &cmd);

    LOG_INPUT("invalid LM reset-limits value=0");
    LOG_ERROR("err=%s", esp_err_to_name(err));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

TEST_CASE("motor cmd builds manual PP example frames exactly", "[motor_protocol]")
{
    typedef struct {
        esp_err_t (*build)(uint8_t, bool, motor_pp_index_t, motor_cmd_t *);
        esp_err_t (*build_set)(uint8_t, bool, motor_pp_index_t, uint8_t, motor_cmd_t *);
        motor_pp_index_t index;
        bool is_set;
        uint8_t value;
        uint8_t expected_dlc;
        uint8_t expected_data[2];
    } pp_case_t;
    static const pp_case_t cases[] = {
        {motor_cmd_pp_get, NULL, MOTOR_PP_INDEX_BITRATE, false, 0x00, 1U, {0x05, 0x00}},
        {motor_cmd_pp_get, NULL, MOTOR_PP_INDEX_NODE_ID, false, 0x00, 1U, {0x07, 0x00}},
        {motor_cmd_pp_get, NULL, MOTOR_PP_INDEX_GROUP_ID, false, 0x00, 1U, {0x08, 0x00}},
        {NULL, motor_cmd_pp_set_u8, MOTOR_PP_INDEX_BITRATE, true, 0x02, 2U, {0x05, 0x02}},
        {NULL, motor_cmd_pp_set_u8, MOTOR_PP_INDEX_NODE_ID, true, 0x0B, 2U, {0x07, 0x0B}},
        {NULL, motor_cmd_pp_set_u8, MOTOR_PP_INDEX_GROUP_ID, true, 0x14, 2U, {0x08, 0x14}},
    };
    size_t i;

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        motor_cmd_t cmd;
        esp_err_t err = cases[i].is_set
                            ? cases[i].build_set(0x05, true, cases[i].index, cases[i].value, &cmd)
                            : cases[i].build(0x05, true, cases[i].index, &cmd);

        TEST_ASSERT_EQUAL(ESP_OK, err);
        assert_frame_core(&cmd, 0x05, 0x01, true, cases[i].expected_dlc);
        TEST_ASSERT_TRUE(cmd.has_index);
        TEST_ASSERT_EQUAL_UINT16(cases[i].index, cmd.index);
        TEST_ASSERT_EQUAL_HEX8_ARRAY(cases[i].expected_data, cmd.msg.data, cases[i].expected_dlc);
    }
}

TEST_CASE("motor cmd builds manual IC example frames exactly", "[motor_protocol]")
{
    typedef struct {
        bool is_set;
        motor_ic_index_t index;
        uint16_t value;
    } ic_case_t;
    static const ic_case_t cases[] = {
        {false, MOTOR_IC_INDEX_AUTO_ENABLE_AFTER_POWERUP, 0x0000},
        {false, MOTOR_IC_INDEX_POSITIVE_MOTOR_DIRECTION, 0x0000},
        {false, MOTOR_IC_INDEX_ENABLE_LOCKDOWN_SYSTEM, 0x0000},
        {false, MOTOR_IC_INDEX_AC_DC_UNITS, 0x0000},
        {false, MOTOR_IC_INDEX_USE_CLOSED_LOOP, 0x0000},
        {false, MOTOR_IC_INDEX_SOFTWARE_LIMIT_ENABLE, 0x0000},
        {false, MOTOR_IC_INDEX_INTERNAL_BRAKE_LOGIC_ENABLE, 0x0000},
        {false, MOTOR_IC_INDEX_USE_INTERNAL_BRAKE, 0x0000},
        {true, MOTOR_IC_INDEX_AUTO_ENABLE_AFTER_POWERUP, 0x0000},
        {true, MOTOR_IC_INDEX_POSITIVE_MOTOR_DIRECTION, 0x0001},
        {true, MOTOR_IC_INDEX_ENABLE_LOCKDOWN_SYSTEM, 0x0000},
        {true, MOTOR_IC_INDEX_AC_DC_UNITS, 0x0001},
        {true, MOTOR_IC_INDEX_USE_CLOSED_LOOP, 0x0001},
        {true, MOTOR_IC_INDEX_SOFTWARE_LIMIT_ENABLE, 0x0001},
        {true, MOTOR_IC_INDEX_INTERNAL_BRAKE_LOGIC_ENABLE, 0x0001},
        {true, MOTOR_IC_INDEX_USE_INTERNAL_BRAKE, 0x0001},
    };
    size_t i;

    for (i = 0; i < sizeof(cases) / sizeof(cases[0]); ++i) {
        motor_cmd_t cmd;
        esp_err_t err = cases[i].is_set
                            ? motor_cmd_ic_set_u16(0x05, true, cases[i].index, cases[i].value, &cmd)
                            : motor_cmd_ic_get(0x05, true, cases[i].index, &cmd);

        TEST_ASSERT_EQUAL(ESP_OK, err);
        assert_frame_core(&cmd, 0x05, 0x06, true, cases[i].is_set ? 3U : 1U);
        TEST_ASSERT_TRUE(cmd.has_index);
        TEST_ASSERT_EQUAL_UINT16(cases[i].index, cmd.index);
        TEST_ASSERT_EQUAL_HEX8((uint8_t)cases[i].index, cmd.msg.data[0]);
        if (cases[i].is_set) {
            TEST_ASSERT_EQUAL_UINT16(cases[i].value, motor_codec_unpack_u16_le(&cmd.msg.data[1]));
        }
    }
}

TEST_CASE("motor cmd builds manual ER and SY example frames exactly", "[motor_protocol]")
{
    motor_cmd_t er_get;
    motor_cmd_t er_clear;
    motor_cmd_t sy_reboot;
    motor_cmd_t sy_restore;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_er_get(0x05, true, MOTOR_ER_INDEX_HISTORY_1, &er_get));
    assert_frame_core(&er_get, 0x05, 0x0F, true, 1U);
    TEST_ASSERT_EQUAL_HEX8(0x0A, er_get.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_er_clear_all(0x05, true, &er_clear));
    assert_frame_core(&er_clear, 0x05, 0x0F, true, 2U);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(((const uint8_t[]){0x00, 0x00}), er_clear.msg.data, 2U);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_sy_reboot(0x05, &sy_reboot));
    assert_frame_core(&sy_reboot, 0x05, 0x7E, false, 1U);
    TEST_ASSERT_EQUAL_HEX8(0x01, sy_reboot.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_sy_restore_factory_defaults(0x05, &sy_restore));
    assert_frame_core(&sy_restore, 0x05, 0x7E, false, 1U);
    TEST_ASSERT_EQUAL_HEX8(0x02, sy_restore.msg.data[0]);
}

TEST_CASE("motor cmd builds manual MT MO BG ST PA OG and LM example frames exactly", "[motor_protocol]")
{
    motor_cmd_t cmd;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mt_get(0x05, true, MOTOR_MT_INDEX_MICROSTEP, &cmd));
    assert_frame_core(&cmd, 0x05, 0x10, true, 1U);
    TEST_ASSERT_EQUAL_HEX8(0x00, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mt_set_u16(0x05, true, MOTOR_MT_INDEX_MICROSTEP, 16U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x10, true, 3U);
    TEST_ASSERT_EQUAL_HEX8(0x00, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_UINT16(16U, motor_codec_unpack_u16_le(&cmd.msg.data[1]));

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mt_set_u16(0x05, true, MOTOR_MT_INDEX_BRAKE_STATE, 0U, &cmd));
    assert_frame_core(&cmd, 0x05, 0x10, true, 3U);
    TEST_ASSERT_EQUAL_HEX8(0x05, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_UINT16(0U, motor_codec_unpack_u16_le(&cmd.msg.data[1]));

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mo_get(0x05, true, &cmd));
    assert_frame_core(&cmd, 0x05, 0x15, true, 0U);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mo_set(0x05, true, MOTOR_MO_STATE_ENABLE, &cmd));
    assert_frame_core(&cmd, 0x05, 0x15, true, 1U);
    TEST_ASSERT_EQUAL_HEX8(0x01, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mo_set(0x05, true, MOTOR_MO_STATE_DISABLE, &cmd));
    assert_frame_core(&cmd, 0x05, 0x15, true, 1U);
    TEST_ASSERT_EQUAL_HEX8(0x00, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bg_begin(0x05, true, &cmd));
    assert_frame_core(&cmd, 0x05, 0x16, true, 0U);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_st_stop(0x05, true, &cmd));
    assert_frame_core(&cmd, 0x05, 0x17, true, 0U);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pa_get(0x05, true, &cmd));
    assert_frame_core(&cmd, 0x05, 0x20, true, 0U);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pa_set(0x05, true, 1000, &cmd));
    assert_frame_core(&cmd, 0x05, 0x20, true, 4U);
    TEST_ASSERT_EQUAL_INT32(1000, motor_codec_unpack_i32_le(&cmd.msg.data[0]));

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_og_set_origin(0x05, true, &cmd));
    assert_frame_core(&cmd, 0x05, 0x21, true, 0U);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_lm_get(0x05, true, MOTOR_LM_INDEX_LOWER_WORKING_LIMIT, &cmd));
    assert_frame_core(&cmd, 0x05, 0x2C, true, 1U);
    TEST_ASSERT_EQUAL_HEX8(0x01, cmd.msg.data[0]);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_lm_set_i32(0x05, true, MOTOR_LM_INDEX_UPPER_WORKING_LIMIT, 10000, &cmd));
    assert_frame_core(&cmd, 0x05, 0x2C, true, 5U);
    TEST_ASSERT_EQUAL_HEX8(0x02, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_INT32(10000, motor_codec_unpack_i32_le(&cmd.msg.data[1]));

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_lm_set_i32(0x05, true, MOTOR_LM_INDEX_ENABLE_DISABLE, 0, &cmd));
    assert_frame_core(&cmd, 0x05, 0x2C, true, 5U);
    TEST_ASSERT_EQUAL_HEX8(0xFF, cmd.msg.data[0]);
    TEST_ASSERT_EQUAL_INT32(0, motor_codec_unpack_i32_le(&cmd.msg.data[1]));
}
