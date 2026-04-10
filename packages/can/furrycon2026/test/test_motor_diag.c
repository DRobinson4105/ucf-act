/*
 * Responsibility:
 * Focused Unity coverage for the motor_diag public type and formatting surface.
 */

#include <string.h>

#include "unity.h"

#include "motor_codec.h"
#include "motor_diag.h"
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

TEST_CASE("motor diag describes and formats PP GET command semantically", "[motor_diag]")
{
    motor_cmd_t cmd;
    motor_diag_cmd_desc_t desc;
    char line[128];

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_get(0x06, true, MOTOR_PP_INDEX_BITRATE, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_GET, desc.kind);
    TEST_ASSERT_EQUAL_STRING("PP", desc.family_symbol);
    TEST_ASSERT_TRUE(desc.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_PP_INDEX_BITRATE, desc.index);
    TEST_ASSERT_EQUAL_STRING("CAN bitrate", desc.semantic_name);

    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=6 PP[5] GET (CAN bitrate)", line);
}

TEST_CASE("motor diag describes and formats MO ACK semantically", "[motor_diag]")
{
    static const uint8_t data[] = {MOTOR_MO_STATE_ENABLE};
    twai_message_t msg = make_ext_frame(0x06, 0x15, 1U, data);
    motor_rx_t rx;
    motor_diag_rx_desc_t desc;
    char line[128];

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_ACK, desc.kind);
    TEST_ASSERT_EQUAL_STRING("Motor driver", desc.semantic_name);
    TEST_ASSERT_TRUE(desc.value.has_value);
    TEST_ASSERT_EQUAL_STRING("ON", desc.value.text);

    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=6 ACK MO Motor driver = ON", line);
}

TEST_CASE("motor diag keeps ordinary MO ACK free of semantic lineage", "[motor_diag]")
{
    static const uint8_t data[] = {MOTOR_MO_STATE_ENABLE};
    twai_message_t msg = make_ext_frame(0x06, 0x15, 1U, data);
    motor_rx_t rx;
    motor_diag_rx_desc_t desc;
    char line[128];

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_ACK, desc.kind);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_MO, desc.object);
    TEST_ASSERT_EQUAL_HEX8(0x15, desc.base_code);
    TEST_ASSERT_EQUAL_STRING("MO", desc.family_symbol);
    TEST_ASSERT_FALSE(desc.has_index);
    TEST_ASSERT_FALSE(desc.has_semantic_lineage);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_NONE, desc.semantic_object);
    TEST_ASSERT_NULL(desc.semantic_family_symbol);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_UNKNOWN, desc.semantic_cmd_kind);
    TEST_ASSERT_FALSE(desc.has_semantic_index);
    TEST_ASSERT_EQUAL_UINT16(0U, desc.semantic_index);
    TEST_ASSERT_EQUAL_STRING("Motor driver", desc.semantic_name);
    TEST_ASSERT_TRUE(desc.value.has_value);
    TEST_ASSERT_EQUAL_STRING("ON", desc.value.text);

    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=6 ACK MO Motor driver = ON", line);
}

TEST_CASE("motor diag preserves truthful fallback for unknown RT status mapping", "[motor_diag]")
{
    static const uint8_t data[] = {0x7BU, 0x00U};
    twai_message_t msg = make_ext_frame(0x06, 0x5A, 2U, data);
    motor_rx_t rx;
    motor_diag_rx_desc_t desc;
    char line[128];

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_RT_UNKNOWN, desc.kind);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_COMPLETENESS_PARTIAL, desc.completeness);
    TEST_ASSERT_EQUAL_STRING("unknown notification type", desc.note);

    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=6 RT raw d0=0x7B d1=0x00 (unknown notification type)", line);
}

TEST_CASE("motor diag preserves structural DV ACK and adds PA set lineage", "[motor_diag]")
{
    uint8_t data[5] = {4U};
    twai_message_t msg;
    motor_rx_t rx;
    motor_diag_rx_desc_t desc;
    char line[160];

    motor_codec_pack_i32_le(&data[1], 1000);
    msg = make_ext_frame(0x06, 0x2E, 5U, data);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_ACK, desc.kind);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_DV, desc.object);
    TEST_ASSERT_EQUAL_HEX8(0x2E, desc.base_code);
    TEST_ASSERT_EQUAL_STRING("DV", desc.family_symbol);
    TEST_ASSERT_TRUE(desc.has_index);
    TEST_ASSERT_EQUAL_UINT16(4U, desc.index);
    TEST_ASSERT_TRUE(desc.has_semantic_lineage);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_PA, desc.semantic_object);
    TEST_ASSERT_EQUAL_STRING("PA", desc.semantic_family_symbol);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_SET, desc.semantic_cmd_kind);
    TEST_ASSERT_FALSE(desc.has_semantic_index);
    TEST_ASSERT_EQUAL_STRING("Absolute position", desc.semantic_name);
    TEST_ASSERT_TRUE(desc.value.has_value);
    TEST_ASSERT_TRUE(desc.value.has_raw_number);
    TEST_ASSERT_EQUAL_INT32(1000, desc.value.raw_number);

    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=6 ACK DV[4] (for PA set) Absolute position = 1000 pulse", line);
}

TEST_CASE("motor diag describes ER history query responses structurally", "[motor_diag]")
{
    uint8_t data[] = {
        MOTOR_ER_INDEX_HISTORY_1,
        0x33,
        motor_codec_compose_cw(0x01U, true),
        MOTOR_PP_INDEX_BITRATE,
        0x00,
        0x00
    };
    twai_message_t msg = make_ext_frame(0x06, 0x0F, 6U, data);
    motor_rx_t rx;
    motor_diag_rx_desc_t desc;
    char line[160];

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_ER_RESPONSE, desc.kind);
    TEST_ASSERT_TRUE(desc.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_ER_INDEX_HISTORY_1, desc.index);
    TEST_ASSERT_EQUAL_STRING("First error slot", desc.semantic_name);
    TEST_ASSERT_TRUE(desc.has_error_code);
    TEST_ASSERT_EQUAL_HEX8(0x33, desc.error_code);
    TEST_ASSERT_EQUAL_STRING("Instruction data error", desc.error_name);
    TEST_ASSERT_TRUE(desc.has_related_family);
    TEST_ASSERT_EQUAL_STRING("PP", desc.related_family_symbol);
    TEST_ASSERT_TRUE(desc.has_related_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_PP_INDEX_BITRATE, desc.related_index);

    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=6 ER[10] RESPONSE (First error slot) error=0x33 (Instruction data error) related=PP[5]", line);
}

TEST_CASE("motor diag describes ER GET history commands without unknown mapping", "[motor_diag]")
{
    motor_cmd_t cmd;
    motor_diag_cmd_desc_t desc;
    char line[128];

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_er_get(0x06, true, MOTOR_ER_INDEX_HISTORY_1, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_GET, desc.kind);
    TEST_ASSERT_EQUAL_STRING("ER", desc.family_symbol);
    TEST_ASSERT_TRUE(desc.has_index);
    TEST_ASSERT_EQUAL_UINT16(MOTOR_ER_INDEX_HISTORY_1, desc.index);
    TEST_ASSERT_EQUAL_STRING("First error slot", desc.semantic_name);
    TEST_ASSERT_NULL(desc.note);

    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=6 ER[10] GET (First error slot)", line);
}

TEST_CASE("motor diag keeps thrown ER d0 zero rendered as thrown error", "[motor_diag]")
{
    twai_message_t msg = make_ext_frame(0x06, 0x0F, 6U, (const uint8_t[]){0x00, 0x33, 0x81, 0x05, 0x00, 0x00});
    motor_rx_t rx;
    motor_diag_rx_desc_t desc;
    char line[160];

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_ERROR, desc.kind);
    TEST_ASSERT_EQUAL_HEX8(0x33, desc.error_code);

    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=6 ERROR code=0x33 (Instruction data error) related=PP[5]", line);
}

TEST_CASE("motor diag keeps shallow MT mapping numeric instead of inventing richer semantics", "[motor_diag]")
{
    static const uint8_t data[] = {MOTOR_MT_INDEX_MICROSTEP, 0x10, 0x00};
    twai_message_t msg = make_ext_frame(0x06, 0x10, 3U, data);
    motor_rx_t rx;
    motor_diag_rx_desc_t desc;

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_ACK, desc.kind);
    TEST_ASSERT_EQUAL_STRING("Micro-Stepping Resolution", desc.semantic_name);
    TEST_ASSERT_TRUE(desc.value.has_value);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_U16, desc.value.kind);
    TEST_ASSERT_TRUE(desc.value.has_raw_number);
    TEST_ASSERT_EQUAL_INT32(16, desc.value.raw_number);
    TEST_ASSERT_NULL(desc.value.text);
}

TEST_CASE("motor diag describes new indexed families and enum semantics truthfully", "[motor_diag]")
{
    motor_cmd_t cmd;
    motor_diag_cmd_desc_t cmd_desc;
    motor_rx_t rx;
    motor_diag_rx_desc_t rx_desc;
    char line[200];
    twai_message_t mp_mode_msg = make_ext_frame(0x05, 0x22, 3U, (const uint8_t[]){0x03, 0x01, 0x00});
    twai_message_t mp_time_msg = make_ext_frame(0x05, 0x22, 3U, (const uint8_t[]){0x04, 0x00, 0x00});
    twai_message_t qe_msg = make_ext_frame(0x05, 0x3D, 3U, (const uint8_t[]){0x03, 0xC0, 0x5D});

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_ie_set_u16(0x05, true, MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, 1U, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &cmd_desc));
    TEST_ASSERT_EQUAL_STRING("FIFO empty notification", cmd_desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_ENUM, cmd_desc.value.kind);
    TEST_ASSERT_EQUAL_STRING("enabled", cmd_desc.value.text);
    motor_diag_format_cmd(line, sizeof(line), &cmd_desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 IE[10] SET (FIFO empty notification) = enabled", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&mp_mode_msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &rx_desc));
    TEST_ASSERT_EQUAL_STRING("PVT data management mode", rx_desc.semantic_name);
    TEST_ASSERT_EQUAL_STRING("Single mode", rx_desc.value.text);
    motor_diag_format_rx(line, sizeof(line), &rx_desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK MP[3] PVT data management mode = Single mode", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&mp_time_msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &rx_desc));
    TEST_ASSERT_EQUAL_STRING("PT motion time", rx_desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_TEXT, rx_desc.value.kind);
    TEST_ASSERT_EQUAL_STRING("0", rx_desc.value.text);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_COMPLETENESS_PARTIAL, rx_desc.completeness);
    TEST_ASSERT_EQUAL_STRING("zero may have a special meaning", rx_desc.note);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&qe_msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &rx_desc));
    TEST_ASSERT_EQUAL_STRING("Battery voltage", rx_desc.semantic_name);
    TEST_ASSERT_EQUAL(24000, rx_desc.value.raw_number);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_COMPLETENESS_PARTIAL, rx_desc.completeness);
    TEST_ASSERT_EQUAL_STRING("units are not yet verified", rx_desc.note);
    motor_diag_format_rx(line, sizeof(line), &rx_desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK QE[3] Battery voltage = 24000 (units are not yet verified)", line);
}

TEST_CASE("motor diag describes SD and BL scalar u32 values", "[motor_diag]")
{
    motor_cmd_t cmd;
    motor_diag_cmd_desc_t cmd_desc;
    motor_rx_t rx;
    motor_diag_rx_desc_t rx_desc;
    char line[200];
    uint8_t data[4];
    twai_message_t bl_msg;

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_sd_set_u32(0x05, true, 321000U, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &cmd_desc));
    TEST_ASSERT_EQUAL_STRING("Stop deceleration", cmd_desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_U32, cmd_desc.value.kind);
    TEST_ASSERT_EQUAL(321000, cmd_desc.value.raw_number);
    motor_diag_format_cmd(line, sizeof(line), &cmd_desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 SD SET (Stop deceleration) = 321000 pulse/sec^2", line);

    motor_codec_pack_u32_le(data, 70000U);
    bl_msg = make_ext_frame(0x05, 0x2D, 4U, data);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&bl_msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &rx_desc));
    TEST_ASSERT_EQUAL_STRING("Backlash compensation", rx_desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_U32, rx_desc.value.kind);
    TEST_ASSERT_EQUAL(70000, rx_desc.value.raw_number);
    motor_diag_format_rx(line, sizeof(line), &rx_desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK BL Backlash compensation = 70000 pulse", line);
}

TEST_CASE("motor diag keeps newer shallow mappings numeric and free of unintended lineage", "[motor_diag]")
{
    motor_cmd_t il_cmd;
    motor_diag_cmd_desc_t cmd_desc;
    motor_rx_t rx;
    motor_diag_rx_desc_t rx_desc;
    char line[200];
    twai_message_t il_msg = make_ext_frame(0x05, 0x34, 3U, (const uint8_t[]){0x03, 0x34, 0x12});
    twai_message_t sd_msg;
    uint8_t sd_data[4];

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_il_set_u16(0x05, true, MOTOR_IL_INDEX_STALL_BEHAVIOR, 0x1234U, &il_cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&il_cmd, &cmd_desc));
    TEST_ASSERT_EQUAL_STRING("Stall behavior", cmd_desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_U16, cmd_desc.value.kind);
    TEST_ASSERT_EQUAL(0x1234, cmd_desc.value.raw_number);
    TEST_ASSERT_NULL(cmd_desc.value.text);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_COMPLETENESS_PARTIAL, cmd_desc.completeness);
    TEST_ASSERT_EQUAL_STRING("value semantics are only numerically mapped", cmd_desc.note);
    motor_diag_format_cmd(line, sizeof(line), &cmd_desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 IL[3] SET (Stall behavior) = 4660 (value semantics are only numerically mapped)", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&il_msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &rx_desc));
    TEST_ASSERT_FALSE(rx_desc.has_semantic_lineage);
    TEST_ASSERT_EQUAL_STRING("Stall behavior", rx_desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_U16, rx_desc.value.kind);
    TEST_ASSERT_EQUAL(0x1234, rx_desc.value.raw_number);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_COMPLETENESS_PARTIAL, rx_desc.completeness);
    TEST_ASSERT_EQUAL_STRING("value semantics are only numerically mapped", rx_desc.note);
    motor_diag_format_rx(line, sizeof(line), &rx_desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK IL[3] Stall behavior = 4660 (value semantics are only numerically mapped)", line);

    motor_codec_pack_u32_le(sd_data, 321000U);
    sd_msg = make_ext_frame(0x05, 0x1C, 4U, sd_data);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&sd_msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &rx_desc));
    TEST_ASSERT_FALSE(rx_desc.has_semantic_lineage);
    TEST_ASSERT_EQUAL_STRING("SD", rx_desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Stop deceleration", rx_desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_U32, rx_desc.value.kind);
    TEST_ASSERT_EQUAL(321000, rx_desc.value.raw_number);
}

TEST_CASE("motor diag describes newer manual-style command examples exactly", "[motor_diag]")
{
    motor_cmd_t cmd;
    motor_diag_cmd_desc_t desc;
    char line[200];

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_qe_get(0x05, true, MOTOR_QE_INDEX_COUNTS_PER_REVOLUTION, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_GET, desc.kind);
    TEST_ASSERT_EQUAL_STRING("QE", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Counts per revolution", desc.semantic_name);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 QE[4] GET (Counts per revolution)", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mp_set_u16(0x05, true, MOTOR_MP_INDEX_QUEUE_LOW_THRESHOLD, 9U, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_SET, desc.kind);
    TEST_ASSERT_EQUAL_STRING("MP", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Queue low threshold", desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_U16, desc.value.kind);
    TEST_ASSERT_EQUAL(9, desc.value.raw_number);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 MP[5] SET (Queue low threshold) = 9", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bl_get(0x05, true, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_GET, desc.kind);
    TEST_ASSERT_EQUAL_STRING("BL", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Backlash compensation", desc.semantic_name);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 BL GET (Backlash compensation)", line);
}

TEST_CASE("motor diag keeps ambiguous newer semantics partial instead of overstating confidence", "[motor_diag]")
{
    motor_cmd_t cmd;
    motor_diag_cmd_desc_t cmd_desc;
    motor_rx_t rx;
    motor_diag_rx_desc_t rx_desc;
    twai_message_t mp_zero_msg = make_ext_frame(0x05, 0x22, 3U, (const uint8_t[]){0x04, 0x00, 0x00});

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mp_get(0x05, true, MOTOR_MP_INDEX_CURRENT_QUEUE_LEVEL_OR_RESET_TABLE, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &cmd_desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_COMPLETENESS_PARTIAL, cmd_desc.completeness);
    TEST_ASSERT_EQUAL_STRING("Queue level or table control parameter", cmd_desc.semantic_name);
    TEST_ASSERT_EQUAL_STRING("parameter meaning is only partially verified", cmd_desc.note);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&mp_zero_msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &rx_desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_COMPLETENESS_PARTIAL, rx_desc.completeness);
    TEST_ASSERT_EQUAL_STRING("zero may have a special meaning", rx_desc.note);
}

TEST_CASE("motor diag preserves LM GET versus SET structural base distinction", "[motor_diag]")
{
    uint8_t get_data[5] = {MOTOR_LM_INDEX_MAX_ACCEL_DECEL};
    uint8_t set_data[5] = {MOTOR_LM_INDEX_MAX_ACCEL_DECEL};
    twai_message_t get_msg;
    twai_message_t set_msg;
    motor_rx_t rx;
    motor_diag_rx_desc_t desc;

    motor_codec_pack_i32_le(&get_data[1], 3333);
    motor_codec_pack_i32_le(&set_data[1], -4444);
    get_msg = make_ext_frame(0x2F, 0x2C, 5U, get_data);
    set_msg = make_ext_frame(0x2F, 0x24, 5U, set_data);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&get_msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_LM, desc.object);
    TEST_ASSERT_EQUAL_HEX8(0x2C, desc.base_code);
    TEST_ASSERT_FALSE(desc.has_semantic_lineage);

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&set_msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_LM, desc.object);
    TEST_ASSERT_EQUAL_HEX8(0x24, desc.base_code);
    TEST_ASSERT_FALSE(desc.has_semantic_lineage);
}

TEST_CASE("motor diag keeps truthful fallback for unclassified valid RX", "[motor_diag]")
{
    static const uint8_t data[] = {0xAA, 0x55};
    twai_message_t msg = make_ext_frame(0x33, 0x33, 2U, data);
    motor_rx_t rx;
    motor_diag_rx_desc_t desc;
    char line[160];

    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_UNCLASSIFIED, desc.kind);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_NONE, desc.object);
    TEST_ASSERT_EQUAL_HEX8(0x33, desc.base_code);
    TEST_ASSERT_EQUAL_STRING("unknown semantic mapping", desc.note);
    TEST_ASSERT_EQUAL_UINT8(2U, desc.raw_len);

    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=51 raw d0=0xAA d1=0x55 (unknown semantic mapping)", line);
}

TEST_CASE("motor diag describes and formats manual command examples across PP IC ER and SY", "[motor_diag]")
{
    motor_cmd_t cmd;
    motor_diag_cmd_desc_t desc;
    char line[160];

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pp_set_u8(0x05, true, MOTOR_PP_INDEX_BITRATE, 0x02, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_SET, desc.kind);
    TEST_ASSERT_EQUAL_STRING("PP", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("CAN bitrate", desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_ENUM, desc.value.kind);
    TEST_ASSERT_TRUE(desc.value.has_raw_number);
    TEST_ASSERT_EQUAL_INT32(2, desc.value.raw_number);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 PP[5] SET (CAN bitrate) = 500000 bps", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_ic_set_u16(0x05, true, MOTOR_IC_INDEX_POSITIVE_MOTOR_DIRECTION, 1U, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_SET, desc.kind);
    TEST_ASSERT_EQUAL_STRING("IC", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Positive Motor Direction", desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_U16, desc.value.kind);
    TEST_ASSERT_TRUE(desc.value.has_raw_number);
    TEST_ASSERT_EQUAL_INT32(1, desc.value.raw_number);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 IC[1] SET (Positive Motor Direction) = 1", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_er_clear_all(0x05, true, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_ACTION, desc.kind);
    TEST_ASSERT_EQUAL_STRING("ER", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Clear all errors", desc.semantic_name);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 ER[0] ACTION (Clear all errors)", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_sy_restore_factory_defaults(0x05, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_ACTION, desc.kind);
    TEST_ASSERT_EQUAL_STRING("SY", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Restore factory defaults", desc.semantic_name);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 SY[2] ACTION (Restore factory defaults)", line);
}

TEST_CASE("motor diag describes and formats manual command examples across MT MO BG ST PA OG and LM", "[motor_diag]")
{
    motor_cmd_t cmd;
    motor_diag_cmd_desc_t desc;
    char line[160];

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mt_set_u16(0x05, true, MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS, 1000U, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL_STRING("Delay of Automatic Enable after Power-on", desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_U16, desc.value.kind);
    TEST_ASSERT_EQUAL_INT32(1000, desc.value.raw_number);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 MT[3] SET (Delay of Automatic Enable after Power-on) = 1000 ms", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_mo_set(0x05, true, MOTOR_MO_STATE_DISABLE, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_FALSE(desc.has_index);
    TEST_ASSERT_EQUAL_STRING("Motor driver", desc.semantic_name);
    TEST_ASSERT_EQUAL_STRING("OFF", desc.value.text);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 MO SET (Motor driver) = OFF", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_bg_begin(0x05, true, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_ACTION, desc.kind);
    TEST_ASSERT_EQUAL_STRING("Begin motion", desc.semantic_name);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 BG ACTION (Begin motion)", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_st_stop(0x05, true, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL_STRING("Stop motion", desc.semantic_name);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 ST ACTION (Stop motion)", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_pa_set(0x05, true, 1000, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_FALSE(desc.has_index);
    TEST_ASSERT_EQUAL_STRING("Absolute position", desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_I32, desc.value.kind);
    TEST_ASSERT_EQUAL_INT32(1000, desc.value.raw_number);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 PA SET (Absolute position) = 1000 pulse", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_og_set_origin(0x05, true, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_EQUAL_STRING("Set origin", desc.semantic_name);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 OG ACTION (Set origin)", line);

    TEST_ASSERT_EQUAL(ESP_OK, motor_cmd_lm_set_i32(0x05, true, MOTOR_LM_INDEX_ENABLE_DISABLE, 0, &cmd));
    TEST_ASSERT_TRUE(motor_diag_describe_cmd(&cmd, &desc));
    TEST_ASSERT_TRUE(desc.has_index);
    TEST_ASSERT_EQUAL_UINT16(255U, desc.index);
    TEST_ASSERT_EQUAL_STRING("Enable/disable software limits", desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_I32, desc.value.kind);
    TEST_ASSERT_EQUAL_INT32(0, desc.value.raw_number);
    motor_diag_format_cmd(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("TX node=5 LM[255] SET (Enable/disable software limits) = 0", line);
}

TEST_CASE("motor diag describes and formats manual ACK examples across PP IC MT MO BG ST PA OG and LM", "[motor_diag]")
{
    twai_message_t msg;
    motor_rx_t rx;
    motor_diag_rx_desc_t desc;
    char line[200];
    uint8_t pa_get_data[4];
    uint8_t pa_set_data[5] = {0x04};
    uint8_t lm_get_data[5] = {0x01};
    uint8_t lm_set_data[5] = {0x02};

    msg = make_ext_frame(0x05, 0x01, 2U, (const uint8_t[]){0x05, 0x03});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_ACK, desc.kind);
    TEST_ASSERT_EQUAL_STRING("PP", desc.family_symbol);
    TEST_ASSERT_TRUE(desc.has_index);
    TEST_ASSERT_EQUAL_UINT16(5U, desc.index);
    TEST_ASSERT_EQUAL_STRING("CAN bitrate", desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_ENUM, desc.value.kind);
    TEST_ASSERT_EQUAL_INT32(3, desc.value.raw_number);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK PP[5] CAN bitrate = 250000 bps", line);

    msg = make_ext_frame(0x05, 0x06, 3U, (const uint8_t[]){0x01, 0x01, 0x00});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL_STRING("IC", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Positive Motor Direction", desc.semantic_name);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_U16, desc.value.kind);
    TEST_ASSERT_EQUAL_INT32(1, desc.value.raw_number);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK IC[1] Positive Motor Direction = 1", line);

    msg = make_ext_frame(0x05, 0x10, 3U, (const uint8_t[]){0x05, 0x01, 0x00});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL_STRING("MT", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Enable/Release Internal Controlled Brake", desc.semantic_name);
    TEST_ASSERT_EQUAL_STRING("ON", desc.value.text);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK MT[5] Enable/Release Internal Controlled Brake = ON", line);

    msg = make_ext_frame(0x05, 0x15, 1U, (const uint8_t[]){0x01});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_FALSE(desc.has_semantic_lineage);
    TEST_ASSERT_EQUAL_STRING("Motor driver", desc.semantic_name);
    TEST_ASSERT_EQUAL_STRING("ON", desc.value.text);

    msg = make_ext_frame(0x05, 0x16, 0U, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL_STRING("BG", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Begin motion", desc.semantic_name);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK BG Begin motion", line);

    msg = make_ext_frame(0x05, 0x17, 0U, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL_STRING("ST", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Stop motion", desc.semantic_name);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK ST Stop motion", line);

    motor_codec_pack_i32_le(pa_get_data, -1000);
    msg = make_ext_frame(0x05, 0x20, 4U, pa_get_data);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL_STRING("PA", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Absolute position", desc.semantic_name);
    TEST_ASSERT_EQUAL_INT32(-1000, desc.value.raw_number);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK PA Absolute position = -1000 pulse", line);

    motor_codec_pack_i32_le(&pa_set_data[1], 1000);
    msg = make_ext_frame(0x05, 0x2E, 5U, pa_set_data);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_DV, desc.object);
    TEST_ASSERT_TRUE(desc.has_semantic_lineage);
    TEST_ASSERT_EQUAL_STRING("PA", desc.semantic_family_symbol);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_CMD_KIND_SET, desc.semantic_cmd_kind);
    TEST_ASSERT_EQUAL_INT32(1000, desc.value.raw_number);

    msg = make_ext_frame(0x05, 0x21, 0U, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL_STRING("OG", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Set origin", desc.semantic_name);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK OG Set origin", line);

    motor_codec_pack_i32_le(&lm_get_data[1], 4124);
    msg = make_ext_frame(0x05, 0x2C, 5U, lm_get_data);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL_HEX8(0x2C, desc.base_code);
    TEST_ASSERT_EQUAL_UINT16(1U, desc.index);
    TEST_ASSERT_EQUAL_STRING("Lower Working Limit", desc.semantic_name);
    TEST_ASSERT_EQUAL_INT32(4124, desc.value.raw_number);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK LM[1] Lower Working Limit = 4124 pulse", line);

    motor_codec_pack_i32_le(&lm_set_data[1], 10000);
    msg = make_ext_frame(0x05, 0x24, 5U, lm_set_data);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL_HEX8(0x24, desc.base_code);
    TEST_ASSERT_EQUAL_UINT16(2U, desc.index);
    TEST_ASSERT_EQUAL_STRING("Upper Working Limit", desc.semantic_name);
    TEST_ASSERT_EQUAL_INT32(10000, desc.value.raw_number);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ACK LM[2] Upper Working Limit = 10000 pulse", line);
}

TEST_CASE("motor diag describes and formats manual ER and RT examples truthfully", "[motor_diag]")
{
    twai_message_t msg;
    motor_rx_t rx;
    motor_diag_rx_desc_t desc;
    char line[200];
    uint8_t completion_data[6] = {0x29, 0x00, 0x00, 0x00, 0x00, 0x00};

    msg = make_ext_frame(0x05, 0x0F, 6U, (const uint8_t[]){0x00, 0x33, 0x81, 0x05, 0x00, 0x00});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_ERROR, desc.kind);
    TEST_ASSERT_TRUE(desc.has_error_code);
    TEST_ASSERT_EQUAL_HEX8(0x33, desc.error_code);
    TEST_ASSERT_EQUAL_STRING("Instruction data error", desc.error_name);
    TEST_ASSERT_TRUE(desc.has_related_family);
    TEST_ASSERT_EQUAL_STRING("PP", desc.related_family_symbol);
    TEST_ASSERT_TRUE(desc.has_related_index);
    TEST_ASSERT_EQUAL_UINT16(5U, desc.related_index);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 ERROR code=0x33 (Instruction data error) related=PP[5]", line);

    msg = make_ext_frame(0x05, 0x5A, 2U, (const uint8_t[]){0x00, 0x01});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_RT_ALARM, desc.kind);
    TEST_ASSERT_EQUAL_STRING("RT", desc.family_symbol);
    TEST_ASSERT_EQUAL_STRING("Emergency stop and lock", desc.semantic_name);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 RT alarm Emergency stop and lock", line);

    msg = make_ext_frame(0x05, 0x5A, 2U, (const uint8_t[]){0x11, 0x00});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_RT_STATUS, desc.kind);
    TEST_ASSERT_EQUAL_STRING("Input 1 falling edge", desc.semantic_name);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 RT status Input 1 falling edge", line);

    msg = make_ext_frame(0x05, 0x5A, 2U, (const uint8_t[]){0x12, 0x00});
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_RT_STATUS, desc.kind);
    TEST_ASSERT_EQUAL_STRING("Input 1 rising edge", desc.semantic_name);

    motor_codec_pack_i32_le(&completion_data[2], 1000);
    msg = make_ext_frame(0x05, 0x5A, 6U, completion_data);
    TEST_ASSERT_EQUAL(ESP_OK, motor_rx_parse(&msg, &rx));
    TEST_ASSERT_TRUE(motor_diag_describe_rx(&rx, &desc));
    TEST_ASSERT_EQUAL(MOTOR_DIAG_RX_KIND_RT_STATUS, desc.kind);
    TEST_ASSERT_EQUAL_STRING("PTP positioning completed", desc.semantic_name);
    TEST_ASSERT_TRUE(desc.value.has_value);
    TEST_ASSERT_EQUAL(MOTOR_DIAG_VALUE_KIND_I32, desc.value.kind);
    TEST_ASSERT_EQUAL_INT32(1000, desc.value.number);
    TEST_ASSERT_FALSE(desc.value.has_raw_number);
    motor_diag_format_rx(line, sizeof(line), &desc);
    TEST_ASSERT_EQUAL_STRING("RX node=5 RT status PTP positioning completed, current position = 1000 pulse", line);
}
