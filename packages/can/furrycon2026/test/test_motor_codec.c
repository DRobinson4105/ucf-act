/*
 * Responsibility:
 * Unity coverage for the mechanical motor codec layer.
 */

#include <inttypes.h>
#include <limits.h>
#include <string.h>

#include "unity.h"

#include "esp_err.h"
#include "motor_codec.h"
#include "test_log.h"

TEST_CASE("motor codec composes and decomposes CW", "[motor_protocol]")
{
    LOG_SECTION("motor codec CW composition");
    uint8_t composed = motor_codec_compose_cw(0x24, true);
    uint8_t base_plain = motor_codec_base_code(0x24);
    uint8_t base_composed = motor_codec_base_code(composed);
    bool ack_plain = motor_codec_ack_requested(0x24);
    bool ack_composed = motor_codec_ack_requested(composed);

    LOG_INPUT("base_code=0x%02X ack_requested=%d", 0x24, true);
    LOG_OUTPUT("composed=0x%02X base_plain=0x%02X base_composed=0x%02X ack_plain=%d ack_composed=%d",
               composed, base_plain, base_composed, ack_plain, ack_composed);

    TEST_ASSERT_EQUAL_HEX8(0xA4, composed);
    TEST_ASSERT_EQUAL_HEX8(0x24, base_plain);
    TEST_ASSERT_EQUAL_HEX8(0x24, base_composed);
    TEST_ASSERT_FALSE(ack_plain);
    TEST_ASSERT_TRUE(ack_composed);
}

TEST_CASE("motor codec round trips extended ID", "[motor_protocol]")
{
    LOG_SECTION("motor codec extended ID round trip");
    uint32_t ext_id = motor_codec_build_ext_id(0x05, 0x81);
    uint8_t decoded_id = motor_codec_decode_id(ext_id);
    uint8_t decoded_cw = motor_codec_decode_cw(ext_id);

    LOG_INPUT("logical_id=0x%02X cw_raw=0x%02X", 0x05, 0x81);
    LOG_OUTPUT("ext_id=0x%08" PRIX32 " decoded_id=0x%02X decoded_cw=0x%02X",
               ext_id, decoded_id, decoded_cw);

    TEST_ASSERT_EQUAL_HEX8(0x05, decoded_id);
    TEST_ASSERT_EQUAL_HEX8(0x81, decoded_cw);
}

TEST_CASE("motor codec round trips upper logical ID bits", "[motor_protocol]")
{
    LOG_SECTION("motor codec upper logical ID bits");
    uint32_t ext_id = motor_codec_build_ext_id(0x65, 0x81);
    uint8_t decoded_id = motor_codec_decode_id(ext_id);
    uint8_t decoded_cw = motor_codec_decode_cw(ext_id);

    LOG_INPUT("logical_id=0x%02X cw_raw=0x%02X", 0x65, 0x81);
    LOG_OUTPUT("ext_id=0x%08" PRIX32 " decoded_id=0x%02X decoded_cw=0x%02X",
               ext_id, decoded_id, decoded_cw);

    TEST_ASSERT_EQUAL_HEX8(0x65, decoded_id);
    TEST_ASSERT_EQUAL_HEX8(0x81, decoded_cw);
}

TEST_CASE("motor codec round trips logical ID boundaries", "[motor_protocol]")
{
    LOG_SECTION("motor codec logical ID boundaries");
    uint32_t ext_id_min = motor_codec_build_ext_id(0x00, 0x00);
    uint32_t ext_id_low_mid = motor_codec_build_ext_id(0x01, 0x12);
    uint32_t ext_id_high_mid = motor_codec_build_ext_id(0x3F, 0x55);
    uint32_t ext_id_max = motor_codec_build_ext_id(0x7F, 0x7F);

    LOG_INPUT("min logical_id=0x%02X cw_raw=0x%02X", 0x00, 0x00);
    LOG_OUTPUT("min ext_id=0x%08" PRIX32 " decoded_id=0x%02X decoded_cw=0x%02X",
               ext_id_min, motor_codec_decode_id(ext_id_min), motor_codec_decode_cw(ext_id_min));
    LOG_INPUT("low-mid logical_id=0x%02X cw_raw=0x%02X", 0x01, 0x12);
    LOG_OUTPUT("low-mid ext_id=0x%08" PRIX32 " decoded_id=0x%02X decoded_cw=0x%02X",
               ext_id_low_mid, motor_codec_decode_id(ext_id_low_mid), motor_codec_decode_cw(ext_id_low_mid));
    LOG_INPUT("high-mid logical_id=0x%02X cw_raw=0x%02X", 0x3F, 0x55);
    LOG_OUTPUT("high-mid ext_id=0x%08" PRIX32 " decoded_id=0x%02X decoded_cw=0x%02X",
               ext_id_high_mid, motor_codec_decode_id(ext_id_high_mid), motor_codec_decode_cw(ext_id_high_mid));
    LOG_INPUT("max logical_id=0x%02X cw_raw=0x%02X", 0x7F, 0x7F);
    LOG_OUTPUT("max ext_id=0x%08" PRIX32 " decoded_id=0x%02X decoded_cw=0x%02X",
               ext_id_max, motor_codec_decode_id(ext_id_max), motor_codec_decode_cw(ext_id_max));

    TEST_ASSERT_EQUAL_HEX8(0x00, motor_codec_decode_id(ext_id_min));
    TEST_ASSERT_EQUAL_HEX8(0x00, motor_codec_decode_cw(ext_id_min));
    TEST_ASSERT_EQUAL_HEX32(0x04000000, ext_id_min);
    TEST_ASSERT_EQUAL_HEX8(0x01, motor_codec_decode_id(ext_id_low_mid));
    TEST_ASSERT_EQUAL_HEX8(0x12, motor_codec_decode_cw(ext_id_low_mid));
    TEST_ASSERT_EQUAL_HEX32(0x04080012, ext_id_low_mid);
    TEST_ASSERT_EQUAL_HEX8(0x3F, motor_codec_decode_id(ext_id_high_mid));
    TEST_ASSERT_EQUAL_HEX8(0x55, motor_codec_decode_cw(ext_id_high_mid));
    TEST_ASSERT_EQUAL_HEX32(0x04F84055, ext_id_high_mid);
    TEST_ASSERT_EQUAL_HEX8(0x7F, motor_codec_decode_id(ext_id_max));
    TEST_ASSERT_EQUAL_HEX8(0x7F, motor_codec_decode_cw(ext_id_max));
    TEST_ASSERT_EQUAL_HEX32(0x04F8C07F, ext_id_max);
}

TEST_CASE("motor codec packs and unpacks little endian values", "[motor_protocol]")
{
    LOG_SECTION("motor codec little endian packing");
    uint8_t row_bytes[2] = {0};
    uint8_t pulse_bytes[4] = {0};
    uint16_t unpacked_u16;
    int32_t unpacked_i32;

    motor_codec_pack_u16_le(row_bytes, 266U);
    motor_codec_pack_i32_le(pulse_bytes, 100000);
    unpacked_u16 = motor_codec_unpack_u16_le(row_bytes);
    unpacked_i32 = motor_codec_unpack_i32_le(pulse_bytes);

    LOG_INPUT("u16=%u i32=%ld", 266U, (long)100000);
    LOG_OUTPUT("u16 bytes=[%02X %02X] unpacked=%u",
               row_bytes[0], row_bytes[1], unpacked_u16);
    LOG_OUTPUT("i32 bytes=[%02X %02X %02X %02X] unpacked=%ld",
               pulse_bytes[0], pulse_bytes[1], pulse_bytes[2], pulse_bytes[3], (long)unpacked_i32);

    TEST_ASSERT_EQUAL_HEX8(0x0A, row_bytes[0]);
    TEST_ASSERT_EQUAL_HEX8(0x01, row_bytes[1]);
    TEST_ASSERT_EQUAL_HEX8(0xA0, pulse_bytes[0]);
    TEST_ASSERT_EQUAL_HEX8(0x86, pulse_bytes[1]);
    TEST_ASSERT_EQUAL_HEX8(0x01, pulse_bytes[2]);
    TEST_ASSERT_EQUAL_HEX8(0x00, pulse_bytes[3]);
    TEST_ASSERT_EQUAL_UINT16(266U, unpacked_u16);
    TEST_ASSERT_EQUAL_INT32(100000, unpacked_i32);
}

TEST_CASE("motor codec packs and unpacks u16 boundaries", "[motor_protocol]")
{
    LOG_SECTION("motor codec u16 boundaries");
    uint8_t zero_bytes[2] = {0};
    uint8_t max_bytes[2] = {0};

    motor_codec_pack_u16_le(zero_bytes, 0U);
    motor_codec_pack_u16_le(max_bytes, UINT16_MAX);

    LOG_INPUT("u16 values=[0, %u]", UINT16_MAX);
    LOG_OUTPUT("0 bytes=[%02X %02X] unpacked=%u",
               zero_bytes[0], zero_bytes[1], motor_codec_unpack_u16_le(zero_bytes));
    LOG_OUTPUT("UINT16_MAX bytes=[%02X %02X] unpacked=%u",
               max_bytes[0], max_bytes[1], motor_codec_unpack_u16_le(max_bytes));

    TEST_ASSERT_EQUAL_HEX8(0x00, zero_bytes[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, zero_bytes[1]);
    TEST_ASSERT_EQUAL_UINT16(0U, motor_codec_unpack_u16_le(zero_bytes));

    TEST_ASSERT_EQUAL_HEX8(0xFF, max_bytes[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, max_bytes[1]);
    TEST_ASSERT_EQUAL_UINT16(UINT16_MAX, motor_codec_unpack_u16_le(max_bytes));
}

TEST_CASE("motor codec packs and unpacks negative i32 values", "[motor_protocol]")
{
    LOG_SECTION("motor codec negative i32 values");
    uint8_t minus_one_bytes[4] = {0};
    uint8_t minus_mid_bytes[4] = {0};
    uint8_t minus_min_bytes[4] = {0};

    motor_codec_pack_i32_le(minus_one_bytes, -1);
    motor_codec_pack_i32_le(minus_mid_bytes, -12345);
    motor_codec_pack_i32_le(minus_min_bytes, INT32_MIN);

    LOG_INPUT("i32 values=[-1, -12345, %ld]", (long)INT32_MIN);
    LOG_OUTPUT("-1 bytes=[%02X %02X %02X %02X] unpacked=%ld",
               minus_one_bytes[0], minus_one_bytes[1], minus_one_bytes[2], minus_one_bytes[3],
               (long)motor_codec_unpack_i32_le(minus_one_bytes));
    LOG_OUTPUT("-12345 bytes=[%02X %02X %02X %02X] unpacked=%ld",
               minus_mid_bytes[0], minus_mid_bytes[1], minus_mid_bytes[2], minus_mid_bytes[3],
               (long)motor_codec_unpack_i32_le(minus_mid_bytes));
    LOG_OUTPUT("INT32_MIN bytes=[%02X %02X %02X %02X] unpacked=%ld",
               minus_min_bytes[0], minus_min_bytes[1], minus_min_bytes[2], minus_min_bytes[3],
               (long)motor_codec_unpack_i32_le(minus_min_bytes));

    TEST_ASSERT_EQUAL_HEX8(0xFF, minus_one_bytes[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, minus_one_bytes[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, minus_one_bytes[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, minus_one_bytes[3]);
    TEST_ASSERT_EQUAL_INT32(-1, motor_codec_unpack_i32_le(minus_one_bytes));

    TEST_ASSERT_EQUAL_HEX8(0xC7, minus_mid_bytes[0]);
    TEST_ASSERT_EQUAL_HEX8(0xCF, minus_mid_bytes[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, minus_mid_bytes[2]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, minus_mid_bytes[3]);
    TEST_ASSERT_EQUAL_INT32(-12345, motor_codec_unpack_i32_le(minus_mid_bytes));

    TEST_ASSERT_EQUAL_HEX8(0x00, minus_min_bytes[0]);
    TEST_ASSERT_EQUAL_HEX8(0x00, minus_min_bytes[1]);
    TEST_ASSERT_EQUAL_HEX8(0x00, minus_min_bytes[2]);
    TEST_ASSERT_EQUAL_HEX8(0x80, minus_min_bytes[3]);
    TEST_ASSERT_EQUAL_INT32(INT32_MIN, motor_codec_unpack_i32_le(minus_min_bytes));
}

TEST_CASE("motor codec packs and unpacks positive i32 boundary", "[motor_protocol]")
{
    LOG_SECTION("motor codec positive i32 boundary");
    uint8_t max_bytes[4] = {0};

    motor_codec_pack_i32_le(max_bytes, INT32_MAX);

    LOG_INPUT("i32 value=%ld", (long)INT32_MAX);
    LOG_OUTPUT("INT32_MAX bytes=[%02X %02X %02X %02X] unpacked=%ld",
               max_bytes[0], max_bytes[1], max_bytes[2], max_bytes[3],
               (long)motor_codec_unpack_i32_le(max_bytes));

    TEST_ASSERT_EQUAL_HEX8(0xFF, max_bytes[0]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, max_bytes[1]);
    TEST_ASSERT_EQUAL_HEX8(0xFF, max_bytes[2]);
    TEST_ASSERT_EQUAL_HEX8(0x7F, max_bytes[3]);
    TEST_ASSERT_EQUAL_INT32(INT32_MAX, motor_codec_unpack_i32_le(max_bytes));
}

TEST_CASE("motor codec CW boundaries and masking", "[motor_protocol]")
{
    LOG_SECTION("motor codec CW boundaries");
    uint8_t composed_min = motor_codec_compose_cw(0x00, false);
    uint8_t composed_zero_ack = motor_codec_compose_cw(0x00, true);
    uint8_t composed_mid = motor_codec_compose_cw(0x55, false);
    uint8_t composed_max = motor_codec_compose_cw(0x7F, true);
    uint8_t composed_masked = motor_codec_compose_cw(0xFF, false);

    LOG_INPUT("min base_code=0x00 ack_requested=0");
    LOG_OUTPUT("min composed=0x%02X", composed_min);
    LOG_INPUT("zero-ack base_code=0x00 ack_requested=1");
    LOG_OUTPUT("zero-ack composed=0x%02X base=0x%02X ack=%d",
               composed_zero_ack, motor_codec_base_code(composed_zero_ack),
               motor_codec_ack_requested(composed_zero_ack));
    LOG_INPUT("mid base_code=0x55 ack_requested=0");
    LOG_OUTPUT("mid composed=0x%02X base=0x%02X ack=%d",
               composed_mid, motor_codec_base_code(composed_mid),
               motor_codec_ack_requested(composed_mid));
    LOG_INPUT("max base_code=0x7F ack_requested=1");
    LOG_OUTPUT("max composed=0x%02X", composed_max);
    LOG_INPUT("masked base_code=0xFF ack_requested=0");
    LOG_OUTPUT("masked composed=0x%02X base=0x%02X ack=%d",
               composed_masked, motor_codec_base_code(composed_masked), motor_codec_ack_requested(composed_masked));

    TEST_ASSERT_EQUAL_HEX8(0x00, composed_min);
    TEST_ASSERT_EQUAL_HEX8(0x80, composed_zero_ack);
    TEST_ASSERT_EQUAL_HEX8(0x00, motor_codec_base_code(composed_zero_ack));
    TEST_ASSERT_TRUE(motor_codec_ack_requested(composed_zero_ack));
    TEST_ASSERT_EQUAL_HEX8(0x55, composed_mid);
    TEST_ASSERT_EQUAL_HEX8(0x55, motor_codec_base_code(composed_mid));
    TEST_ASSERT_FALSE(motor_codec_ack_requested(composed_mid));
    TEST_ASSERT_EQUAL_HEX8(0xFF, composed_max);
    TEST_ASSERT_EQUAL_HEX8(0x7F, composed_masked);
    TEST_ASSERT_EQUAL_HEX8(0x7F, motor_codec_base_code(composed_masked));
    TEST_ASSERT_FALSE(motor_codec_ack_requested(composed_masked));
}

TEST_CASE("motor codec extracts sid and eid from known extended ID", "[motor_protocol]")
{
    LOG_SECTION("motor codec SID and EID extraction");
    uint32_t ext_id = 0x0428C081;

    LOG_INPUT("ext_id=0x%08" PRIX32, ext_id);
    LOG_OUTPUT("sid=0x%03X eid=0x%05" PRIX32,
               motor_codec_extract_sid(ext_id), motor_codec_extract_eid(ext_id));

    TEST_ASSERT_EQUAL_HEX16(0x10A, motor_codec_extract_sid(ext_id));
    TEST_ASSERT_EQUAL_HEX32(0x0C081, motor_codec_extract_eid(ext_id));
    TEST_ASSERT_EQUAL_HEX8(0x65, motor_codec_decode_id(ext_id));
    TEST_ASSERT_EQUAL_HEX8(0x81, motor_codec_decode_cw(ext_id));
}

TEST_CASE("motor codec initializes extended data command", "[motor_protocol]")
{
    LOG_SECTION("motor codec init command");
    motor_cmd_t cmd;
    esp_err_t err;

    err = motor_codec_init_cmd(0x05, MOTOR_SECTION_NONE, MOTOR_OBJECT_NONE, 0x01, true, 4U,
                               "codec_init", &cmd);

    LOG_INPUT("target_id=0x%02X section=%d object=%d base_code=0x%02X ack_requested=%d dlc=%u name=%s",
              0x05, MOTOR_SECTION_NONE, MOTOR_OBJECT_NONE, 0x01, true, 4U, "codec_init");
    LOG_OUTPUT("err=%s target_id=0x%02X cw_raw=0x%02X base_code=0x%02X ack_requested=%d ext_id=0x%08" PRIX32 " dlc=%u",
               esp_err_to_name(err), cmd.target_id, cmd.cw_raw, cmd.base_code, cmd.ack_requested,
               cmd.msg.identifier, cmd.msg.data_length_code);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_HEX8(0x05, cmd.target_id);
    TEST_ASSERT_EQUAL_HEX8(0x81, cmd.cw_raw);
    TEST_ASSERT_EQUAL_HEX8(0x01, cmd.base_code);
    TEST_ASSERT_TRUE(cmd.ack_requested);
    TEST_ASSERT_EQUAL(MOTOR_SECTION_NONE, cmd.section);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_NONE, cmd.object);
    TEST_ASSERT_FALSE(cmd.has_index);
    TEST_ASSERT_EQUAL_STRING("codec_init", cmd.name);
    TEST_ASSERT_TRUE(cmd.msg.extd);
    TEST_ASSERT_FALSE(cmd.msg.rtr);
    TEST_ASSERT_FALSE(cmd.msg.self);
    TEST_ASSERT_FALSE(cmd.msg.ss);
    TEST_ASSERT_FALSE(cmd.msg.dlc_non_comp);
    TEST_ASSERT_EQUAL_UINT8(4U, cmd.msg.data_length_code);
    TEST_ASSERT_EQUAL_HEX32(motor_codec_build_ext_id(0x05, 0x81),
                            cmd.msg.identifier);
}

TEST_CASE("motor codec init command rejects invalid arguments", "[motor_protocol]")
{
    LOG_SECTION("motor codec init validation");
    motor_cmd_t cmd;
    esp_err_t err_null = motor_codec_init_cmd(0x05, MOTOR_SECTION_NONE, MOTOR_OBJECT_NONE, 0x01, true, 4U,
                                              "codec_init", NULL);
    esp_err_t err_bad_dlc = motor_codec_init_cmd(0x05, MOTOR_SECTION_NONE, MOTOR_OBJECT_NONE, 0x01, true, 9U,
                                                 "codec_init", &cmd);

    LOG_INPUT("null out: target_id=0x05 dlc=4");
    LOG_ERROR("null out err=%s", esp_err_to_name(err_null));
    LOG_INPUT("bad dlc: target_id=0x05 dlc=9");
    LOG_ERROR("bad dlc err=%s", esp_err_to_name(err_bad_dlc));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err_null);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err_bad_dlc);
}

TEST_CASE("motor codec init command zeroes prefilled struct", "[motor_protocol]")
{
    LOG_SECTION("motor codec init zeroes struct");
    motor_cmd_t cmd;
    esp_err_t err;

    memset(&cmd, 0xA5, sizeof(cmd));
    err = motor_codec_init_cmd(0x05, MOTOR_SECTION_NONE, MOTOR_OBJECT_NONE, 0x01, false, 0U,
                               "codec_zero", &cmd);

    LOG_INPUT("prefilled cmd=0xA5 pattern then init target_id=0x05 dlc=0");
    LOG_OUTPUT("err=%s identifier=0x%08" PRIX32 " extd=%d rtr=%d self=%d ss=%d dlc_non_comp=%d dlc=%u",
               esp_err_to_name(err), cmd.msg.identifier, cmd.msg.extd, cmd.msg.rtr,
               cmd.msg.self, cmd.msg.ss, cmd.msg.dlc_non_comp, cmd.msg.data_length_code);

    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_HEX8(0x05, cmd.target_id);
    TEST_ASSERT_EQUAL_HEX8(0x01, cmd.cw_raw);
    TEST_ASSERT_EQUAL_HEX8(0x01, cmd.base_code);
    TEST_ASSERT_FALSE(cmd.ack_requested);
    TEST_ASSERT_EQUAL(MOTOR_SECTION_NONE, cmd.section);
    TEST_ASSERT_EQUAL(MOTOR_OBJECT_NONE, cmd.object);
    TEST_ASSERT_FALSE(cmd.has_index);
    TEST_ASSERT_EQUAL_STRING("codec_zero", cmd.name);
    TEST_ASSERT_EQUAL_HEX32(motor_codec_build_ext_id(0x05, 0x01), cmd.msg.identifier);
    TEST_ASSERT_TRUE(cmd.msg.extd);
    TEST_ASSERT_FALSE(cmd.msg.rtr);
    TEST_ASSERT_FALSE(cmd.msg.self);
    TEST_ASSERT_FALSE(cmd.msg.ss);
    TEST_ASSERT_FALSE(cmd.msg.dlc_non_comp);
    TEST_ASSERT_EQUAL_UINT8(0U, cmd.msg.data_length_code);
    TEST_ASSERT_EACH_EQUAL_HEX8(0x00, cmd.msg.data, 8);
}
