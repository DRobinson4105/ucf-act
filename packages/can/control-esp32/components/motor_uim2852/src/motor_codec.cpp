/*
 * Responsibility:
 * Mechanical protocol layer for bit, byte, ID, and ACK transformations.
 */

#include "motor_codec.h"

#include <string.h>

#define MOTOR_CODEC_ACK_BIT               0x80U
#define MOTOR_CODEC_BASE_CODE_MASK        0x7FU
#define MOTOR_CODEC_SID_MASK              0x01FFU
#define MOTOR_CODEC_EID_MASK              0x3FFFFU
#define MOTOR_CODEC_EXT_ID_MASK           0x1FFFFFFFU
#define MOTOR_CODEC_MAX_DLC               8U

uint8_t motor_codec_compose_cw(uint8_t base_code, bool ack_requested)
{
    return (uint8_t)(motor_codec_base_code(base_code) |
                     (ack_requested ? MOTOR_CODEC_ACK_BIT : 0U));
}

uint8_t motor_codec_base_code(uint8_t cw_raw)
{
    return (uint8_t)(cw_raw & MOTOR_CODEC_BASE_CODE_MASK);
}

bool motor_codec_ack_requested(uint8_t cw_raw)
{
    return (cw_raw & MOTOR_CODEC_ACK_BIT) != 0U;
}

uint16_t motor_codec_encode_sid(uint8_t logical_id)
{
    return (uint16_t)((((uint16_t)logical_id << 1) & 0x003FU) | 0x0100U);
}

uint32_t motor_codec_encode_eid(uint8_t logical_id, uint8_t cw_raw)
{
    return ((((uint32_t)logical_id << 1) & 0x00C0U) << 8) | (uint32_t)cw_raw;
}

uint32_t motor_codec_build_ext_id(uint8_t logical_id, uint8_t cw_raw)
{
    return motor_codec_build_ext_id_endpoints(0x04U, logical_id, cw_raw);
}

uint32_t motor_codec_build_ext_id_endpoints(uint8_t producer_id, uint8_t consumer_id, uint8_t cw_raw)
{
    uint32_t sid = (uint32_t)((((uint16_t)(producer_id & 0x1FU)) << 6) |
                              (((uint16_t)(consumer_id & 0x1FU)) << 1));
    uint32_t eid = (uint32_t)(((uint32_t)((producer_id >> 5) & 0x03U) << 16) |
                              ((uint32_t)((consumer_id >> 5) & 0x03U) << 14));

    eid |= (uint32_t)cw_raw;

    return ((sid << 18) | eid) & MOTOR_CODEC_EXT_ID_MASK;
}

uint16_t motor_codec_extract_sid(uint32_t ext_id)
{
    return (uint16_t)((ext_id >> 18) & MOTOR_CODEC_SID_MASK);
}

uint32_t motor_codec_extract_eid(uint32_t ext_id)
{
    return ext_id & MOTOR_CODEC_EID_MASK;
}

uint8_t motor_codec_decode_id(uint32_t ext_id)
{
    return motor_codec_decode_consumer_id(ext_id);
}

uint8_t motor_codec_decode_producer_id(uint32_t ext_id)
{
    uint16_t sid = motor_codec_extract_sid(ext_id);
    uint32_t eid = motor_codec_extract_eid(ext_id);

    return (uint8_t)((((eid >> 16) & 0x03U) << 5) |
                     ((sid >> 6) & 0x1FU));
}

uint8_t motor_codec_decode_consumer_id(uint32_t ext_id)
{
    uint16_t sid = motor_codec_extract_sid(ext_id);
    uint32_t eid = motor_codec_extract_eid(ext_id);

    return (uint8_t)((((eid >> 14) & 0x03U) << 5) |
                     ((sid >> 1) & 0x1FU));
}

uint8_t motor_codec_decode_cw(uint32_t ext_id)
{
    return (uint8_t)(motor_codec_extract_eid(ext_id) & 0x00FFU);
}

void motor_codec_pack_u16_le(uint8_t *dst, uint16_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

void motor_codec_pack_u32_le(uint8_t *dst, uint32_t value)
{
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

void motor_codec_pack_i32_le(uint8_t *dst, int32_t value)
{
    uint32_t uvalue = (uint32_t)value;

    dst[0] = (uint8_t)(uvalue & 0xFFU);
    dst[1] = (uint8_t)((uvalue >> 8) & 0xFFU);
    dst[2] = (uint8_t)((uvalue >> 16) & 0xFFU);
    dst[3] = (uint8_t)((uvalue >> 24) & 0xFFU);
}

uint16_t motor_codec_unpack_u16_le(const uint8_t *src)
{
    return (uint16_t)((uint16_t)src[0] |
                      ((uint16_t)src[1] << 8));
}

uint32_t motor_codec_unpack_u32_le(const uint8_t *src)
{
    uint32_t uvalue;

    uvalue = (uint32_t)src[0];
    uvalue |= (uint32_t)src[1] << 8;
    uvalue |= (uint32_t)src[2] << 16;
    uvalue |= (uint32_t)src[3] << 24;

    return uvalue;
}

int32_t motor_codec_unpack_i32_le(const uint8_t *src)
{
    uint32_t uvalue;

    uvalue = (uint32_t)src[0];
    uvalue |= (uint32_t)src[1] << 8;
    uvalue |= (uint32_t)src[2] << 16;
    uvalue |= (uint32_t)src[3] << 24;

    return (int32_t)uvalue;
}

esp_err_t motor_codec_init_cmd(
    uint8_t target_id,
    motor_section_t section,
    motor_object_t object,
    uint8_t base_code,
    bool ack_requested,
    uint8_t dlc,
    const char *name,
    motor_cmd_t *out)
{
    uint8_t cw_raw;

    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (dlc > MOTOR_CODEC_MAX_DLC) {
        return ESP_ERR_INVALID_ARG;
    }

    cw_raw = motor_codec_compose_cw(base_code, ack_requested);

    memset(out, 0, sizeof(*out));
    out->target_id = target_id;
    out->cw_raw = cw_raw;
    out->base_code = motor_codec_base_code(base_code);
    out->ack_requested = ack_requested;
    out->section = section;
    out->object = object;
    out->has_index = false;
    out->index = 0U;
    out->name = name;

    out->msg.identifier = motor_codec_build_ext_id(target_id, cw_raw);
    out->msg.extd = 1;
    out->msg.rtr = 0;
    out->msg.ss = 0;
    out->msg.self = 0;
    out->msg.dlc_non_comp = 0;
    out->msg.data_length_code = dlc;

    return ESP_OK;
}
