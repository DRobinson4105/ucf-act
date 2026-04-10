/*
 * Responsibility:
 * Private mechanical codec API shared by TX builders and RX parsing.
 */

#ifndef MOTOR_CODEC_H
#define MOTOR_CODEC_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"
#include "motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

uint8_t motor_codec_compose_cw(uint8_t base_code, bool ack_requested);
uint8_t motor_codec_base_code(uint8_t cw_raw);
bool motor_codec_ack_requested(uint8_t cw_raw);

uint16_t motor_codec_encode_sid(uint8_t logical_id);
uint32_t motor_codec_encode_eid(uint8_t logical_id, uint8_t cw_raw);
uint32_t motor_codec_build_ext_id(uint8_t logical_id, uint8_t cw_raw);

uint16_t motor_codec_extract_sid(uint32_t ext_id);
uint32_t motor_codec_extract_eid(uint32_t ext_id);
uint8_t motor_codec_decode_id(uint32_t ext_id);
uint8_t motor_codec_decode_cw(uint32_t ext_id);

void motor_codec_pack_u16_le(uint8_t *dst, uint16_t value);
void motor_codec_pack_u32_le(uint8_t *dst, uint32_t value);
void motor_codec_pack_i32_le(uint8_t *dst, int32_t value);
uint16_t motor_codec_unpack_u16_le(const uint8_t *src);
uint32_t motor_codec_unpack_u32_le(const uint8_t *src);
int32_t motor_codec_unpack_i32_le(const uint8_t *src);

esp_err_t motor_codec_init_cmd(
    uint8_t target_id,
    motor_section_t section,
    motor_object_t object,
    uint8_t base_code,
    bool ack_requested,
    uint8_t dlc,
    const char *name,
    motor_cmd_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_CODEC_H */
