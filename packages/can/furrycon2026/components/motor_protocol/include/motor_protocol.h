/*
 * Responsibility:
 * Public API surface for the motor_protocol component.
 */

#ifndef MOTOR_PROTOCOL_H
#define MOTOR_PROTOCOL_H

#include "motor_types.h"
#include "motor_rx.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t motor_cmd_pp_get(uint8_t target_id, bool ack_requested, motor_pp_index_t index, motor_cmd_t *out);
esp_err_t motor_cmd_pp_set_u8(uint8_t target_id, bool ack_requested, motor_pp_index_t index, uint8_t value, motor_cmd_t *out);

esp_err_t motor_cmd_ic_get(uint8_t target_id, bool ack_requested, motor_ic_index_t index, motor_cmd_t *out);
esp_err_t motor_cmd_ic_set_u16(uint8_t target_id, bool ack_requested, motor_ic_index_t index, uint16_t value, motor_cmd_t *out);

esp_err_t motor_cmd_ie_get(uint8_t target_id, bool ack_requested, motor_ie_index_t index, motor_cmd_t *out);
esp_err_t motor_cmd_ie_set_u16(uint8_t target_id, bool ack_requested, motor_ie_index_t index, uint16_t value, motor_cmd_t *out);

esp_err_t motor_cmd_er_get(uint8_t target_id, bool ack_requested, motor_er_index_t index, motor_cmd_t *out);
esp_err_t motor_cmd_er_clear_all(uint8_t target_id, bool ack_requested, motor_cmd_t *out);

esp_err_t motor_cmd_qe_get(uint8_t target_id, bool ack_requested, motor_qe_index_t index, motor_cmd_t *out);
esp_err_t motor_cmd_qe_set_u16(uint8_t target_id, bool ack_requested, motor_qe_index_t index, uint16_t value, motor_cmd_t *out);

esp_err_t motor_cmd_sy_reboot(uint8_t target_id, motor_cmd_t *out);
esp_err_t motor_cmd_sy_restore_factory_defaults(uint8_t target_id, motor_cmd_t *out);

esp_err_t motor_cmd_mt_get(uint8_t target_id, bool ack_requested, motor_mt_index_t index, motor_cmd_t *out);
esp_err_t motor_cmd_mt_set_u16(uint8_t target_id, bool ack_requested, motor_mt_index_t index, uint16_t value, motor_cmd_t *out);

esp_err_t motor_cmd_il_get(uint8_t target_id, bool ack_requested, motor_il_index_t index, motor_cmd_t *out);
esp_err_t motor_cmd_il_set_u16(uint8_t target_id, bool ack_requested, motor_il_index_t index, uint16_t value, motor_cmd_t *out);

esp_err_t motor_cmd_mo_get(uint8_t target_id, bool ack_requested, motor_cmd_t *out);
esp_err_t motor_cmd_mo_set(uint8_t target_id, bool ack_requested, motor_mo_state_t state, motor_cmd_t *out);

esp_err_t motor_cmd_bg_begin(uint8_t target_id, bool ack_requested, motor_cmd_t *out);
esp_err_t motor_cmd_st_stop(uint8_t target_id, bool ack_requested, motor_cmd_t *out);

esp_err_t motor_cmd_pa_get(uint8_t target_id, bool ack_requested, motor_cmd_t *out);
esp_err_t motor_cmd_pa_set(uint8_t target_id, bool ack_requested, int32_t position, motor_cmd_t *out);

esp_err_t motor_cmd_mp_get(uint8_t target_id, bool ack_requested, motor_mp_index_t index, motor_cmd_t *out);
esp_err_t motor_cmd_mp_set_u16(uint8_t target_id, bool ack_requested, motor_mp_index_t index, uint16_t value, motor_cmd_t *out);

esp_err_t motor_cmd_og_set_origin(uint8_t target_id, bool ack_requested, motor_cmd_t *out);

esp_err_t motor_cmd_lm_get(uint8_t target_id, bool ack_requested, motor_lm_index_t index, motor_cmd_t *out);
esp_err_t motor_cmd_lm_set_i32(uint8_t target_id, bool ack_requested, motor_lm_index_t index, int32_t value, motor_cmd_t *out);

esp_err_t motor_cmd_sd_get(uint8_t target_id, bool ack_requested, motor_cmd_t *out);
esp_err_t motor_cmd_sd_set_u32(uint8_t target_id, bool ack_requested, uint32_t value, motor_cmd_t *out);

esp_err_t motor_cmd_bl_get(uint8_t target_id, bool ack_requested, motor_cmd_t *out);
esp_err_t motor_cmd_bl_set_u32(uint8_t target_id, bool ack_requested, uint32_t value, motor_cmd_t *out);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_PROTOCOL_H */
