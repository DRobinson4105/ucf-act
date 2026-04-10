/*
 * Responsibility:
 * Structural API for parsing and inspecting incoming motor protocol frames.
 */

#ifndef MOTOR_RX_H
#define MOTOR_RX_H

#include "esp_err.h"
#include "motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t motor_rx_parse(const twai_message_t *msg, motor_rx_t *out);

bool motor_rx_is_ack(const motor_rx_t *rx);
bool motor_rx_is_error(const motor_rx_t *rx);
bool motor_rx_is_notification(const motor_rx_t *rx);
bool motor_rx_is_unclassified(const motor_rx_t *rx);

bool motor_rx_matches_cmd(const motor_rx_t *rx, const motor_cmd_t *cmd);

bool motor_rx_error_slot(const motor_rx_t *rx, uint8_t *out_slot);
bool motor_rx_error_code(const motor_rx_t *rx, uint8_t *out_code);
bool motor_rx_error_related_cw(const motor_rx_t *rx, uint8_t *out_related_cw);
bool motor_rx_error_related_base_code(const motor_rx_t *rx, uint8_t *out_related_base_code);
bool motor_rx_error_related_subindex(const motor_rx_t *rx, uint8_t *out_related_subindex);
bool motor_rx_er_is_clear_all_response(const motor_rx_t *rx);
bool motor_rx_er_is_thrown_error(const motor_rx_t *rx);
bool motor_rx_er_is_history_response(const motor_rx_t *rx);

bool motor_rx_notification_d0(const motor_rx_t *rx, uint8_t *out_d0);
bool motor_rx_notification_d1(const motor_rx_t *rx, uint8_t *out_d1);
bool motor_rx_notification_is_alarm(const motor_rx_t *rx);
bool motor_rx_notification_is_status(const motor_rx_t *rx);
bool motor_rx_notification_code(const motor_rx_t *rx, uint8_t *out_code);
bool motor_rx_notification_current_position(const motor_rx_t *rx, int32_t *out_position);

bool motor_rx_ack_pp(const motor_rx_t *rx, motor_pp_index_t *out_index, uint8_t *out_value);
bool motor_rx_ack_ic(const motor_rx_t *rx, motor_ic_index_t *out_index, uint16_t *out_value);
bool motor_rx_ack_ie(const motor_rx_t *rx, motor_ie_index_t *out_index, uint16_t *out_value);
/*
 * ER command responses are structurally represented as MOTOR_RX_KIND_ERROR
 * frames. This helper is the family-specific accessor for valid queried ER
 * history-response payloads (d0 in [10..18]) carried on that ERROR-kind frame
 * shape.
 */
bool motor_rx_ack_er(const motor_rx_t *rx,
                     motor_er_index_t *out_slot,
                     uint8_t *out_error_code,
                     uint8_t *out_related_cw,
                     uint8_t *out_related_subindex);
bool motor_rx_ack_qe(const motor_rx_t *rx, motor_qe_index_t *out_index, uint16_t *out_value);
bool motor_rx_ack_mt(const motor_rx_t *rx, motor_mt_index_t *out_index, uint16_t *out_value);
bool motor_rx_ack_il(const motor_rx_t *rx, motor_il_index_t *out_index, uint16_t *out_value);
bool motor_rx_ack_mo(const motor_rx_t *rx, uint8_t *out_state);
bool motor_rx_ack_bg(const motor_rx_t *rx);
bool motor_rx_ack_st(const motor_rx_t *rx);
bool motor_rx_ack_pa_get(const motor_rx_t *rx, int32_t *out_position);
bool motor_rx_ack_pa_set(const motor_rx_t *rx, int32_t *out_position);
bool motor_rx_ack_mp(const motor_rx_t *rx, motor_mp_index_t *out_index, uint16_t *out_value);
bool motor_rx_ack_pv(const motor_rx_t *rx, uint16_t *out_row_index);
bool motor_rx_ack_pt(const motor_rx_t *rx, uint16_t *out_row_index, int32_t *out_position);
bool motor_rx_ack_og(const motor_rx_t *rx);
bool motor_rx_ack_lm_get(const motor_rx_t *rx, motor_lm_index_t *out_index, int32_t *out_value);
bool motor_rx_ack_lm_set(const motor_rx_t *rx, motor_lm_index_t *out_index, int32_t *out_value);
bool motor_rx_ack_sd(const motor_rx_t *rx, uint32_t *out_value);
bool motor_rx_ack_bl(const motor_rx_t *rx, uint32_t *out_value);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_RX_H */
