/*
 * Responsibility:
 * Semantic description of parsed incoming motor RX frames.
 */

#include "motor_diag_internal.h"

#include <string.h>

#include "motor_codec.h"

#define MOTOR_DIAG_RT_ALARM_D0 0x00U

static void copy_rx_raw(const motor_rx_t *rx, motor_diag_rx_desc_t *out)
{
    out->raw_len = rx->dl;
    if (out->raw_len > sizeof(out->raw)) {
        out->raw_len = sizeof(out->raw);
    }
    memcpy(out->raw, rx->data, out->raw_len);
}

static const motor_diag_param_meta_t *lookup_ack_param(const motor_rx_t *rx, uint16_t *out_index)
{
    motor_pp_index_t pp_index;
    motor_ic_index_t ic_index;
    motor_ie_index_t ie_index;
    motor_qe_index_t qe_index;
    motor_mt_index_t mt_index;
    motor_il_index_t il_index;
    motor_mp_index_t mp_index;
    motor_lm_index_t lm_index;
    uint16_t row_index;

    switch (rx->object) {
        case MOTOR_OBJECT_PP:
            if (motor_rx_ack_pp(rx, &pp_index, NULL)) {
                *out_index = pp_index;
                return motor_diag_lookup_param(MOTOR_OBJECT_PP, pp_index);
            }
            break;
        case MOTOR_OBJECT_IC:
            if (motor_rx_ack_ic(rx, &ic_index, NULL)) {
                *out_index = ic_index;
                return motor_diag_lookup_param(MOTOR_OBJECT_IC, ic_index);
            }
            break;
        case MOTOR_OBJECT_IE:
            if (motor_rx_ack_ie(rx, &ie_index, NULL)) {
                *out_index = ie_index;
                return motor_diag_lookup_param(MOTOR_OBJECT_IE, ie_index);
            }
            break;
        case MOTOR_OBJECT_QE:
            if (motor_rx_ack_qe(rx, &qe_index, NULL)) {
                *out_index = qe_index;
                return motor_diag_lookup_param(MOTOR_OBJECT_QE, qe_index);
            }
            break;
        case MOTOR_OBJECT_MT:
            if (motor_rx_ack_mt(rx, &mt_index, NULL)) {
                *out_index = mt_index;
                return motor_diag_lookup_param(MOTOR_OBJECT_MT, mt_index);
            }
            break;
        case MOTOR_OBJECT_IL:
            if (motor_rx_ack_il(rx, &il_index, NULL)) {
                *out_index = il_index;
                return motor_diag_lookup_param(MOTOR_OBJECT_IL, il_index);
            }
            break;
        case MOTOR_OBJECT_MO:
        case MOTOR_OBJECT_BG:
        case MOTOR_OBJECT_ST:
        case MOTOR_OBJECT_SD:
        case MOTOR_OBJECT_PA:
        case MOTOR_OBJECT_PV:
        case MOTOR_OBJECT_BL:
        case MOTOR_OBJECT_OG:
            *out_index = 0U;
            return motor_diag_lookup_param(rx->object, 0U);
        case MOTOR_OBJECT_MP:
            if (motor_rx_ack_mp(rx, &mp_index, NULL)) {
                *out_index = mp_index;
                return motor_diag_lookup_param(MOTOR_OBJECT_MP, mp_index);
            }
            break;
        case MOTOR_OBJECT_LM:
            if (motor_rx_ack_lm_get(rx, &lm_index, NULL) || motor_rx_ack_lm_set(rx, &lm_index, NULL)) {
                *out_index = lm_index;
                return motor_diag_lookup_param(MOTOR_OBJECT_LM, lm_index);
            }
            break;
        case MOTOR_OBJECT_PT:
            if (motor_rx_ack_pt(rx, &row_index, NULL)) {
                *out_index = row_index;
                return motor_diag_lookup_param(MOTOR_OBJECT_PT, 0U);
            }
            break;
        case MOTOR_OBJECT_DV:
            if (motor_rx_ack_pa_set(rx, NULL)) {
                *out_index = 4U;
                return motor_diag_lookup_param(MOTOR_OBJECT_DV, 4U);
            }
            break;
        default:
            break;
    }

    return NULL;
}

static void describe_ack(const motor_rx_t *rx, motor_diag_rx_desc_t *out)
{
    const motor_diag_param_meta_t *param_meta;
    uint16_t index = 0U;

    out->kind = MOTOR_DIAG_RX_KIND_ACK;
    out->completeness = MOTOR_DIAG_COMPLETENESS_PARTIAL;

    param_meta = lookup_ack_param(rx, &index);
    if (param_meta == NULL) {
        out->note = "unknown semantic mapping";
        copy_rx_raw(rx, out);
        return;
    }

    out->family_symbol = motor_diag_lookup_family(rx->object)->family_symbol;
    out->semantic_name = param_meta->semantic_name;
    if (rx->object == MOTOR_OBJECT_PP ||
        rx->object == MOTOR_OBJECT_IC ||
        rx->object == MOTOR_OBJECT_IE ||
        rx->object == MOTOR_OBJECT_QE ||
        rx->object == MOTOR_OBJECT_MT ||
        rx->object == MOTOR_OBJECT_IL ||
        rx->object == MOTOR_OBJECT_MP ||
        rx->object == MOTOR_OBJECT_LM ||
        rx->object == MOTOR_OBJECT_PT ||
        rx->object == MOTOR_OBJECT_DV) {
        out->has_index = true;
        out->index = index;
    }

    if (!motor_diag_describe_ack_value(rx, out, param_meta)) {
        out->note = "known family but value mapping is incomplete";
        copy_rx_raw(rx, out);
        return;
    }

    if (param_meta->object == MOTOR_OBJECT_IL) {
        out->note = "value semantics are only numerically mapped";
        return;
    }

    if (param_meta->object == MOTOR_OBJECT_MP &&
        param_meta->index == MOTOR_MP_INDEX_CURRENT_QUEUE_LEVEL_OR_RESET_TABLE) {
        out->note = "parameter meaning is only partially verified";
        return;
    }

    if (param_meta->object == MOTOR_OBJECT_QE &&
        param_meta->index == MOTOR_QE_INDEX_BATTERY_VOLTAGE) {
        out->note = "units are not yet verified";
        return;
    }

    if (param_meta->object == MOTOR_OBJECT_MP &&
        param_meta->index == MOTOR_MP_INDEX_PT_MOTION_TIME &&
        out->value.has_raw_number &&
        out->value.raw_number == 0) {
        out->note = "zero may have a special meaning";
        return;
    }

    if (motor_rx_ack_pa_set(rx, NULL)) {
        out->has_semantic_lineage = true;
        out->semantic_object = MOTOR_OBJECT_PA;
        out->semantic_family_symbol = "PA";
        out->semantic_cmd_kind = MOTOR_DIAG_CMD_KIND_SET;
    }

    out->completeness = MOTOR_DIAG_COMPLETENESS_COMPLETE;
}

static void describe_error(const motor_rx_t *rx, motor_diag_rx_desc_t *out)
{
    const motor_diag_error_meta_t *error_meta;
    const char *related_symbol;

    out->kind = MOTOR_DIAG_RX_KIND_ERROR;
    out->family_symbol = "ER";
    out->completeness = MOTOR_DIAG_COMPLETENESS_PARTIAL;
    out->has_error_code = rx->error.valid;
    out->error_code = rx->error.code;

    error_meta = motor_diag_lookup_error(rx->error.code);
    out->error_name = error_meta != NULL ? error_meta->semantic_name : NULL;

    related_symbol = motor_diag_family_symbol_from_base(rx->error.related_base_code);
    if (related_symbol != NULL) {
        out->has_related_family = true;
        out->related_family_symbol = related_symbol;
    }
    if (rx->error.valid) {
        out->has_related_index = true;
        out->related_index = rx->error.related_subindex;
    }

    if (error_meta != NULL && related_symbol != NULL) {
        out->completeness = MOTOR_DIAG_COMPLETENESS_COMPLETE;
    } else {
        out->note = "unknown semantic mapping";
    }

    copy_rx_raw(rx, out);
}

static void describe_er_response(const motor_rx_t *rx, motor_diag_rx_desc_t *out)
{
    const motor_diag_param_meta_t *param_meta;
    const motor_diag_error_meta_t *error_meta;
    motor_er_index_t slot = MOTOR_ER_INDEX_CLEAR_ALL;
    uint8_t error_code = 0U;
    uint8_t related_cw = 0U;
    uint8_t related_subindex = 0U;

    out->kind = MOTOR_DIAG_RX_KIND_ER_RESPONSE;
    out->family_symbol = "ER";
    out->completeness = MOTOR_DIAG_COMPLETENESS_PARTIAL;

    if (motor_rx_er_is_clear_all_response(rx)) {
        out->has_index = true;
        out->index = MOTOR_ER_INDEX_CLEAR_ALL;
        param_meta = motor_diag_lookup_param(MOTOR_OBJECT_ER, MOTOR_ER_INDEX_CLEAR_ALL);
        if (param_meta != NULL) {
            out->semantic_name = param_meta->semantic_name;
        }
        out->completeness = MOTOR_DIAG_COMPLETENESS_COMPLETE;
        return;
    }

    if (!motor_rx_ack_er(rx,
                         &slot,
                         &error_code,
                         &related_cw,
                         &related_subindex)) {
        out->note = "unknown semantic mapping";
        copy_rx_raw(rx, out);
        return;
    }

    out->has_index = true;
    out->index = slot;
    out->has_error_code = true;
    out->error_code = error_code;
    out->has_related_cw = true;
    out->related_cw = related_cw;
    out->has_related_index = true;
    out->related_index = related_subindex;

    param_meta = motor_diag_lookup_param(MOTOR_OBJECT_ER, slot);
    if (param_meta != NULL) {
        out->semantic_name = param_meta->semantic_name;
    }

    error_meta = motor_diag_lookup_error(error_code);
    if (error_meta != NULL) {
        out->error_name = error_meta->semantic_name;
    }

    out->related_family_symbol = motor_diag_family_symbol_from_base(motor_codec_base_code(related_cw));
    out->has_related_family = out->related_family_symbol != NULL;
    out->completeness = MOTOR_DIAG_COMPLETENESS_COMPLETE;
}

static void describe_notification(const motor_rx_t *rx, motor_diag_rx_desc_t *out)
{
    const motor_diag_rt_meta_t *rt_meta;
    uint8_t code;
    bool is_alarm;
    int32_t position;

    out->family_symbol = "RT";
    out->completeness = MOTOR_DIAG_COMPLETENESS_PARTIAL;

    is_alarm = motor_rx_notification_is_alarm(rx);
    if (is_alarm) {
        out->kind = MOTOR_DIAG_RX_KIND_RT_ALARM;
    } else if (motor_rx_notification_is_status(rx)) {
        out->kind = MOTOR_DIAG_RX_KIND_RT_STATUS;
    } else {
        out->kind = MOTOR_DIAG_RX_KIND_RT_UNKNOWN;
        out->note = "unknown notification type";
        copy_rx_raw(rx, out);
        return;
    }

    if (!motor_rx_notification_code(rx, &code)) {
        out->note = "unknown notification type";
        copy_rx_raw(rx, out);
        return;
    }

    rt_meta = motor_diag_lookup_rt(is_alarm, code);
    if (rt_meta == NULL) {
        out->kind = MOTOR_DIAG_RX_KIND_RT_UNKNOWN;
        out->note = "unknown notification type";
        copy_rx_raw(rx, out);
        return;
    }

    out->semantic_name = rt_meta->semantic_name;
    if (rt_meta->value_kind == MOTOR_DIAG_VALUE_KIND_I32 &&
        motor_rx_notification_current_position(rx, &position)) {
        out->value.kind = MOTOR_DIAG_VALUE_KIND_I32;
        out->value.has_value = true;
        out->value.number = position;
        out->value.units = rt_meta->units;
    }

    out->completeness = MOTOR_DIAG_COMPLETENESS_COMPLETE;
}

bool motor_diag_describe_rx(const motor_rx_t *rx, motor_diag_rx_desc_t *out)
{
    const motor_diag_family_meta_t *family_meta;

    if (rx == NULL || out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    family_meta = motor_diag_lookup_family(rx->object);
    out->node_id = rx->producer_id;
    out->object = rx->object;
    out->base_code = rx->base_code;
    out->family_symbol = family_meta != NULL ? family_meta->family_symbol : NULL;

    switch (rx->kind) {
        case MOTOR_RX_KIND_ACK:
            describe_ack(rx, out);
            return true;
        case MOTOR_RX_KIND_ERROR:
            if (motor_rx_er_is_history_response(rx) ||
                motor_rx_er_is_clear_all_response(rx)) {
                describe_er_response(rx, out);
                return true;
            }
            describe_error(rx, out);
            return true;
        case MOTOR_RX_KIND_NOTIFICATION:
            describe_notification(rx, out);
            return true;
        case MOTOR_RX_KIND_UNCLASSIFIED:
        default:
            out->kind = MOTOR_DIAG_RX_KIND_UNCLASSIFIED;
            out->completeness = MOTOR_DIAG_COMPLETENESS_UNKNOWN;
            out->note = "unknown semantic mapping";
            copy_rx_raw(rx, out);
            return true;
    }
}
