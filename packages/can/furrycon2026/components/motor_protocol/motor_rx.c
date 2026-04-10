/*
 * Responsibility:
 * Incoming frame interpreter for parsing and classifying motor RX frames.
 */

#include "motor_rx.h"

#include <string.h>

#include "motor_codec.h"

#define MOTOR_RX_BASE_PP          0x01U
#define MOTOR_RX_BASE_IC          0x06U
#define MOTOR_RX_BASE_IE          0x07U
#define MOTOR_RX_BASE_ER          0x0FU
#define MOTOR_RX_BASE_MT          0x10U
#define MOTOR_RX_BASE_MO          0x15U
#define MOTOR_RX_BASE_BG          0x16U
#define MOTOR_RX_BASE_ST          0x17U
#define MOTOR_RX_BASE_SD          0x1CU
#define MOTOR_RX_BASE_PA          0x20U
#define MOTOR_RX_BASE_OG          0x21U
#define MOTOR_RX_BASE_MP          0x22U
#define MOTOR_RX_BASE_LM_SET      0x24U
#define MOTOR_RX_BASE_LM_GET      0x2CU
#define MOTOR_RX_BASE_BL          0x2DU
#define MOTOR_RX_BASE_DV          0x2EU
#define MOTOR_RX_BASE_IL          0x34U
#define MOTOR_RX_BASE_QE          0x3DU
#define MOTOR_RX_BASE_NOTIFY      0x5AU
#define MOTOR_RX_MAX_DLC          8U
#define MOTOR_RX_NOTIFY_ALARM_D0  0x00U
#define MOTOR_RX_NOTIFY_POS_OFFSET 2U
#define MOTOR_RX_DV_SUBINDEX_PA_ABSOLUTE 4U

static bool validate_pp_index(uint8_t index)
{
    return index == MOTOR_PP_INDEX_BITRATE ||
           index == MOTOR_PP_INDEX_NODE_ID ||
           index == MOTOR_PP_INDEX_GROUP_ID;
}

static bool validate_ic_index(uint8_t index)
{
    return index == MOTOR_IC_INDEX_AUTO_ENABLE_AFTER_POWERUP ||
           index == MOTOR_IC_INDEX_POSITIVE_MOTOR_DIRECTION ||
           index == MOTOR_IC_INDEX_ENABLE_LOCKDOWN_SYSTEM ||
           index == MOTOR_IC_INDEX_AC_DC_UNITS ||
           index == MOTOR_IC_INDEX_USE_CLOSED_LOOP ||
           index == MOTOR_IC_INDEX_SOFTWARE_LIMIT_ENABLE ||
           index == MOTOR_IC_INDEX_INTERNAL_BRAKE_LOGIC_ENABLE ||
           index == MOTOR_IC_INDEX_USE_INTERNAL_BRAKE;
}

static bool validate_er_history_index(uint8_t index)
{
    return index >= MOTOR_ER_INDEX_HISTORY_1 &&
           index <= MOTOR_ER_INDEX_HISTORY_9;
}

static bool validate_ie_index(uint8_t index)
{
    return index == MOTOR_IE_INDEX_IN1_CHANGE_NOTIFICATION ||
           index == MOTOR_IE_INDEX_IN2_CHANGE_NOTIFICATION ||
           index == MOTOR_IE_INDEX_IN3_CHANGE_NOTIFICATION ||
           index == MOTOR_IE_INDEX_PTP_POSITIONING_FINISH_NOTIFICATION ||
           index == MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION ||
           index == MOTOR_IE_INDEX_FIFO_LOW_WARNING_NOTIFICATION;
}

static bool validate_qe_index(uint8_t index)
{
    return index == MOTOR_QE_INDEX_LINES_PER_REVOLUTION ||
           index == MOTOR_QE_INDEX_MAX_POSITION_ERROR ||
           index == MOTOR_QE_INDEX_BATTERY_VOLTAGE ||
           index == MOTOR_QE_INDEX_COUNTS_PER_REVOLUTION;
}

static bool validate_mt_index(uint8_t index)
{
    return index == MOTOR_MT_INDEX_MICROSTEP ||
           index == MOTOR_MT_INDEX_WORKING_CURRENT ||
           index == MOTOR_MT_INDEX_IDLE_CURRENT_PERCENT ||
           index == MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS ||
           index == MOTOR_MT_INDEX_BRAKE_STATE;
}

static bool validate_il_index(uint8_t index)
{
    return index == MOTOR_IL_INDEX_IN1_TRIGGER_ACTION ||
           index == MOTOR_IL_INDEX_IN2_TRIGGER_ACTION ||
           index == MOTOR_IL_INDEX_IN3_TRIGGER_ACTION ||
           index == MOTOR_IL_INDEX_STALL_BEHAVIOR;
}

static bool validate_lm_index(uint8_t index)
{
    return index == MOTOR_LM_INDEX_MAX_SPEED ||
           index == MOTOR_LM_INDEX_LOWER_WORKING_LIMIT ||
           index == MOTOR_LM_INDEX_UPPER_WORKING_LIMIT ||
           index == MOTOR_LM_INDEX_LOWER_BUMPING_LIMIT ||
           index == MOTOR_LM_INDEX_UPPER_BUMPING_LIMIT ||
           index == MOTOR_LM_INDEX_MAX_POSITION_ERROR ||
           index == MOTOR_LM_INDEX_MAX_ACCEL_DECEL ||
           index == MOTOR_LM_INDEX_RESET_LIMITS ||
           index == MOTOR_LM_INDEX_ENABLE_DISABLE;
}

static bool validate_mp_index(uint8_t index)
{
    return index == MOTOR_MP_INDEX_CURRENT_QUEUE_LEVEL_OR_RESET_TABLE ||
           index == MOTOR_MP_INDEX_FIRST_VALID_ROW ||
           index == MOTOR_MP_INDEX_LAST_VALID_ROW ||
           index == MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE ||
           index == MOTOR_MP_INDEX_PT_MOTION_TIME ||
           index == MOTOR_MP_INDEX_QUEUE_LOW_THRESHOLD ||
           index == MOTOR_MP_INDEX_NEXT_AVAILABLE_WRITING_ROW;
}

static bool validate_mo_state(uint8_t state)
{
    return state == MOTOR_MO_STATE_DISABLE ||
           state == MOTOR_MO_STATE_ENABLE;
}

static bool validate_pa_dv_subindex(uint8_t subindex)
{
    switch (subindex) {
        case MOTOR_RX_DV_SUBINDEX_PA_ABSOLUTE:
            return true;
        default:
            return false;
    }
}

static bool rx_has_kind(const motor_rx_t *rx, motor_rx_kind_t kind)
{
    return rx != NULL && rx->kind == kind;
}

static bool rx_is_base(const motor_rx_t *rx, uint8_t base_code)
{
    return rx != NULL && rx->base_code == base_code;
}

static bool classify_ack_base(uint8_t base_code)
{
    switch (base_code) {
        case MOTOR_RX_BASE_PP:
        case MOTOR_RX_BASE_IC:
        case MOTOR_RX_BASE_IE:
        case MOTOR_RX_BASE_MT:
        case MOTOR_RX_BASE_MO:
        case MOTOR_RX_BASE_BG:
        case MOTOR_RX_BASE_ST:
        case MOTOR_RX_BASE_SD:
        case MOTOR_RX_BASE_PA:
        case MOTOR_RX_BASE_OG:
        case MOTOR_RX_BASE_MP:
        case MOTOR_RX_BASE_LM_SET:
        case MOTOR_RX_BASE_LM_GET:
        case MOTOR_RX_BASE_BL:
        case MOTOR_RX_BASE_DV:
        case MOTOR_RX_BASE_IL:
        case MOTOR_RX_BASE_QE:
            return true;
        default:
            return false;
    }
}

static motor_rx_kind_t classify_kind(uint8_t base_code)
{
    if (base_code == MOTOR_RX_BASE_ER) {
        return MOTOR_RX_KIND_ERROR;
    }

    if (base_code == MOTOR_RX_BASE_NOTIFY) {
        return MOTOR_RX_KIND_NOTIFICATION;
    }

    if (classify_ack_base(base_code)) {
        return MOTOR_RX_KIND_ACK;
    }

    return MOTOR_RX_KIND_UNCLASSIFIED;
}

static void map_base_code(uint8_t base_code,
                          motor_section_t *out_section,
                          motor_object_t *out_object)
{
    motor_section_t section = MOTOR_SECTION_NONE;
    motor_object_t object = MOTOR_OBJECT_NONE;

    switch (base_code) {
        case MOTOR_RX_BASE_PP:
            section = MOTOR_SECTION_PROTOCOL;
            object = MOTOR_OBJECT_PP;
            break;
        case MOTOR_RX_BASE_IC:
            section = MOTOR_SECTION_SYSTEM;
            object = MOTOR_OBJECT_IC;
            break;
        case MOTOR_RX_BASE_IE:
            section = MOTOR_SECTION_SYSTEM;
            object = MOTOR_OBJECT_IE;
            break;
        case MOTOR_RX_BASE_ER:
            section = MOTOR_SECTION_SYSTEM;
            object = MOTOR_OBJECT_ER;
            break;
        case MOTOR_RX_BASE_QE:
            section = MOTOR_SECTION_SYSTEM;
            object = MOTOR_OBJECT_QE;
            break;
        case MOTOR_RX_BASE_MT:
            section = MOTOR_SECTION_MOTOR_DRIVER;
            object = MOTOR_OBJECT_MT;
            break;
        case MOTOR_RX_BASE_IL:
            section = MOTOR_SECTION_IO;
            object = MOTOR_OBJECT_IL;
            break;
        case MOTOR_RX_BASE_MO:
            section = MOTOR_SECTION_MOTOR_DRIVER;
            object = MOTOR_OBJECT_MO;
            break;
        case MOTOR_RX_BASE_BG:
            section = MOTOR_SECTION_MOTION_CONTROL;
            object = MOTOR_OBJECT_BG;
            break;
        case MOTOR_RX_BASE_ST:
            section = MOTOR_SECTION_MOTION_CONTROL;
            object = MOTOR_OBJECT_ST;
            break;
        case MOTOR_RX_BASE_SD:
            section = MOTOR_SECTION_MOTION_CONTROL;
            object = MOTOR_OBJECT_SD;
            break;
        case MOTOR_RX_BASE_PA:
            section = MOTOR_SECTION_MOTION_CONTROL;
            object = MOTOR_OBJECT_PA;
            break;
        case MOTOR_RX_BASE_OG:
            section = MOTOR_SECTION_MOTION_CONTROL;
            object = MOTOR_OBJECT_OG;
            break;
        case MOTOR_RX_BASE_MP:
            section = MOTOR_SECTION_INTERPOLATED_MOTION;
            object = MOTOR_OBJECT_MP;
            break;
        case MOTOR_RX_BASE_LM_SET:
        case MOTOR_RX_BASE_LM_GET:
            section = MOTOR_SECTION_MOTION_CONTROL;
            object = MOTOR_OBJECT_LM;
            break;
        case MOTOR_RX_BASE_BL:
            section = MOTOR_SECTION_MOTION_CONTROL;
            object = MOTOR_OBJECT_BL;
            break;
        case MOTOR_RX_BASE_DV:
            section = MOTOR_SECTION_MOTION_CONTROL;
            object = MOTOR_OBJECT_DV;
            break;
        case MOTOR_RX_BASE_NOTIFY:
            section = MOTOR_SECTION_NOTIFICATION;
            object = MOTOR_OBJECT_RT;
            break;
        default:
            break;
    }

    if (out_section != NULL) {
        *out_section = section;
    }

    if (out_object != NULL) {
        *out_object = object;
    }
}

static esp_err_t validate_frame(const twai_message_t *msg, motor_rx_t *out)
{
    if (msg == NULL || out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!msg->extd || msg->rtr || msg->data_length_code > MOTOR_RX_MAX_DLC) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static void populate_error_overlay(motor_rx_t *out)
{
    if (out->kind != MOTOR_RX_KIND_ERROR || out->dl < 4U) {
        return;
    }

    out->error.valid = true;
    out->error.slot = out->data[0];
    out->error.code = out->data[1];
    out->error.related_cw = out->data[2];
    out->error.related_base_code = motor_codec_base_code(out->data[2]);
    out->error.related_subindex = out->data[3];
}

static void populate_notification_overlay(motor_rx_t *out)
{
    if (out->kind != MOTOR_RX_KIND_NOTIFICATION) {
        return;
    }

    if (out->dl >= 1U) {
        out->notification.has_d0 = true;
        out->notification.d0 = out->data[0];
    }

    if (out->dl >= 2U) {
        out->notification.has_d1 = true;
        out->notification.d1 = out->data[1];
    }

    /*
     * Current-position extraction is intended for the completion-style RT
     * notification layout. Treat it as provisional for other status payload
     * shapes until the manual confirms a broader invariant.
     */
    if (out->dl >= 6U && out->notification.has_d0 &&
        out->notification.d0 != MOTOR_RX_NOTIFY_ALARM_D0) {
        out->notification.has_current_position = true;
        out->notification.current_position =
            motor_codec_unpack_i32_le(&out->data[MOTOR_RX_NOTIFY_POS_OFFSET]);
    }
}

static bool ack_shape(const motor_rx_t *rx, uint8_t expected_base, uint8_t expected_dlc)
{
    return motor_rx_is_ack(rx) &&
           rx_is_base(rx, expected_base) &&
           rx->dl == expected_dlc;
}

static bool ack_shape_any_base(const motor_rx_t *rx,
                               uint8_t expected_base_a,
                               uint8_t expected_base_b,
                               uint8_t expected_dlc)
{
    return motor_rx_is_ack(rx) &&
           (rx_is_base(rx, expected_base_a) || rx_is_base(rx, expected_base_b)) &&
           rx->dl == expected_dlc;
}

static bool ack_shape_bg(const motor_rx_t *rx)
{
    return motor_rx_is_ack(rx) &&
           rx_is_base(rx, MOTOR_RX_BASE_BG) &&
           (rx->dl == 0U || rx->dl == 4U);
}

static bool er_is_clear_all_zero_payload(const motor_rx_t *rx)
{
    return rx != NULL &&
           motor_rx_is_error(rx) &&
           rx_is_base(rx, MOTOR_RX_BASE_ER) &&
           rx->dl == 6U &&
           rx->data[0] == MOTOR_ER_INDEX_CLEAR_ALL &&
           rx->data[1] == 0U &&
           rx->data[2] == 0U &&
           rx->data[3] == 0U &&
           rx->data[4] == 0U &&
           rx->data[5] == 0U;
}

static bool cmd_requires_strict_producer_match(const motor_cmd_t *cmd)
{
    if (cmd == NULL) {
        return false;
    }

    /*
     * Use the original TX command target metadata as the authority for
     * producer strictness. Today target_id == 0 is the only explicit
     * non-direct/global representation carried by motor_cmd_t.
     */
    return cmd->target_id != 0U;
}

static bool ack_target_matches(const motor_rx_t *rx, const motor_cmd_t *cmd)
{
    if (rx == NULL || cmd == NULL) {
        return false;
    }

    if (cmd_requires_strict_producer_match(cmd) &&
        rx->producer_id != cmd->target_id) {
        return false;
    }

    return true;
}

static bool expected_response_base(const motor_cmd_t *cmd, uint8_t *out_base)
{
    uint8_t response_base;

    if (cmd == NULL || out_base == NULL) {
        return false;
    }

    switch (cmd->object) {
        case MOTOR_OBJECT_PP:
            response_base = MOTOR_RX_BASE_PP;
            break;
        case MOTOR_OBJECT_IC:
            response_base = MOTOR_RX_BASE_IC;
            break;
        case MOTOR_OBJECT_IE:
            response_base = MOTOR_RX_BASE_IE;
            break;
        case MOTOR_OBJECT_ER:
            response_base = MOTOR_RX_BASE_ER;
            break;
        case MOTOR_OBJECT_QE:
            response_base = MOTOR_RX_BASE_QE;
            break;
        case MOTOR_OBJECT_MT:
            response_base = MOTOR_RX_BASE_MT;
            break;
        case MOTOR_OBJECT_IL:
            response_base = MOTOR_RX_BASE_IL;
            break;
        case MOTOR_OBJECT_MO:
            response_base = MOTOR_RX_BASE_MO;
            break;
        case MOTOR_OBJECT_BG:
            response_base = MOTOR_RX_BASE_BG;
            break;
        case MOTOR_OBJECT_ST:
            response_base = MOTOR_RX_BASE_ST;
            break;
        case MOTOR_OBJECT_SD:
            response_base = MOTOR_RX_BASE_SD;
            break;
        case MOTOR_OBJECT_PA:
            /*
             * Current asymmetric PA matching relies on the present motor_cmd
             * builder payload contract: GET uses DLC 0 and SET uses DLC 4.
             * Revisit this rule if motor_cmd payload shapes change.
             */
            response_base = (cmd->msg.data_length_code == 0U) ? MOTOR_RX_BASE_PA : MOTOR_RX_BASE_DV;
            break;
        case MOTOR_OBJECT_MP:
            response_base = MOTOR_RX_BASE_MP;
            break;
        case MOTOR_OBJECT_OG:
            response_base = MOTOR_RX_BASE_OG;
            break;
        case MOTOR_OBJECT_LM:
            /*
             * Current asymmetric LM matching relies on the present motor_cmd
             * builder payload contract: GET uses DLC 1 and SET uses DLC 5.
             * Revisit this rule if motor_cmd payload shapes change.
             */
            response_base = (cmd->msg.data_length_code == 1U) ? MOTOR_RX_BASE_LM_GET : MOTOR_RX_BASE_LM_SET;
            break;
        case MOTOR_OBJECT_BL:
            response_base = MOTOR_RX_BASE_BL;
            break;
        default:
            return false;
    }

    *out_base = response_base;
    return true;
}

esp_err_t motor_rx_parse(const twai_message_t *msg, motor_rx_t *out)
{
    esp_err_t err = validate_frame(msg, out);

    if (err != ESP_OK) {
        return err;
    }

    memset(out, 0, sizeof(*out));
    out->ext_id_raw = msg->identifier;
    out->producer_id = motor_codec_decode_id(msg->identifier);
    out->cw_raw = motor_codec_decode_cw(msg->identifier);
    out->base_code = motor_codec_base_code(out->cw_raw);
    out->dl = msg->data_length_code;
    out->kind = classify_kind(out->base_code);
    map_base_code(out->base_code, &out->section, &out->object);
    memcpy(out->data, msg->data, out->dl);

    populate_error_overlay(out);
    populate_notification_overlay(out);

    return ESP_OK;
}

bool motor_rx_is_ack(const motor_rx_t *rx)
{
    return rx_has_kind(rx, MOTOR_RX_KIND_ACK);
}

bool motor_rx_is_error(const motor_rx_t *rx)
{
    return rx_has_kind(rx, MOTOR_RX_KIND_ERROR);
}

bool motor_rx_is_notification(const motor_rx_t *rx)
{
    return rx_has_kind(rx, MOTOR_RX_KIND_NOTIFICATION);
}

bool motor_rx_is_unclassified(const motor_rx_t *rx)
{
    return rx_has_kind(rx, MOTOR_RX_KIND_UNCLASSIFIED);
}

bool motor_rx_error_slot(const motor_rx_t *rx, uint8_t *out_slot)
{
    if (!motor_rx_is_error(rx) || !rx->error.valid || out_slot == NULL) {
        return false;
    }

    *out_slot = rx->error.slot;
    return true;
}

bool motor_rx_error_code(const motor_rx_t *rx, uint8_t *out_code)
{
    if (!motor_rx_is_error(rx) || !rx->error.valid || out_code == NULL) {
        return false;
    }

    *out_code = rx->error.code;
    return true;
}

bool motor_rx_error_related_cw(const motor_rx_t *rx, uint8_t *out_related_cw)
{
    if (!motor_rx_is_error(rx) || !rx->error.valid || out_related_cw == NULL) {
        return false;
    }

    *out_related_cw = rx->error.related_cw;
    return true;
}

bool motor_rx_error_related_base_code(const motor_rx_t *rx, uint8_t *out_related_base_code)
{
    if (!motor_rx_is_error(rx) || !rx->error.valid || out_related_base_code == NULL) {
        return false;
    }

    *out_related_base_code = rx->error.related_base_code;
    return true;
}

bool motor_rx_error_related_subindex(const motor_rx_t *rx, uint8_t *out_related_subindex)
{
    if (!motor_rx_is_error(rx) || !rx->error.valid || out_related_subindex == NULL) {
        return false;
    }

    *out_related_subindex = rx->error.related_subindex;
    return true;
}

bool motor_rx_er_is_thrown_error(const motor_rx_t *rx)
{
    if (er_is_clear_all_zero_payload(rx)) {
        return false;
    }

    return motor_rx_is_error(rx) &&
           rx_is_base(rx, MOTOR_RX_BASE_ER) &&
           rx->dl >= 1U &&
           rx->data[0] == MOTOR_ER_INDEX_CLEAR_ALL;
}

bool motor_rx_er_is_clear_all_response(const motor_rx_t *rx)
{
    return er_is_clear_all_zero_payload(rx);
}

bool motor_rx_er_is_history_response(const motor_rx_t *rx)
{
    return motor_rx_is_error(rx) &&
           rx_is_base(rx, MOTOR_RX_BASE_ER) &&
           rx->dl == 6U &&
           validate_er_history_index(rx->data[0]);
}

bool motor_rx_notification_d0(const motor_rx_t *rx, uint8_t *out_d0)
{
    if (!motor_rx_is_notification(rx) || !rx->notification.has_d0 || out_d0 == NULL) {
        return false;
    }

    *out_d0 = rx->notification.d0;
    return true;
}

bool motor_rx_notification_d1(const motor_rx_t *rx, uint8_t *out_d1)
{
    if (!motor_rx_is_notification(rx) || !rx->notification.has_d1 || out_d1 == NULL) {
        return false;
    }

    *out_d1 = rx->notification.d1;
    return true;
}

bool motor_rx_notification_is_alarm(const motor_rx_t *rx)
{
    return motor_rx_is_notification(rx) &&
           rx->notification.has_d0 &&
           rx->notification.has_d1 &&
           rx->notification.d0 == MOTOR_RX_NOTIFY_ALARM_D0;
}

bool motor_rx_notification_is_status(const motor_rx_t *rx)
{
    return motor_rx_is_notification(rx) &&
           rx->notification.has_d0 &&
           rx->notification.d0 != MOTOR_RX_NOTIFY_ALARM_D0;
}

bool motor_rx_notification_code(const motor_rx_t *rx, uint8_t *out_code)
{
    if (out_code == NULL) {
        return false;
    }

    if (motor_rx_notification_is_alarm(rx)) {
        *out_code = rx->notification.d1;
        return true;
    }

    if (motor_rx_notification_is_status(rx)) {
        *out_code = rx->notification.d0;
        return true;
    }

    return false;
}

bool motor_rx_notification_current_position(const motor_rx_t *rx, int32_t *out_position)
{
    if (!motor_rx_notification_is_status(rx) ||
        !rx->notification.has_current_position ||
        out_position == NULL) {
        return false;
    }

    *out_position = rx->notification.current_position;
    return true;
}

bool motor_rx_ack_pp(const motor_rx_t *rx, motor_pp_index_t *out_index, uint8_t *out_value)
{
    if (!ack_shape(rx, MOTOR_RX_BASE_PP, 2U) || !validate_pp_index(rx->data[0])) {
        return false;
    }

    if (out_index != NULL) {
        *out_index = (motor_pp_index_t)rx->data[0];
    }

    if (out_value != NULL) {
        *out_value = rx->data[1];
    }

    return true;
}

bool motor_rx_ack_ic(const motor_rx_t *rx, motor_ic_index_t *out_index, uint16_t *out_value)
{
    if (!ack_shape(rx, MOTOR_RX_BASE_IC, 3U) || !validate_ic_index(rx->data[0])) {
        return false;
    }

    if (out_index != NULL) {
        *out_index = (motor_ic_index_t)rx->data[0];
    }

    if (out_value != NULL) {
        *out_value = motor_codec_unpack_u16_le(&rx->data[1]);
    }

    return true;
}

bool motor_rx_ack_ie(const motor_rx_t *rx, motor_ie_index_t *out_index, uint16_t *out_value)
{
    if (!ack_shape(rx, MOTOR_RX_BASE_IE, 3U) || !validate_ie_index(rx->data[0])) {
        return false;
    }

    if (out_index != NULL) {
        *out_index = (motor_ie_index_t)rx->data[0];
    }

    if (out_value != NULL) {
        *out_value = motor_codec_unpack_u16_le(&rx->data[1]);
    }

    return true;
}

bool motor_rx_ack_er(const motor_rx_t *rx,
                     motor_er_index_t *out_slot,
                     uint8_t *out_error_code,
                     uint8_t *out_related_cw,
                     uint8_t *out_related_subindex)
{
    /*
     * ER queried-history responses are structurally ERROR-kind frames, but
     * this helper is the family-specific accessor for the valid command-
     * response payload shape only.
     */
    if (!motor_rx_er_is_history_response(rx)) {
        return false;
    }

    if (out_slot != NULL) {
        *out_slot = (motor_er_index_t)rx->data[0];
    }

    if (out_error_code != NULL) {
        *out_error_code = rx->data[1];
    }

    if (out_related_cw != NULL) {
        *out_related_cw = rx->data[2];
    }

    if (out_related_subindex != NULL) {
        *out_related_subindex = rx->data[3];
    }

    return true;
}

bool motor_rx_ack_qe(const motor_rx_t *rx, motor_qe_index_t *out_index, uint16_t *out_value)
{
    if (!ack_shape(rx, MOTOR_RX_BASE_QE, 3U) || !validate_qe_index(rx->data[0])) {
        return false;
    }

    if (out_index != NULL) {
        *out_index = (motor_qe_index_t)rx->data[0];
    }

    if (out_value != NULL) {
        *out_value = motor_codec_unpack_u16_le(&rx->data[1]);
    }

    return true;
}

bool motor_rx_ack_mt(const motor_rx_t *rx, motor_mt_index_t *out_index, uint16_t *out_value)
{
    if (!ack_shape(rx, MOTOR_RX_BASE_MT, 3U) || !validate_mt_index(rx->data[0])) {
        return false;
    }

    if (out_index != NULL) {
        *out_index = (motor_mt_index_t)rx->data[0];
    }

    if (out_value != NULL) {
        *out_value = motor_codec_unpack_u16_le(&rx->data[1]);
    }

    return true;
}

bool motor_rx_ack_il(const motor_rx_t *rx, motor_il_index_t *out_index, uint16_t *out_value)
{
    if (!ack_shape(rx, MOTOR_RX_BASE_IL, 3U) || !validate_il_index(rx->data[0])) {
        return false;
    }

    if (out_index != NULL) {
        *out_index = (motor_il_index_t)rx->data[0];
    }

    if (out_value != NULL) {
        *out_value = motor_codec_unpack_u16_le(&rx->data[1]);
    }

    return true;
}

bool motor_rx_ack_mo(const motor_rx_t *rx, uint8_t *out_state)
{
    if (!ack_shape(rx, MOTOR_RX_BASE_MO, 1U) || !validate_mo_state(rx->data[0])) {
        return false;
    }

    if (out_state != NULL) {
        *out_state = rx->data[0];
    }

    return true;
}

bool motor_rx_ack_bg(const motor_rx_t *rx)
{
    return ack_shape_bg(rx);
}

bool motor_rx_ack_st(const motor_rx_t *rx)
{
    return ack_shape(rx, MOTOR_RX_BASE_ST, 0U);
}

bool motor_rx_ack_pa_get(const motor_rx_t *rx, int32_t *out_position)
{
    if (!ack_shape(rx, MOTOR_RX_BASE_PA, 4U)) {
        return false;
    }

    if (out_position != NULL) {
        *out_position = motor_codec_unpack_i32_le(&rx->data[0]);
    }

    return true;
}

bool motor_rx_ack_pa_set(const motor_rx_t *rx, int32_t *out_position)
{
    /*
     * PA SET currently uses the DV response family with a fixed subindex for
     * absolute-position payloads. Extend validate_pa_dv_subindex() if future
     * DV-based command responses are added.
     */
    if (!ack_shape(rx, MOTOR_RX_BASE_DV, 5U) || !validate_pa_dv_subindex(rx->data[0])) {
        return false;
    }

    if (out_position != NULL) {
        *out_position = motor_codec_unpack_i32_le(&rx->data[1]);
    }

    return true;
}

bool motor_rx_ack_mp(const motor_rx_t *rx, motor_mp_index_t *out_index, uint16_t *out_value)
{
    if (!ack_shape(rx, MOTOR_RX_BASE_MP, 3U) || !validate_mp_index(rx->data[0])) {
        return false;
    }

    if (out_index != NULL) {
        *out_index = (motor_mp_index_t)rx->data[0];
    }

    if (out_value != NULL) {
        *out_value = motor_codec_unpack_u16_le(&rx->data[1]);
    }

    return true;
}

bool motor_rx_ack_og(const motor_rx_t *rx)
{
    return ack_shape(rx, MOTOR_RX_BASE_OG, 0U);
}

bool motor_rx_ack_lm_get(const motor_rx_t *rx, motor_lm_index_t *out_index, int32_t *out_value)
{
    if (!ack_shape(rx, MOTOR_RX_BASE_LM_GET, 5U) || !validate_lm_index(rx->data[0])) {
        return false;
    }

    if (out_index != NULL) {
        *out_index = (motor_lm_index_t)rx->data[0];
    }

    if (out_value != NULL) {
        *out_value = motor_codec_unpack_i32_le(&rx->data[1]);
    }

    return true;
}

bool motor_rx_ack_lm_set(const motor_rx_t *rx, motor_lm_index_t *out_index, int32_t *out_value)
{
    /*
     * Some targets reply to ACK-requested LM SET commands with the general LM
     * response base (0x2C) rather than the dedicated LM-SET base (0x24).
     * Accept both shapes so command matching follows on-wire behavior.
     */
    if (!ack_shape_any_base(rx, MOTOR_RX_BASE_LM_SET, MOTOR_RX_BASE_LM_GET, 5U) ||
        !validate_lm_index(rx->data[0])) {
        return false;
    }

    if (out_index != NULL) {
        *out_index = (motor_lm_index_t)rx->data[0];
    }

    if (out_value != NULL) {
        *out_value = motor_codec_unpack_i32_le(&rx->data[1]);
    }

    return true;
}

bool motor_rx_ack_sd(const motor_rx_t *rx, uint32_t *out_value)
{
    if (!ack_shape(rx, MOTOR_RX_BASE_SD, 4U)) {
        return false;
    }

    if (out_value != NULL) {
        *out_value = motor_codec_unpack_u32_le(&rx->data[0]);
    }

    return true;
}

bool motor_rx_ack_bl(const motor_rx_t *rx, uint32_t *out_value)
{
    if (!ack_shape(rx, MOTOR_RX_BASE_BL, 4U)) {
        return false;
    }

    if (out_value != NULL) {
        *out_value = motor_codec_unpack_u32_le(&rx->data[0]);
    }

    return true;
}

bool motor_rx_matches_cmd(const motor_rx_t *rx, const motor_cmd_t *cmd)
{
    uint8_t expected_base;
    motor_pp_index_t pp_index;
    motor_ic_index_t ic_index;
    motor_ie_index_t ie_index;
    motor_er_index_t er_slot;
    motor_qe_index_t qe_index;
    motor_mt_index_t mt_index;
    motor_il_index_t il_index;
    motor_mp_index_t mp_index;
    motor_lm_index_t lm_index;
    uint8_t u8_value;
    uint16_t u16_value;
    int32_t i32_value;
    uint32_t u32_value;

    if (rx == NULL || cmd == NULL || !ack_target_matches(rx, cmd)) {
        return false;
    }

    if (cmd->object != MOTOR_OBJECT_LM) {
        if (!expected_response_base(cmd, &expected_base) || rx->base_code != expected_base) {
            return false;
        }
    }

    switch (cmd->object) {
        case MOTOR_OBJECT_PP:
            if (!motor_rx_ack_pp(rx, &pp_index, &u8_value) || pp_index != (motor_pp_index_t)cmd->index) {
                return false;
            }

            return cmd->msg.data_length_code != 2U || u8_value == cmd->msg.data[1];
        case MOTOR_OBJECT_IC:
            if (!motor_rx_ack_ic(rx, &ic_index, &u16_value) || ic_index != (motor_ic_index_t)cmd->index) {
                return false;
            }

            return cmd->msg.data_length_code != 3U || u16_value == motor_codec_unpack_u16_le(&cmd->msg.data[1]);
        case MOTOR_OBJECT_IE:
            if (!motor_rx_ack_ie(rx, &ie_index, &u16_value) || ie_index != (motor_ie_index_t)cmd->index) {
                return false;
            }

            return cmd->msg.data_length_code != 3U || u16_value == motor_codec_unpack_u16_le(&cmd->msg.data[1]);
        case MOTOR_OBJECT_ER:
            if (cmd->index == MOTOR_ER_INDEX_CLEAR_ALL) {
                return cmd->msg.data_length_code == 2U &&
                       motor_rx_er_is_clear_all_response(rx);
            }

            return motor_rx_ack_er(rx, &er_slot, NULL, NULL, NULL) &&
                   er_slot == (motor_er_index_t)cmd->index;
        case MOTOR_OBJECT_QE:
            if (!motor_rx_ack_qe(rx, &qe_index, &u16_value) || qe_index != (motor_qe_index_t)cmd->index) {
                return false;
            }

            return cmd->msg.data_length_code != 3U || u16_value == motor_codec_unpack_u16_le(&cmd->msg.data[1]);
        case MOTOR_OBJECT_MT:
            if (!motor_rx_ack_mt(rx, &mt_index, &u16_value) || mt_index != (motor_mt_index_t)cmd->index) {
                return false;
            }

            return cmd->msg.data_length_code != 3U || u16_value == motor_codec_unpack_u16_le(&cmd->msg.data[1]);
        case MOTOR_OBJECT_IL:
            if (!motor_rx_ack_il(rx, &il_index, &u16_value) || il_index != (motor_il_index_t)cmd->index) {
                return false;
            }

            return cmd->msg.data_length_code != 3U || u16_value == motor_codec_unpack_u16_le(&cmd->msg.data[1]);
        case MOTOR_OBJECT_MO:
            if (!motor_rx_ack_mo(rx, &u8_value)) {
                return false;
            }

            return cmd->msg.data_length_code != 1U || u8_value == cmd->msg.data[0];
        case MOTOR_OBJECT_BG:
            return motor_rx_ack_bg(rx);
        case MOTOR_OBJECT_ST:
            return motor_rx_ack_st(rx);
        case MOTOR_OBJECT_SD:
            return motor_rx_ack_sd(rx, &u32_value) &&
                   (cmd->msg.data_length_code != 4U || u32_value == motor_codec_unpack_u32_le(&cmd->msg.data[0]));
        case MOTOR_OBJECT_PA:
            if (cmd->msg.data_length_code == 0U) {
                return motor_rx_ack_pa_get(rx, NULL);
            }

            return motor_rx_ack_pa_set(rx, &i32_value) &&
                   i32_value == motor_codec_unpack_i32_le(&cmd->msg.data[0]);
        case MOTOR_OBJECT_MP:
            if (!motor_rx_ack_mp(rx, &mp_index, &u16_value) || mp_index != (motor_mp_index_t)cmd->index) {
                return false;
            }

            return cmd->msg.data_length_code != 3U || u16_value == motor_codec_unpack_u16_le(&cmd->msg.data[1]);
        case MOTOR_OBJECT_OG:
            return motor_rx_ack_og(rx);
        case MOTOR_OBJECT_LM:
            if (cmd->msg.data_length_code == 1U) {
                if (rx->base_code != MOTOR_RX_BASE_LM_GET) {
                    return false;
                }

                return motor_rx_ack_lm_get(rx, &lm_index, NULL) &&
                       lm_index == (motor_lm_index_t)cmd->index;
            }

            if (rx->base_code != MOTOR_RX_BASE_LM_SET &&
                rx->base_code != MOTOR_RX_BASE_LM_GET) {
                return false;
            }

            return motor_rx_ack_lm_set(rx, &lm_index, &i32_value) &&
                   lm_index == (motor_lm_index_t)cmd->index &&
                   i32_value == motor_codec_unpack_i32_le(&cmd->msg.data[1]);
        case MOTOR_OBJECT_BL:
            return motor_rx_ack_bl(rx, &u32_value) &&
                   (cmd->msg.data_length_code != 4U || u32_value == motor_codec_unpack_u32_le(&cmd->msg.data[0]));
        default:
            return false;
    }
}
