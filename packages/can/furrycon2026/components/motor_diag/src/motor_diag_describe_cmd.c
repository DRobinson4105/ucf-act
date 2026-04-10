/*
 * Responsibility:
 * Semantic description of outgoing motor commands.
 */

#include "motor_diag_internal.h"

#include <string.h>

static void copy_raw_value(const motor_cmd_t *cmd, motor_diag_value_desc_t *value)
{
    uint8_t raw_len;

    if (cmd->msg.data_length_code <= 1U) {
        return;
    }

    raw_len = cmd->has_index ? (uint8_t)(cmd->msg.data_length_code - 1U) : cmd->msg.data_length_code;
    if (raw_len > sizeof(value->raw)) {
        raw_len = sizeof(value->raw);
    }

    value->kind = MOTOR_DIAG_VALUE_KIND_RAW_BYTES;
    value->raw_len = raw_len;
    memcpy(value->raw, &cmd->msg.data[cmd->has_index ? 1U : 0U], raw_len);
}

static motor_diag_cmd_kind_t classify_cmd_kind(const motor_cmd_t *cmd)
{
    if (cmd == NULL) {
        return MOTOR_DIAG_CMD_KIND_UNKNOWN;
    }

    switch (cmd->object) {
        case MOTOR_OBJECT_PP:
        case MOTOR_OBJECT_IC:
        case MOTOR_OBJECT_IE:
        case MOTOR_OBJECT_QE:
        case MOTOR_OBJECT_MT:
        case MOTOR_OBJECT_IL:
        case MOTOR_OBJECT_MP:
        case MOTOR_OBJECT_LM:
            return cmd->msg.data_length_code == 1U ? MOTOR_DIAG_CMD_KIND_GET : MOTOR_DIAG_CMD_KIND_SET;
        case MOTOR_OBJECT_MO:
        case MOTOR_OBJECT_SD:
        case MOTOR_OBJECT_BL:
            return cmd->msg.data_length_code == 0U ? MOTOR_DIAG_CMD_KIND_GET : MOTOR_DIAG_CMD_KIND_SET;
        case MOTOR_OBJECT_PA:
            return cmd->msg.data_length_code == 0U ? MOTOR_DIAG_CMD_KIND_GET : MOTOR_DIAG_CMD_KIND_SET;
        case MOTOR_OBJECT_BG:
        case MOTOR_OBJECT_ST:
        case MOTOR_OBJECT_OG:
        case MOTOR_OBJECT_SY:
            return MOTOR_DIAG_CMD_KIND_ACTION;
        case MOTOR_OBJECT_ER:
            return cmd->msg.data_length_code <= 1U ? MOTOR_DIAG_CMD_KIND_GET : MOTOR_DIAG_CMD_KIND_ACTION;
        default:
            return MOTOR_DIAG_CMD_KIND_UNKNOWN;
    }
}

static const motor_diag_param_meta_t *lookup_cmd_param(const motor_cmd_t *cmd)
{
    if (cmd == NULL) {
        return NULL;
    }

    if (cmd->object == MOTOR_OBJECT_MO ||
        cmd->object == MOTOR_OBJECT_BG ||
        cmd->object == MOTOR_OBJECT_ST ||
        cmd->object == MOTOR_OBJECT_SD ||
        cmd->object == MOTOR_OBJECT_PA ||
        cmd->object == MOTOR_OBJECT_BL ||
        cmd->object == MOTOR_OBJECT_OG) {
        return motor_diag_lookup_param(cmd->object, 0U);
    }

    if (!cmd->has_index) {
        return NULL;
    }

    return motor_diag_lookup_param(cmd->object, cmd->index);
}

static void apply_confidence_note(const motor_diag_param_meta_t *param_meta,
                                  const motor_diag_value_desc_t *value,
                                  motor_diag_cmd_desc_t *out)
{
    if (param_meta == NULL || out == NULL) {
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
        value != NULL &&
        value->has_raw_number &&
        value->raw_number == 0) {
        out->note = "zero may have a special meaning";
    }
}

bool motor_diag_describe_cmd(const motor_cmd_t *cmd, motor_diag_cmd_desc_t *out)
{
    const motor_diag_family_meta_t *family_meta;
    const motor_diag_param_meta_t *param_meta;

    if (cmd == NULL || out == NULL) {
        return false;
    }

    memset(out, 0, sizeof(*out));

    family_meta = motor_diag_lookup_family(cmd->object);
    param_meta = lookup_cmd_param(cmd);

    out->node_id = cmd->target_id;
    out->object = cmd->object;
    out->family_symbol = family_meta != NULL ? family_meta->family_symbol : NULL;
    out->has_index = cmd->has_index;
    out->index = cmd->index;
    out->kind = classify_cmd_kind(cmd);
    out->ack_requested = cmd->ack_requested;
    out->semantic_name = param_meta != NULL ? param_meta->semantic_name : NULL;
    out->completeness = family_meta != NULL ? MOTOR_DIAG_COMPLETENESS_PARTIAL : MOTOR_DIAG_COMPLETENESS_UNKNOWN;

    if (family_meta == NULL) {
        out->note = "unknown semantic mapping";
        return true;
    }

    if (param_meta == NULL) {
        out->note = "unknown semantic mapping";
        return true;
    }

    if (out->kind == MOTOR_DIAG_CMD_KIND_SET && !motor_diag_decode_cmd_value(cmd, param_meta, &out->value)) {
        copy_raw_value(cmd, &out->value);
        out->note = "known family but value mapping is incomplete";
        return true;
    }

    apply_confidence_note(param_meta, &out->value, out);
    if (out->note != NULL) {
        return true;
    }

    out->completeness = MOTOR_DIAG_COMPLETENESS_COMPLETE;
    return true;
}
