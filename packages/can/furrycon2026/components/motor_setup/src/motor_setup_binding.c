/*
 * Responsibility:
 * Private motor_setup action-to-motor_exec binding helpers.
 */

#include "motor_setup_binding.h"

#include "esp_err.h"

static motor_exec_submit_result_t invalid_submit_result(void)
{
    return (motor_exec_submit_result_t) {
        .accepted = false,
        .code = MOTOR_EXEC_SUBMIT_INVALID_ARGUMENT,
        .err = ESP_ERR_INVALID_ARG,
    };
}

static bool action_supported_for_brake(const motor_setup_action_t *action)
{
    if (action == NULL) {
        return false;
    }

    switch (action->object) {
        case MOTOR_OBJECT_PP:
        case MOTOR_OBJECT_IC:
        case MOTOR_OBJECT_IE:
        case MOTOR_OBJECT_QE:
        case MOTOR_OBJECT_MT:
        case MOTOR_OBJECT_IL:
        case MOTOR_OBJECT_MO:
        case MOTOR_OBJECT_PA:
        case MOTOR_OBJECT_MP:
        case MOTOR_OBJECT_LM:
        case MOTOR_OBJECT_SD:
        case MOTOR_OBJECT_BL:
            return action->operation == MOTOR_EXEC_OPERATION_GET ||
                   action->operation == MOTOR_EXEC_OPERATION_SET;
        case MOTOR_OBJECT_ER:
            return action->operation == MOTOR_EXEC_OPERATION_GET ||
                   action->operation == MOTOR_EXEC_OPERATION_CLEAR;
        case MOTOR_OBJECT_SY:
            return action->operation == MOTOR_EXEC_OPERATION_ACTION;
        case MOTOR_OBJECT_BG:
        case MOTOR_OBJECT_ST:
        case MOTOR_OBJECT_OG:
            return action->operation == MOTOR_EXEC_OPERATION_ACTION;
        default:
            return false;
    }
}

static motor_exec_submit_result_t submit_brake_action(const motor_setup_action_t *action,
                                                      const motor_exec_submit_opts_t *opts)
{
    if (action == NULL || opts == NULL) {
        return invalid_submit_result();
    }

    switch (action->object) {
        case MOTOR_OBJECT_PP:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_pp_get(action->node_id,
                                               action->ack_requested,
                                               (motor_pp_index_t)action->index,
                                               opts);
            }
            return motor_exec_brake_pp_set_u8(action->node_id,
                                              action->ack_requested,
                                              (motor_pp_index_t)action->index,
                                              action->value.as.u8,
                                              opts);
        case MOTOR_OBJECT_IC:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_ic_get(action->node_id,
                                               action->ack_requested,
                                               (motor_ic_index_t)action->index,
                                               opts);
            }
            return motor_exec_brake_ic_set_u16(action->node_id,
                                               action->ack_requested,
                                               (motor_ic_index_t)action->index,
                                               action->value.kind == MOTOR_SETUP_VALUE_KIND_BOOL ?
                                                   (uint16_t)(action->value.as.boolean ? 1U : 0U) :
                                                   action->value.as.u16,
                                               opts);
        case MOTOR_OBJECT_IE:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_ie_get(action->node_id,
                                               action->ack_requested,
                                               (motor_ie_index_t)action->index,
                                               opts);
            }
            return motor_exec_brake_ie_set_u16(action->node_id,
                                               action->ack_requested,
                                               (motor_ie_index_t)action->index,
                                               action->value.kind == MOTOR_SETUP_VALUE_KIND_BOOL ?
                                                   (uint16_t)(action->value.as.boolean ? 1U : 0U) :
                                                   action->value.as.u16,
                                               opts);
        case MOTOR_OBJECT_ER:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_er_get(action->node_id,
                                               action->ack_requested,
                                               (motor_er_index_t)action->index,
                                               opts);
            }
            return motor_exec_brake_er_clear_all(action->node_id, action->ack_requested, opts);
        case MOTOR_OBJECT_QE:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_qe_get(action->node_id,
                                               action->ack_requested,
                                               (motor_qe_index_t)action->index,
                                               opts);
            }
            return motor_exec_brake_qe_set_u16(action->node_id,
                                               action->ack_requested,
                                               (motor_qe_index_t)action->index,
                                               action->value.as.u16,
                                               opts);
        case MOTOR_OBJECT_SY:
            switch ((motor_sy_op_t)action->index) {
                case MOTOR_SY_OP_REBOOT:
                    return motor_exec_brake_sy_reboot(action->node_id, opts);
                case MOTOR_SY_OP_RESTORE_FACTORY_DEFAULTS:
                    return motor_exec_brake_sy_restore_defaults(action->node_id, opts);
                default:
                    return invalid_submit_result();
            }
        case MOTOR_OBJECT_MT:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_mt_get(action->node_id,
                                               action->ack_requested,
                                               (motor_mt_index_t)action->index,
                                               opts);
            }
            return motor_exec_brake_mt_set_u16(action->node_id,
                                               action->ack_requested,
                                               (motor_mt_index_t)action->index,
                                               action->value.kind == MOTOR_SETUP_VALUE_KIND_ENUM ?
                                                   (uint16_t)action->value.as.enum_value :
                                                   action->value.as.u16,
                                               opts);
        case MOTOR_OBJECT_IL:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_il_get(action->node_id,
                                               action->ack_requested,
                                               (motor_il_index_t)action->index,
                                               opts);
            }
            return motor_exec_brake_il_set_u16(action->node_id,
                                               action->ack_requested,
                                               (motor_il_index_t)action->index,
                                               action->value.kind == MOTOR_SETUP_VALUE_KIND_BOOL ?
                                                   (uint16_t)(action->value.as.boolean ? 1U : 0U) :
                                                   action->value.kind == MOTOR_SETUP_VALUE_KIND_ENUM ?
                                                       (uint16_t)action->value.as.enum_value :
                                                       action->value.as.u16,
                                               opts);
        case MOTOR_OBJECT_MO:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_mo_get(action->node_id, action->ack_requested, opts);
            }
            return motor_exec_brake_mo_set(action->node_id,
                                           action->ack_requested,
                                           (motor_mo_state_t)action->value.as.enum_value,
                                           opts);
        case MOTOR_OBJECT_BG:
            return motor_exec_brake_bg_begin(action->node_id, action->ack_requested, opts);
        case MOTOR_OBJECT_ST:
            return motor_exec_brake_st_stop(action->node_id, action->ack_requested, opts);
        case MOTOR_OBJECT_PA:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_pa_get(action->node_id, action->ack_requested, opts);
            }
            return motor_exec_brake_pa_set(action->node_id,
                                           action->ack_requested,
                                           action->value.as.i32,
                                           opts);
        case MOTOR_OBJECT_MP:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_mp_get(action->node_id,
                                               action->ack_requested,
                                               (motor_mp_index_t)action->index,
                                               opts);
            }
            return motor_exec_brake_mp_set_u16(action->node_id,
                                               action->ack_requested,
                                               (motor_mp_index_t)action->index,
                                               action->value.kind == MOTOR_SETUP_VALUE_KIND_BOOL ?
                                                   (uint16_t)(action->value.as.boolean ? 1U : 0U) :
                                                   action->value.kind == MOTOR_SETUP_VALUE_KIND_ENUM ?
                                                       (uint16_t)action->value.as.enum_value :
                                                       action->value.as.u16,
                                               opts);
        case MOTOR_OBJECT_OG:
            return motor_exec_brake_og_set_origin(action->node_id, action->ack_requested, opts);
        case MOTOR_OBJECT_LM:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_lm_get(action->node_id,
                                               action->ack_requested,
                                               (motor_lm_index_t)action->index,
                                               opts);
            }
            return motor_exec_brake_lm_set_i32(action->node_id,
                                               action->ack_requested,
                                               (motor_lm_index_t)action->index,
                                               action->value.as.i32,
                                               opts);
        case MOTOR_OBJECT_SD:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_sd_get(action->node_id, action->ack_requested, opts);
            }
            return motor_exec_brake_sd_set_u32(action->node_id,
                                               action->ack_requested,
                                               action->value.as.u32,
                                               opts);
        case MOTOR_OBJECT_BL:
            if (action->operation == MOTOR_EXEC_OPERATION_GET) {
                return motor_exec_brake_bl_get(action->node_id, action->ack_requested, opts);
            }
            return motor_exec_brake_bl_set_u32(action->node_id,
                                               action->ack_requested,
                                               action->value.as.u32,
                                               opts);
        default:
            return invalid_submit_result();
    }
}

bool motor_setup_action_binding_supported(const motor_setup_action_t *action)
{
    if (action == NULL) {
        return false;
    }

    switch (action->role) {
        case MOTOR_EXEC_ROLE_BRAKE:
            return action_supported_for_brake(action);
        case MOTOR_EXEC_ROLE_NONE:
        default:
            return false;
    }
}

motor_exec_submit_result_t motor_setup_submit_action(const motor_setup_action_t *action,
                                                     const motor_exec_submit_opts_t *opts)
{
    if (action == NULL || opts == NULL) {
        return invalid_submit_result();
    }

    switch (action->role) {
        case MOTOR_EXEC_ROLE_BRAKE:
            return submit_brake_action(action, opts);
        case MOTOR_EXEC_ROLE_NONE:
        default:
            return invalid_submit_result();
    }
}
