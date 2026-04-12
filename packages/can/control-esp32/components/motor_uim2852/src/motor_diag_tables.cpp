/*
 * Responsibility:
 * Static semantic metadata tables for motor_diag.
 */

#include "motor_diag_internal.h"

#include <stddef.h>

#include "motor_codec.h"

#define ARRAY_LEN(a) (sizeof(a) / sizeof((a)[0]))

#define MOTOR_DIAG_RX_BASE_PP      0x01U
#define MOTOR_DIAG_RX_BASE_IC      0x06U
#define MOTOR_DIAG_RX_BASE_IE      0x07U
#define MOTOR_DIAG_RX_BASE_ER      0x0FU
#define MOTOR_DIAG_RX_BASE_MT      0x10U
#define MOTOR_DIAG_RX_BASE_MO      0x15U
#define MOTOR_DIAG_RX_BASE_BG      0x16U
#define MOTOR_DIAG_RX_BASE_ST      0x17U
#define MOTOR_DIAG_RX_BASE_SD      0x1CU
#define MOTOR_DIAG_RX_BASE_PA      0x20U
#define MOTOR_DIAG_RX_BASE_OG      0x21U
#define MOTOR_DIAG_RX_BASE_MP      0x22U
#define MOTOR_DIAG_RX_BASE_PV      0x23U
#define MOTOR_DIAG_RX_BASE_LM_SET  0x24U
#define MOTOR_DIAG_RX_BASE_LM_GET  0x2CU
#define MOTOR_DIAG_RX_BASE_BL      0x2DU
#define MOTOR_DIAG_RX_BASE_DV      0x2EU
#define MOTOR_DIAG_RX_BASE_IL      0x34U
#define MOTOR_DIAG_RX_BASE_QE      0x3DU
#define MOTOR_DIAG_RX_BASE_NOTIFY  0x5AU
#define MOTOR_DIAG_RX_NOTIFY_ALARM 0x00U

static const motor_diag_enum_entry_t s_pp_bitrate_enum[] = {
    {MOTOR_PP_BITRATE_1M, "CAN bitrate", "1000000", "bps"},
    {MOTOR_PP_BITRATE_800K, "CAN bitrate", "800000", "bps"},
    {MOTOR_PP_BITRATE_500K, "CAN bitrate", "500000", "bps"},
    {MOTOR_PP_BITRATE_250K, "CAN bitrate", "250000", "bps"},
    {MOTOR_PP_BITRATE_125K, "CAN bitrate", "125000", "bps"},
};

static const motor_diag_enum_entry_t s_bool_off_on_enum[] = {
    {0, NULL, "OFF", NULL},
    {1, NULL, "ON", NULL},
};

static const motor_diag_enum_entry_t s_bool_disabled_enabled_enum[] = {
    {0, NULL, "disabled", NULL},
    {1, NULL, "enabled", NULL},
};

static const motor_diag_enum_entry_t s_closed_loop_enum[] = {
    {0, NULL, "open loop", NULL},
    {1, NULL, "closed loop", NULL},
};

static const motor_diag_enum_entry_t s_mo_state_enum[] = {
    {MOTOR_MO_STATE_DISABLE, NULL, "OFF", NULL},
    {MOTOR_MO_STATE_ENABLE, NULL, "ON", NULL},
};

static const motor_diag_enum_entry_t s_mp_mode_enum[] = {
    {MOTOR_MP_MODE_FIFO, NULL, "FIFO mode", NULL},
    {MOTOR_MP_MODE_SINGLE, NULL, "Single mode", NULL},
    {MOTOR_MP_MODE_LOOP, NULL, "Loop mode", NULL},
};

static const motor_diag_family_meta_t s_family_meta[] = {
    {MOTOR_OBJECT_PP, "PP", "Position protocol parameter"},
    {MOTOR_OBJECT_IC, "IC", "Initial configuration"},
    {MOTOR_OBJECT_IE, "IE", "Inform enable"},
    {MOTOR_OBJECT_ER, "ER", "Error history"},
    {MOTOR_OBJECT_QE, "QE", "Quadrature encoder"},
    {MOTOR_OBJECT_SY, "SY", "System operation"},
    {MOTOR_OBJECT_MT, "MT", "Motor driver tuning"},
    {MOTOR_OBJECT_IL, "IL", "Input logic"},
    {MOTOR_OBJECT_MO, "MO", "Motor driver"},
    {MOTOR_OBJECT_BG, "BG", "Begin motion"},
    {MOTOR_OBJECT_ST, "ST", "Stop motion"},
    {MOTOR_OBJECT_SD, "SD", "Stop deceleration"},
    {MOTOR_OBJECT_PA, "PA", "Absolute position"},
    {MOTOR_OBJECT_MP, "MP", "PVT motion parameter"},
    {MOTOR_OBJECT_PV, "PV", "PVT motion row selector"},
    {MOTOR_OBJECT_PT, "PT", "PT queue row position"},
    {MOTOR_OBJECT_OG, "OG", "Set origin"},
    {MOTOR_OBJECT_LM, "LM", "Limit"},
    {MOTOR_OBJECT_BL, "BL", "Backlash compensation"},
    {MOTOR_OBJECT_RT, "RT", "Realtime notification"},
    {MOTOR_OBJECT_DV, "DV", "Drive variable"},
};

static const motor_diag_param_meta_t s_param_meta[] = {
    {MOTOR_OBJECT_PP, MOTOR_PP_INDEX_BITRATE, "CAN bitrate", MOTOR_DIAG_VALUE_KIND_ENUM,
     s_pp_bitrate_enum, ARRAY_LEN(s_pp_bitrate_enum), "bps"},
    {MOTOR_OBJECT_PP, MOTOR_PP_INDEX_NODE_ID, "Node ID", MOTOR_DIAG_VALUE_KIND_U8,
     NULL, 0U, NULL},
    {MOTOR_OBJECT_PP, MOTOR_PP_INDEX_GROUP_ID, "Group ID", MOTOR_DIAG_VALUE_KIND_U8,
     NULL, 0U, NULL},
    {MOTOR_OBJECT_IC, MOTOR_IC_INDEX_AUTO_ENABLE_AFTER_POWERUP, "Auto-Enable Motor Driver after Power up",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_bool_off_on_enum, ARRAY_LEN(s_bool_off_on_enum), NULL},
    {MOTOR_OBJECT_IC, MOTOR_IC_INDEX_POSITIVE_MOTOR_DIRECTION, "Positive Motor Direction",
     MOTOR_DIAG_VALUE_KIND_U16, NULL, 0U, NULL},
    {MOTOR_OBJECT_IC, MOTOR_IC_INDEX_ENABLE_LOCKDOWN_SYSTEM, "Enable Lockdown System",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_bool_off_on_enum, ARRAY_LEN(s_bool_off_on_enum), NULL},
    {MOTOR_OBJECT_IC, MOTOR_IC_INDEX_AC_DC_UNITS, "Units for AC and DC",
     MOTOR_DIAG_VALUE_KIND_U16, NULL, 0U, NULL},
    {MOTOR_OBJECT_IC, MOTOR_IC_INDEX_USE_CLOSED_LOOP, "Using Closed-loop Control",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_closed_loop_enum, ARRAY_LEN(s_closed_loop_enum), NULL},
    {MOTOR_OBJECT_IC, MOTOR_IC_INDEX_SOFTWARE_LIMIT_ENABLE, "Software Limit",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_bool_off_on_enum, ARRAY_LEN(s_bool_off_on_enum), NULL},
    {MOTOR_OBJECT_IC, MOTOR_IC_INDEX_INTERNAL_BRAKE_LOGIC_ENABLE, "Internally Controlled Brake Logic",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_bool_off_on_enum, ARRAY_LEN(s_bool_off_on_enum), NULL},
    {MOTOR_OBJECT_IC, MOTOR_IC_INDEX_USE_INTERNAL_BRAKE, "Using Internally Controlled Brake",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_bool_off_on_enum, ARRAY_LEN(s_bool_off_on_enum), NULL},
    {MOTOR_OBJECT_IE, MOTOR_IE_INDEX_IN1_CHANGE_NOTIFICATION, "IN1 change notification",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_bool_disabled_enabled_enum, ARRAY_LEN(s_bool_disabled_enabled_enum), NULL},
    {MOTOR_OBJECT_IE, MOTOR_IE_INDEX_IN2_CHANGE_NOTIFICATION, "IN2 change notification",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_bool_disabled_enabled_enum, ARRAY_LEN(s_bool_disabled_enabled_enum), NULL},
    {MOTOR_OBJECT_IE, MOTOR_IE_INDEX_IN3_CHANGE_NOTIFICATION, "IN3 change notification",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_bool_disabled_enabled_enum, ARRAY_LEN(s_bool_disabled_enabled_enum), NULL},
    {MOTOR_OBJECT_IE, MOTOR_IE_INDEX_PTP_POSITIONING_FINISH_NOTIFICATION, "PTP positioning finish notification",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_bool_disabled_enabled_enum, ARRAY_LEN(s_bool_disabled_enabled_enum), NULL},
    {MOTOR_OBJECT_IE, MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION, "FIFO empty notification",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_bool_disabled_enabled_enum, ARRAY_LEN(s_bool_disabled_enabled_enum), NULL},
    {MOTOR_OBJECT_IE, MOTOR_IE_INDEX_FIFO_LOW_WARNING_NOTIFICATION, "FIFO low warning notification",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_bool_disabled_enabled_enum, ARRAY_LEN(s_bool_disabled_enabled_enum), NULL},
    {MOTOR_OBJECT_ER, MOTOR_ER_INDEX_CLEAR_ALL, "Clear all errors",
     MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_ER, MOTOR_ER_INDEX_HISTORY_1, "First error slot",
     MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_ER, MOTOR_ER_INDEX_HISTORY_2, "Second error slot",
     MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_ER, MOTOR_ER_INDEX_HISTORY_3, "Third error slot",
     MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_ER, MOTOR_ER_INDEX_HISTORY_4, "Fourth error slot",
     MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_ER, MOTOR_ER_INDEX_HISTORY_5, "Fifth error slot",
     MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_ER, MOTOR_ER_INDEX_HISTORY_6, "Sixth error slot",
     MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_ER, MOTOR_ER_INDEX_HISTORY_7, "Seventh error slot",
     MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_ER, MOTOR_ER_INDEX_HISTORY_8, "Eighth error slot",
     MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_ER, MOTOR_ER_INDEX_HISTORY_9, "Ninth error slot",
     MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_QE, MOTOR_QE_INDEX_LINES_PER_REVOLUTION, "Lines per revolution", MOTOR_DIAG_VALUE_KIND_U16,
     NULL, 0U, "lines"},
    {MOTOR_OBJECT_QE, MOTOR_QE_INDEX_MAX_POSITION_ERROR, "Maximum position error", MOTOR_DIAG_VALUE_KIND_U16,
     NULL, 0U, "pulse"},
    {MOTOR_OBJECT_QE, MOTOR_QE_INDEX_BATTERY_VOLTAGE, "Battery voltage", MOTOR_DIAG_VALUE_KIND_U16,
     NULL, 0U, NULL},
    {MOTOR_OBJECT_QE, MOTOR_QE_INDEX_COUNTS_PER_REVOLUTION, "Counts per revolution", MOTOR_DIAG_VALUE_KIND_U16,
     NULL, 0U, "counts/rev"},
    {MOTOR_OBJECT_SY, MOTOR_SY_OP_REBOOT, "Reboot", MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_SY, MOTOR_SY_OP_RESTORE_FACTORY_DEFAULTS, "Restore factory defaults",
     MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_MT, MOTOR_MT_INDEX_MICROSTEP, "Micro-Stepping Resolution",
     MOTOR_DIAG_VALUE_KIND_U16, NULL, 0U, NULL},
    {MOTOR_OBJECT_MT, MOTOR_MT_INDEX_WORKING_CURRENT, "Working Current",
     MOTOR_DIAG_VALUE_KIND_U16, NULL, 0U, NULL},
    {MOTOR_OBJECT_MT, MOTOR_MT_INDEX_IDLE_CURRENT_PERCENT, "Percentage of Idle Current over Working Current",
     MOTOR_DIAG_VALUE_KIND_U16, NULL, 0U, "%"},
    {MOTOR_OBJECT_MT, MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS, "Delay of Automatic Enable after Power-on",
     MOTOR_DIAG_VALUE_KIND_U16, NULL, 0U, "ms"},
    {MOTOR_OBJECT_MT, MOTOR_MT_INDEX_BRAKE_STATE, "Enable/Release Internal Controlled Brake",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_bool_off_on_enum, ARRAY_LEN(s_bool_off_on_enum), NULL},
    {MOTOR_OBJECT_IL, MOTOR_IL_INDEX_IN1_TRIGGER_ACTION, "IN1 trigger action", MOTOR_DIAG_VALUE_KIND_U16,
     NULL, 0U, NULL},
    {MOTOR_OBJECT_IL, MOTOR_IL_INDEX_IN2_TRIGGER_ACTION, "IN2 trigger action", MOTOR_DIAG_VALUE_KIND_U16,
     NULL, 0U, NULL},
    {MOTOR_OBJECT_IL, MOTOR_IL_INDEX_IN3_TRIGGER_ACTION, "IN3 trigger action", MOTOR_DIAG_VALUE_KIND_U16,
     NULL, 0U, NULL},
    {MOTOR_OBJECT_IL, MOTOR_IL_INDEX_STALL_BEHAVIOR, "Stall behavior", MOTOR_DIAG_VALUE_KIND_U16,
     NULL, 0U, NULL},
    {MOTOR_OBJECT_MO, 0U, "Motor driver", MOTOR_DIAG_VALUE_KIND_ENUM,
     s_mo_state_enum, ARRAY_LEN(s_mo_state_enum), NULL},
    {MOTOR_OBJECT_BG, 0U, "Begin motion", MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_ST, 0U, "Stop motion", MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_SD, 0U, "Stop deceleration", MOTOR_DIAG_VALUE_KIND_U32, NULL, 0U, "pulse/sec^2"},
    {MOTOR_OBJECT_PA, 0U, "Absolute position", MOTOR_DIAG_VALUE_KIND_I32, NULL, 0U, "pulse"},
    {MOTOR_OBJECT_MP, MOTOR_MP_INDEX_CURRENT_QUEUE_LEVEL_OR_RESET_TABLE, "Queue level or table control parameter",
     MOTOR_DIAG_VALUE_KIND_U16, NULL, 0U, NULL},
    {MOTOR_OBJECT_MP, MOTOR_MP_INDEX_FIRST_VALID_ROW, "First valid row", MOTOR_DIAG_VALUE_KIND_U16,
     NULL, 0U, NULL},
    {MOTOR_OBJECT_MP, MOTOR_MP_INDEX_LAST_VALID_ROW, "Last valid row", MOTOR_DIAG_VALUE_KIND_U16,
     NULL, 0U, NULL},
    {MOTOR_OBJECT_MP, MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE, "PVT data management mode",
     MOTOR_DIAG_VALUE_KIND_ENUM, s_mp_mode_enum, ARRAY_LEN(s_mp_mode_enum), NULL},
    {MOTOR_OBJECT_MP, MOTOR_MP_INDEX_PT_MOTION_TIME, "PT motion time", MOTOR_DIAG_VALUE_KIND_U16,
     NULL, 0U, "ms"},
    {MOTOR_OBJECT_MP, MOTOR_MP_INDEX_QUEUE_LOW_THRESHOLD, "Queue low threshold", MOTOR_DIAG_VALUE_KIND_U16,
     NULL, 0U, NULL},
    {MOTOR_OBJECT_MP, MOTOR_MP_INDEX_NEXT_AVAILABLE_WRITING_ROW, "Next available writing row",
     MOTOR_DIAG_VALUE_KIND_U16, NULL, 0U, NULL},
    {MOTOR_OBJECT_PV, 0U, "PVT row index", MOTOR_DIAG_VALUE_KIND_U16, NULL, 0U, NULL},
    {MOTOR_OBJECT_PT, 0U, "PT queued position", MOTOR_DIAG_VALUE_KIND_I32, NULL, 0U, "pulse"},
    {MOTOR_OBJECT_OG, 0U, "Set origin", MOTOR_DIAG_VALUE_KIND_NONE, NULL, 0U, NULL},
    {MOTOR_OBJECT_LM, MOTOR_LM_INDEX_MAX_SPEED, "Maximum Speed", MOTOR_DIAG_VALUE_KIND_I32, NULL, 0U, "pulse/sec"},
    {MOTOR_OBJECT_LM, MOTOR_LM_INDEX_LOWER_WORKING_LIMIT, "Lower Working Limit", MOTOR_DIAG_VALUE_KIND_I32, NULL, 0U, "pulse"},
    {MOTOR_OBJECT_LM, MOTOR_LM_INDEX_UPPER_WORKING_LIMIT, "Upper Working Limit", MOTOR_DIAG_VALUE_KIND_I32, NULL, 0U, "pulse"},
    {MOTOR_OBJECT_LM, MOTOR_LM_INDEX_LOWER_BUMPING_LIMIT, "Lower Bumping Limit", MOTOR_DIAG_VALUE_KIND_I32, NULL, 0U, "pulse"},
    {MOTOR_OBJECT_LM, MOTOR_LM_INDEX_UPPER_BUMPING_LIMIT, "Upper Bumping Limit", MOTOR_DIAG_VALUE_KIND_I32, NULL, 0U, "pulse"},
    {MOTOR_OBJECT_LM, MOTOR_LM_INDEX_MAX_POSITION_ERROR, "Maximum Position Error", MOTOR_DIAG_VALUE_KIND_I32, NULL, 0U, "pulse"},
    {MOTOR_OBJECT_LM, MOTOR_LM_INDEX_MAX_ACCEL_DECEL, "Maximum Acceleration/Deceleration", MOTOR_DIAG_VALUE_KIND_I32, NULL, 0U, "pulse/sec^2"},
    {MOTOR_OBJECT_LM, MOTOR_LM_INDEX_RESET_LIMITS, "Reset working / bumping limits", MOTOR_DIAG_VALUE_KIND_I32, NULL, 0U, NULL},
    {MOTOR_OBJECT_LM, MOTOR_LM_INDEX_ENABLE_DISABLE, "Enable/disable software limits", MOTOR_DIAG_VALUE_KIND_I32, NULL, 0U, NULL},
    {MOTOR_OBJECT_BL, 0U, "Backlash compensation", MOTOR_DIAG_VALUE_KIND_U32, NULL, 0U, "pulse"},
    {MOTOR_OBJECT_DV, 4U, "Absolute position", MOTOR_DIAG_VALUE_KIND_I32, NULL, 0U, "pulse"},
};

static const motor_diag_error_meta_t s_error_meta[] = {
    {0x32U, "Instruction syntax error"},
    {0x33U, "Instruction data error"},
    {0x34U, "Instruction sub-index error"},
    {0x3CU, "SD value is less than DC value"},
    {0x3DU, "Current instruction is not allowed when motor is running"},
    {0x3EU, "BG is not allowed when motor driver is OFF"},
    {0x3FU, "BG is not allowed during emergency stopping"},
    {0x41U, "OG is not allowed when motor is running"},
};

static const motor_diag_rt_meta_t s_rt_meta[] = {
    {true, 0x01U, "Emergency stop and lock", MOTOR_DIAG_VALUE_KIND_NONE, NULL},
    {false, 0x11U, "Input 1 falling edge", MOTOR_DIAG_VALUE_KIND_NONE, NULL},
    {false, 0x12U, "Input 1 rising edge", MOTOR_DIAG_VALUE_KIND_NONE, NULL},
    {false, 0x29U, "PTP positioning completed", MOTOR_DIAG_VALUE_KIND_I32, "pulse"},
};

const motor_diag_family_meta_t *motor_diag_lookup_family(motor_object_t object)
{
    size_t i;

    for (i = 0; i < ARRAY_LEN(s_family_meta); ++i) {
        if (s_family_meta[i].object == object) {
            return &s_family_meta[i];
        }
    }

    return NULL;
}

const motor_diag_param_meta_t *motor_diag_lookup_param(motor_object_t object, uint16_t index)
{
    size_t i;

    for (i = 0; i < ARRAY_LEN(s_param_meta); ++i) {
        if (s_param_meta[i].object == object && s_param_meta[i].index == index) {
            return &s_param_meta[i];
        }
    }

    return NULL;
}

const motor_diag_enum_entry_t *motor_diag_lookup_enum(const motor_diag_enum_entry_t *entries,
                                                      size_t entry_count,
                                                      int32_t raw_value)
{
    size_t i;

    if (entries == NULL) {
        return NULL;
    }

    for (i = 0; i < entry_count; ++i) {
        if (entries[i].raw_value == raw_value) {
            return &entries[i];
        }
    }

    return NULL;
}

const motor_diag_error_meta_t *motor_diag_lookup_error(uint8_t code)
{
    size_t i;

    for (i = 0; i < ARRAY_LEN(s_error_meta); ++i) {
        if (s_error_meta[i].code == code) {
            return &s_error_meta[i];
        }
    }

    return NULL;
}

const motor_diag_rt_meta_t *motor_diag_lookup_rt(bool is_alarm, uint8_t code)
{
    size_t i;

    for (i = 0; i < ARRAY_LEN(s_rt_meta); ++i) {
        if (s_rt_meta[i].is_alarm == is_alarm && s_rt_meta[i].code == code) {
            return &s_rt_meta[i];
        }
    }

    return NULL;
}

const char *motor_diag_family_symbol_from_base(uint8_t base_code)
{
    switch (base_code) {
        case MOTOR_DIAG_RX_BASE_PP:
            return "PP";
        case MOTOR_DIAG_RX_BASE_IC:
            return "IC";
        case MOTOR_DIAG_RX_BASE_IE:
            return "IE";
        case MOTOR_DIAG_RX_BASE_ER:
            return "ER";
        case MOTOR_DIAG_RX_BASE_QE:
            return "QE";
        case MOTOR_DIAG_RX_BASE_MT:
            return "MT";
        case MOTOR_DIAG_RX_BASE_IL:
            return "IL";
        case MOTOR_DIAG_RX_BASE_MO:
            return "MO";
        case MOTOR_DIAG_RX_BASE_BG:
            return "BG";
        case MOTOR_DIAG_RX_BASE_ST:
            return "ST";
        case MOTOR_DIAG_RX_BASE_SD:
            return "SD";
        case MOTOR_DIAG_RX_BASE_PA:
            return "PA";
        case MOTOR_DIAG_RX_BASE_MP:
            return "MP";
        case MOTOR_DIAG_RX_BASE_PV:
            return "PV";
        case MOTOR_DIAG_RX_BASE_OG:
            return "OG";
        case MOTOR_DIAG_RX_BASE_LM_SET:
            return "PT/LM";
        case MOTOR_DIAG_RX_BASE_LM_GET:
            return "LM";
        case MOTOR_DIAG_RX_BASE_BL:
            return "BL";
        case MOTOR_DIAG_RX_BASE_DV:
            return "DV";
        case MOTOR_DIAG_RX_BASE_NOTIFY:
            return "RT";
        default:
            return NULL;
    }
}

static void set_numeric_value(motor_diag_value_desc_t *out_value,
                              motor_diag_value_kind_t value_kind,
                              int64_t number,
                              const char *units)
{
    out_value->kind = value_kind;
    out_value->has_value = true;
    out_value->number = number;
    out_value->has_raw_number = true;
    out_value->raw_number = number;
    out_value->units = units;
}

static bool apply_param_value(int64_t raw_value,
                              const motor_diag_param_meta_t *param_meta,
                              motor_diag_value_desc_t *out_value)
{
    const motor_diag_enum_entry_t *enum_entry;

    if (param_meta == NULL || out_value == NULL) {
        return false;
    }

    if (param_meta->object == MOTOR_OBJECT_MP &&
        param_meta->index == MOTOR_MP_INDEX_PT_MOTION_TIME &&
        raw_value == 0) {
        out_value->kind = MOTOR_DIAG_VALUE_KIND_TEXT;
        out_value->has_value = true;
        out_value->number = raw_value;
        out_value->has_raw_number = true;
        out_value->raw_number = raw_value;
        out_value->text = "0";
        out_value->units = NULL;
        return true;
    }

    if (param_meta->enum_entries != NULL) {
        enum_entry = motor_diag_lookup_enum(param_meta->enum_entries,
                                            param_meta->enum_entry_count,
                                            (int32_t)raw_value);
        if (enum_entry != NULL) {
            out_value->kind = MOTOR_DIAG_VALUE_KIND_ENUM;
            out_value->has_value = true;
            out_value->number = raw_value;
            out_value->has_raw_number = true;
            out_value->raw_number = raw_value;
            out_value->text = enum_entry->render_text;
            out_value->units = enum_entry->units != NULL ? enum_entry->units : param_meta->units;
            return true;
        }
    }

    switch (param_meta->value_kind) {
        case MOTOR_DIAG_VALUE_KIND_U8:
        case MOTOR_DIAG_VALUE_KIND_U16:
        case MOTOR_DIAG_VALUE_KIND_U32:
        case MOTOR_DIAG_VALUE_KIND_I32:
            set_numeric_value(out_value, param_meta->value_kind, raw_value, param_meta->units);
            return true;
        case MOTOR_DIAG_VALUE_KIND_NONE:
            out_value->kind = MOTOR_DIAG_VALUE_KIND_NONE;
            return true;
        default:
            return false;
    }
}

bool motor_diag_decode_cmd_value(const motor_cmd_t *cmd,
                                 const motor_diag_param_meta_t *param_meta,
                                 motor_diag_value_desc_t *out_value)
{
    int64_t raw_value;

    if (cmd == NULL || out_value == NULL || param_meta == NULL) {
        return false;
    }

    switch (cmd->object) {
        case MOTOR_OBJECT_PP:
            if (cmd->msg.data_length_code < 2U) {
                return false;
            }
            raw_value = cmd->msg.data[1];
            return apply_param_value(raw_value, param_meta, out_value);
        case MOTOR_OBJECT_IC:
        case MOTOR_OBJECT_IE:
        case MOTOR_OBJECT_QE:
        case MOTOR_OBJECT_MT:
        case MOTOR_OBJECT_IL:
        case MOTOR_OBJECT_MP:
            if (cmd->msg.data_length_code < 3U) {
                return false;
            }
            raw_value = motor_codec_unpack_u16_le(&cmd->msg.data[1]);
            return apply_param_value(raw_value, param_meta, out_value);
        case MOTOR_OBJECT_MO:
            if (cmd->msg.data_length_code < 1U) {
                return false;
            }
            raw_value = cmd->msg.data[0];
            return apply_param_value(raw_value, param_meta, out_value);
        case MOTOR_OBJECT_PV:
            if (cmd->msg.data_length_code < 2U) {
                return false;
            }
            raw_value = motor_codec_unpack_u16_le(&cmd->msg.data[0]);
            return apply_param_value(raw_value, param_meta, out_value);
        case MOTOR_OBJECT_PA:
            if (cmd->msg.data_length_code < 4U) {
                return false;
            }
            raw_value = motor_codec_unpack_i32_le(&cmd->msg.data[0]);
            return apply_param_value(raw_value, param_meta, out_value);
        case MOTOR_OBJECT_PT:
            if (cmd->msg.data_length_code < 6U) {
                return false;
            }
            raw_value = motor_codec_unpack_i32_le(&cmd->msg.data[2]);
            return apply_param_value(raw_value, param_meta, out_value);
        case MOTOR_OBJECT_SD:
        case MOTOR_OBJECT_BL:
            if (cmd->msg.data_length_code < 4U) {
                return false;
            }
            raw_value = motor_codec_unpack_u32_le(&cmd->msg.data[0]);
            return apply_param_value(raw_value, param_meta, out_value);
        case MOTOR_OBJECT_LM:
            if (cmd->msg.data_length_code < 5U) {
                return false;
            }
            raw_value = motor_codec_unpack_i32_le(&cmd->msg.data[1]);
            return apply_param_value(raw_value, param_meta, out_value);
        default:
            return false;
    }
}

bool motor_diag_describe_ack_value(const motor_rx_t *rx,
                                   motor_diag_rx_desc_t *out,
                                   const motor_diag_param_meta_t *param_meta)
{
    motor_pp_index_t pp_index;
    motor_ic_index_t ic_index;
    motor_ie_index_t ie_index;
    motor_qe_index_t qe_index;
    motor_mt_index_t mt_index;
    motor_il_index_t il_index;
    motor_mp_index_t mp_index;
    motor_lm_index_t lm_index;
    uint8_t u8_value;
    uint16_t u16_value;
    uint16_t row_index;
    int32_t i32_value;
    uint32_t u32_value;

    if (rx == NULL || out == NULL || param_meta == NULL) {
        return false;
    }

    switch (rx->object) {
        case MOTOR_OBJECT_PP:
            return motor_rx_ack_pp(rx, &pp_index, &u8_value) &&
                   apply_param_value(u8_value, param_meta, &out->value);
        case MOTOR_OBJECT_IC:
            return motor_rx_ack_ic(rx, &ic_index, &u16_value) &&
                   apply_param_value(u16_value, param_meta, &out->value);
        case MOTOR_OBJECT_IE:
            return motor_rx_ack_ie(rx, &ie_index, &u16_value) &&
                   apply_param_value(u16_value, param_meta, &out->value);
        case MOTOR_OBJECT_QE:
            return motor_rx_ack_qe(rx, &qe_index, &u16_value) &&
                   apply_param_value(u16_value, param_meta, &out->value);
        case MOTOR_OBJECT_MT:
            return motor_rx_ack_mt(rx, &mt_index, &u16_value) &&
                   apply_param_value(u16_value, param_meta, &out->value);
        case MOTOR_OBJECT_IL:
            return motor_rx_ack_il(rx, &il_index, &u16_value) &&
                   apply_param_value(u16_value, param_meta, &out->value);
        case MOTOR_OBJECT_MO:
            return motor_rx_ack_mo(rx, &u8_value) &&
                   apply_param_value(u8_value, param_meta, &out->value);
        case MOTOR_OBJECT_BG:
            return motor_rx_ack_bg(rx) &&
                   apply_param_value(0, param_meta, &out->value);
        case MOTOR_OBJECT_ST:
            return motor_rx_ack_st(rx) &&
                   apply_param_value(0, param_meta, &out->value);
        case MOTOR_OBJECT_SD:
            return motor_rx_ack_sd(rx, &u32_value) &&
                   apply_param_value(u32_value, param_meta, &out->value);
        case MOTOR_OBJECT_PA:
            return motor_rx_ack_pa_get(rx, &i32_value) &&
                   apply_param_value(i32_value, param_meta, &out->value);
        case MOTOR_OBJECT_MP:
            return motor_rx_ack_mp(rx, &mp_index, &u16_value) &&
                   apply_param_value(u16_value, param_meta, &out->value);
        case MOTOR_OBJECT_PV:
            return motor_rx_ack_pv(rx, &u16_value) &&
                   apply_param_value(u16_value, param_meta, &out->value);
        case MOTOR_OBJECT_PT:
            return motor_rx_ack_pt(rx, &row_index, &i32_value) &&
                   apply_param_value(i32_value, param_meta, &out->value);
        case MOTOR_OBJECT_OG:
            return motor_rx_ack_og(rx) &&
                   apply_param_value(0, param_meta, &out->value);
        case MOTOR_OBJECT_LM:
            return (motor_rx_ack_lm_get(rx, &lm_index, &i32_value) ||
                    motor_rx_ack_lm_set(rx, &lm_index, &i32_value)) &&
                   apply_param_value(i32_value, param_meta, &out->value);
        case MOTOR_OBJECT_DV:
            return motor_rx_ack_pa_set(rx, &i32_value) &&
                   apply_param_value(i32_value, param_meta, &out->value);
        case MOTOR_OBJECT_BL:
            return motor_rx_ack_bl(rx, &u32_value) &&
                   apply_param_value(u32_value, param_meta, &out->value);
        default:
            return false;
    }
}
