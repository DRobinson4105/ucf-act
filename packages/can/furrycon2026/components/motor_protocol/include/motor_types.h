/*
 * Responsibility:
 * Shared public protocol data model for commands, RX frames, and enums.
 */

#ifndef MOTOR_TYPES_H
#define MOTOR_TYPES_H

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_SECTION_NONE = 0,
    MOTOR_SECTION_PROTOCOL,
    MOTOR_SECTION_SYSTEM,
    MOTOR_SECTION_MOTOR_DRIVER,
    MOTOR_SECTION_MOTION_CONTROL,
    MOTOR_SECTION_IO,
    MOTOR_SECTION_NOTIFICATION,
    MOTOR_SECTION_INTERPOLATED_MOTION,
} motor_section_t;

typedef enum {
    MOTOR_OBJECT_NONE = 0,
    MOTOR_OBJECT_PP,
    MOTOR_OBJECT_IC,
    MOTOR_OBJECT_IE,
    MOTOR_OBJECT_ML,
    MOTOR_OBJECT_SN,
    MOTOR_OBJECT_ER,
    MOTOR_OBJECT_QE,
    MOTOR_OBJECT_SY,
    MOTOR_OBJECT_MT,
    MOTOR_OBJECT_MO,
    MOTOR_OBJECT_BG,
    MOTOR_OBJECT_ST,
    MOTOR_OBJECT_MF,
    MOTOR_OBJECT_AC,
    MOTOR_OBJECT_DC,
    MOTOR_OBJECT_SS,
    MOTOR_OBJECT_SD,
    MOTOR_OBJECT_JV,
    MOTOR_OBJECT_SP,
    MOTOR_OBJECT_PR,
    MOTOR_OBJECT_PA,
    MOTOR_OBJECT_OG,
    MOTOR_OBJECT_LM,
    MOTOR_OBJECT_BL,
    MOTOR_OBJECT_MS,
    MOTOR_OBJECT_DV,
    MOTOR_OBJECT_IL,
    MOTOR_OBJECT_TG,
    MOTOR_OBJECT_DI,
    MOTOR_OBJECT_RT,
    MOTOR_OBJECT_MP,
    MOTOR_OBJECT_PV,
    MOTOR_OBJECT_QP,
    MOTOR_OBJECT_QV,
    MOTOR_OBJECT_QT,
    MOTOR_OBJECT_QF,
    MOTOR_OBJECT_PT,
} motor_object_t;

typedef enum {
    MOTOR_PP_INDEX_BITRATE = 5,
    MOTOR_PP_INDEX_NODE_ID = 7,
    MOTOR_PP_INDEX_GROUP_ID = 8,
} motor_pp_index_t;

typedef enum {
    MOTOR_IC_INDEX_AUTO_ENABLE_AFTER_POWERUP = 0,
    MOTOR_IC_INDEX_POSITIVE_MOTOR_DIRECTION = 1,
    MOTOR_IC_INDEX_ENABLE_LOCKDOWN_SYSTEM = 3,
    MOTOR_IC_INDEX_AC_DC_UNITS = 4,
    MOTOR_IC_INDEX_USE_CLOSED_LOOP = 6,
    MOTOR_IC_INDEX_SOFTWARE_LIMIT_ENABLE = 7,
    MOTOR_IC_INDEX_INTERNAL_BRAKE_LOGIC_ENABLE = 8,
    MOTOR_IC_INDEX_USE_INTERNAL_BRAKE = 15,
} motor_ic_index_t;

typedef enum {
    MOTOR_IE_INDEX_IN1_CHANGE_NOTIFICATION = 0,
    MOTOR_IE_INDEX_IN2_CHANGE_NOTIFICATION = 1,
    MOTOR_IE_INDEX_IN3_CHANGE_NOTIFICATION = 2,
    MOTOR_IE_INDEX_PTP_POSITIONING_FINISH_NOTIFICATION = 8,
    MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION = 10,
    MOTOR_IE_INDEX_FIFO_LOW_WARNING_NOTIFICATION = 11,
} motor_ie_index_t;

typedef enum {
    /* d0 == 0 is reserved for thrown errors and the ER clear-all command. */
    MOTOR_ER_INDEX_CLEAR_ALL = 0,
    /* Queried history starts at 10; 10 is the newest tracked error. */
    MOTOR_ER_INDEX_HISTORY_1 = 10,
    MOTOR_ER_INDEX_HISTORY_2 = 11,
    MOTOR_ER_INDEX_HISTORY_3 = 12,
    MOTOR_ER_INDEX_HISTORY_4 = 13,
    MOTOR_ER_INDEX_HISTORY_5 = 14,
    MOTOR_ER_INDEX_HISTORY_6 = 15,
    MOTOR_ER_INDEX_HISTORY_7 = 16,
    MOTOR_ER_INDEX_HISTORY_8 = 17,
    MOTOR_ER_INDEX_HISTORY_9 = 18,
} motor_er_index_t;

typedef enum {
    MOTOR_QE_INDEX_LINES_PER_REVOLUTION = 0,
    MOTOR_QE_INDEX_MAX_POSITION_ERROR = 1,
    MOTOR_QE_INDEX_BATTERY_VOLTAGE = 3,
    MOTOR_QE_INDEX_COUNTS_PER_REVOLUTION = 4,
} motor_qe_index_t;

typedef enum {
    MOTOR_SY_OP_REBOOT = 1,
    MOTOR_SY_OP_RESTORE_FACTORY_DEFAULTS = 2,
} motor_sy_op_t;

typedef enum {
    MOTOR_MT_INDEX_MICROSTEP = 0,
    MOTOR_MT_INDEX_WORKING_CURRENT = 1,
    MOTOR_MT_INDEX_IDLE_CURRENT_PERCENT = 2,
    MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS = 3,
    MOTOR_MT_INDEX_BRAKE_STATE = 5,
} motor_mt_index_t;

typedef enum {
    MOTOR_IL_INDEX_IN1_TRIGGER_ACTION = 0,
    MOTOR_IL_INDEX_IN2_TRIGGER_ACTION = 1,
    MOTOR_IL_INDEX_IN3_TRIGGER_ACTION = 2,
    MOTOR_IL_INDEX_STALL_BEHAVIOR = 3,
} motor_il_index_t;

typedef enum {
    MOTOR_LM_INDEX_MAX_SPEED = 0,
    MOTOR_LM_INDEX_LOWER_WORKING_LIMIT = 1,
    MOTOR_LM_INDEX_UPPER_WORKING_LIMIT = 2,
    MOTOR_LM_INDEX_LOWER_BUMPING_LIMIT = 3,
    MOTOR_LM_INDEX_UPPER_BUMPING_LIMIT = 4,
    MOTOR_LM_INDEX_MAX_POSITION_ERROR = 6,
    MOTOR_LM_INDEX_MAX_ACCEL_DECEL = 7,
    MOTOR_LM_INDEX_RESET_LIMITS = 254,
    MOTOR_LM_INDEX_ENABLE_DISABLE = 255,
} motor_lm_index_t;

typedef enum {
    MOTOR_MP_INDEX_CURRENT_QUEUE_LEVEL_OR_RESET_TABLE = 0,
    MOTOR_MP_INDEX_FIRST_VALID_ROW = 1,
    MOTOR_MP_INDEX_LAST_VALID_ROW = 2,
    MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE = 3,
    MOTOR_MP_INDEX_PT_MOTION_TIME = 4,
    MOTOR_MP_INDEX_QUEUE_LOW_THRESHOLD = 5,
    MOTOR_MP_INDEX_NEXT_AVAILABLE_WRITING_ROW = 6,
} motor_mp_index_t;

typedef enum {
    MOTOR_PP_BITRATE_1M = 0,
    MOTOR_PP_BITRATE_800K = 1,
    MOTOR_PP_BITRATE_500K = 2,
    MOTOR_PP_BITRATE_250K = 3,
    MOTOR_PP_BITRATE_125K = 4,
} motor_pp_bitrate_t;

typedef enum {
    MOTOR_MO_STATE_DISABLE = 0,
    MOTOR_MO_STATE_ENABLE = 1,
} motor_mo_state_t;

typedef enum {
    MOTOR_MT_BRAKE_STATE_RELEASED = 0,
    MOTOR_MT_BRAKE_STATE_ENGAGED = 1,
} motor_mt_brake_state_t;

typedef enum {
    MOTOR_IE_STATE_DISABLE = 0,
    MOTOR_IE_STATE_ENABLE = 1,
} motor_ie_state_t;

typedef enum {
    MOTOR_MP_MODE_FIFO = 0,
    MOTOR_MP_MODE_SINGLE = 1,
    MOTOR_MP_MODE_LOOP = 3,
} motor_mp_mode_t;

typedef enum {
    MOTOR_RX_KIND_UNCLASSIFIED = 0,
    MOTOR_RX_KIND_ACK,
    MOTOR_RX_KIND_NOTIFICATION,
    MOTOR_RX_KIND_ERROR,
} motor_rx_kind_t;

typedef struct {
    bool valid;
    uint8_t slot;
    uint8_t code;
    uint8_t related_cw;
    uint8_t related_base_code;
    uint8_t related_subindex;
} motor_rx_error_info_t;

typedef struct {
    bool has_d0;
    bool has_d1;
    uint8_t d0;
    uint8_t d1;
    bool has_current_position;
    int32_t current_position;
} motor_rx_notification_info_t;

typedef struct {
    twai_message_t msg;
    uint8_t target_id;
    uint8_t cw_raw;
    uint8_t base_code;
    bool ack_requested;
    motor_section_t section;
    motor_object_t object;
    bool has_index;
    uint16_t index;
    const char *name;
} motor_cmd_t;

typedef struct {
    motor_rx_kind_t kind;
    uint32_t ext_id_raw;
    uint8_t producer_id;
    uint8_t cw_raw;
    uint8_t base_code;
    uint8_t dl;
    uint8_t data[8];
    motor_section_t section;
    motor_object_t object;
    motor_rx_error_info_t error;
    motor_rx_notification_info_t notification;
} motor_rx_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_TYPES_H */
