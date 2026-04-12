/*
 * Responsibility:
 * Public structured semantic description types for motor_diag.
 */

#ifndef MOTOR_DIAG_TYPES_H
#define MOTOR_DIAG_TYPES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTOR_DIAG_COMPLETENESS_UNKNOWN = 0,
    MOTOR_DIAG_COMPLETENESS_PARTIAL,
    MOTOR_DIAG_COMPLETENESS_COMPLETE,
} motor_diag_completeness_t;

typedef enum {
    MOTOR_DIAG_VALUE_KIND_NONE = 0,
    MOTOR_DIAG_VALUE_KIND_U8,
    MOTOR_DIAG_VALUE_KIND_U16,
    MOTOR_DIAG_VALUE_KIND_U32,
    MOTOR_DIAG_VALUE_KIND_I32,
    MOTOR_DIAG_VALUE_KIND_ENUM,
    MOTOR_DIAG_VALUE_KIND_TEXT,
    MOTOR_DIAG_VALUE_KIND_RAW_BYTES,
} motor_diag_value_kind_t;

typedef enum {
    MOTOR_DIAG_CMD_KIND_UNKNOWN = 0,
    MOTOR_DIAG_CMD_KIND_GET,
    MOTOR_DIAG_CMD_KIND_SET,
    MOTOR_DIAG_CMD_KIND_ACTION,
} motor_diag_cmd_kind_t;

typedef enum {
    MOTOR_DIAG_RX_KIND_UNKNOWN = 0,
    MOTOR_DIAG_RX_KIND_ACK,
    MOTOR_DIAG_RX_KIND_ERROR,
    MOTOR_DIAG_RX_KIND_ER_RESPONSE,
    MOTOR_DIAG_RX_KIND_RT_ALARM,
    MOTOR_DIAG_RX_KIND_RT_STATUS,
    MOTOR_DIAG_RX_KIND_RT_UNKNOWN,
    MOTOR_DIAG_RX_KIND_UNCLASSIFIED,
} motor_diag_rx_kind_t;

typedef struct {
    motor_diag_value_kind_t kind;
    bool has_value;
    int64_t number;
    bool has_raw_number;
    int64_t raw_number;
    const char *text;
    const char *units;
    uint8_t raw[8];
    uint8_t raw_len;
} motor_diag_value_desc_t;

typedef struct {
    uint8_t node_id;
    motor_object_t object;
    const char *family_symbol;
    bool has_index;
    uint16_t index;
    motor_diag_cmd_kind_t kind;
    bool ack_requested;
    const char *semantic_name;
    motor_diag_value_desc_t value;
    motor_diag_completeness_t completeness;
    const char *note;
} motor_diag_cmd_desc_t;

typedef struct {
    uint8_t node_id;
    motor_diag_rx_kind_t kind;
    motor_object_t object;
    uint8_t base_code;
    const char *family_symbol;
    bool has_index;
    uint16_t index;
    bool has_semantic_lineage;
    motor_object_t semantic_object;
    const char *semantic_family_symbol;
    motor_diag_cmd_kind_t semantic_cmd_kind;
    bool has_semantic_index;
    uint16_t semantic_index;
    const char *semantic_name;
    motor_diag_value_desc_t value;
    bool has_error_code;
    uint8_t error_code;
    const char *error_name;
    bool has_related_family;
    const char *related_family_symbol;
    bool has_related_cw;
    uint8_t related_cw;
    bool has_related_index;
    uint16_t related_index;
    motor_diag_completeness_t completeness;
    const char *note;
    uint8_t raw[8];
    uint8_t raw_len;
} motor_diag_rx_desc_t;

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DIAG_TYPES_H */
