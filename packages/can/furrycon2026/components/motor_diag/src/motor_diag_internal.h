/*
 * Responsibility:
 * Internal metadata schema and shared helpers for motor_diag.
 */

#ifndef MOTOR_DIAG_INTERNAL_H
#define MOTOR_DIAG_INTERNAL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "motor_diag.h"
#include "motor_rx.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int32_t raw_value;
    const char *label;
    const char *render_text;
    const char *units;
} motor_diag_enum_entry_t;

typedef struct {
    motor_object_t object;
    const char *family_symbol;
    const char *family_name;
} motor_diag_family_meta_t;

typedef struct {
    motor_object_t object;
    uint16_t index;
    const char *semantic_name;
    motor_diag_value_kind_t value_kind;
    const motor_diag_enum_entry_t *enum_entries;
    size_t enum_entry_count;
    const char *units;
} motor_diag_param_meta_t;

typedef struct {
    uint8_t code;
    const char *semantic_name;
} motor_diag_error_meta_t;

typedef struct {
    bool is_alarm;
    uint8_t code;
    const char *semantic_name;
    motor_diag_value_kind_t value_kind;
    const char *units;
} motor_diag_rt_meta_t;

const motor_diag_family_meta_t *motor_diag_lookup_family(motor_object_t object);
const motor_diag_param_meta_t *motor_diag_lookup_param(motor_object_t object, uint16_t index);
const motor_diag_enum_entry_t *motor_diag_lookup_enum(const motor_diag_enum_entry_t *entries,
                                                      size_t entry_count,
                                                      int32_t raw_value);
const motor_diag_error_meta_t *motor_diag_lookup_error(uint8_t code);
const motor_diag_rt_meta_t *motor_diag_lookup_rt(bool is_alarm, uint8_t code);
const char *motor_diag_family_symbol_from_base(uint8_t base_code);

bool motor_diag_decode_cmd_value(const motor_cmd_t *cmd,
                                 const motor_diag_param_meta_t *param_meta,
                                 motor_diag_value_desc_t *out_value);

bool motor_diag_describe_ack_value(const motor_rx_t *rx,
                                   motor_diag_rx_desc_t *out,
                                   const motor_diag_param_meta_t *param_meta);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DIAG_INTERNAL_H */
