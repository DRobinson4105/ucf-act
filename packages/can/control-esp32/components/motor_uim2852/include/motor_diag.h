/*
 * Responsibility:
 * Public API surface for semantic description and rendering of motor protocol traffic.
 */

#ifndef MOTOR_DIAG_H
#define MOTOR_DIAG_H

#include <stdbool.h>
#include <stddef.h>

#include "motor_diag_types.h"

#ifdef __cplusplus
extern "C" {
#endif

bool motor_diag_describe_cmd(const motor_cmd_t *cmd, motor_diag_cmd_desc_t *out);
bool motor_diag_describe_rx(const motor_rx_t *rx, motor_diag_rx_desc_t *out);

size_t motor_diag_format_cmd(char *buf, size_t cap, const motor_diag_cmd_desc_t *desc);
size_t motor_diag_format_rx(char *buf, size_t cap, const motor_diag_rx_desc_t *desc);

void motor_diag_log_cmd(const motor_cmd_t *cmd);
void motor_diag_log_rx(const motor_rx_t *rx);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_DIAG_H */
