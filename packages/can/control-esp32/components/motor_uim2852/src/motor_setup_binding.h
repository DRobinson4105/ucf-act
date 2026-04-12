#pragma once

#include "motor_setup.h"

#ifdef __cplusplus
extern "C" {
#endif

bool motor_setup_action_binding_supported(const motor_setup_action_t *action);
motor_exec_submit_result_t motor_setup_submit_action(const motor_setup_action_t *action,
                                                     const motor_exec_submit_opts_t *opts);

#ifdef __cplusplus
}
#endif
