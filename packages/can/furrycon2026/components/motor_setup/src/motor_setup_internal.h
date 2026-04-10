#pragma once

#include "motor_setup.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef motor_exec_submit_result_t (*motor_setup_submit_fn_t)(const motor_setup_action_t *action,
                                                              const motor_exec_submit_opts_t *opts);

void motor_setup_test_set_submit_fn(motor_setup_submit_fn_t fn);
void motor_setup_test_reset_state(void);

#ifdef __cplusplus
}
#endif
