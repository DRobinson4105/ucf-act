#pragma once

#include "motor_dispatch.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef motor_dispatch_result_t (*motor_exec_dispatch_fn_t)(const motor_cmd_t *cmd,
                                                            TickType_t timeout_ticks,
                                                            TickType_t no_ack_grace_ticks,
                                                            const motor_dispatch_observer_t *observer,
                                                            motor_rx_t *out_response,
                                                            motor_rx_t *out_related_error);

void motor_exec_test_set_dispatch_fn(motor_exec_dispatch_fn_t fn);
void motor_exec_test_reset_state(void);

#ifdef __cplusplus
}
#endif
