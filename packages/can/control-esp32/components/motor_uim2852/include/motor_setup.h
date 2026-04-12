/*
 * Responsibility:
 * Public blocking plan runner layered above motor_exec for ordered setup
 * sequences.
 */

#ifndef MOTOR_SETUP_H
#define MOTOR_SETUP_H

#include "motor_setup_types.h"

#ifdef __cplusplus
extern "C" {
#endif

motor_setup_run_cfg_t motor_setup_default_run_cfg(void);
motor_setup_log_cfg_t motor_setup_default_log_cfg(void);

esp_err_t motor_setup_set_log_cfg(const motor_setup_log_cfg_t *cfg);
esp_err_t motor_setup_get_log_cfg(motor_setup_log_cfg_t *out_cfg);

const motor_setup_plan_t *motor_setup_brake_pt_pv_demo_plan(void);
const motor_setup_plan_t *motor_setup_steering_pt_pv_demo_plan(void);

esp_err_t motor_setup_validate_plan(const motor_setup_plan_t *plan);
esp_err_t motor_setup_run_plan(const motor_setup_plan_t *plan,
                               const motor_setup_run_cfg_t *cfg,
                               motor_setup_result_t *out_result);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_SETUP_H */
