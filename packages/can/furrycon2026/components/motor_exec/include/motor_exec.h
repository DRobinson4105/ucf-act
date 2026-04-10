/*
 * Responsibility:
 * Public nonblocking semantic execution API layered above motor_dispatch.
 */

#ifndef MOTOR_EXEC_H
#define MOTOR_EXEC_H

#include "motor_exec_types.h"
#include "motor_types.h"

#ifdef __cplusplus
extern "C" {
#endif

motor_exec_log_cfg_t motor_exec_default_log_cfg(void);

/*
 * Generic default for ordinary single-command execution.
 * Policy: 100 ms command timeout, short no-ACK grace window.
 *
 * Reboot-like system operations are intentionally not covered by this default.
 * Use motor_exec_reboot_timing() or an explicit override for those cases.
 */
motor_exec_timing_t motor_exec_default_timing(void);
motor_exec_timing_t motor_exec_reboot_timing(void);

esp_err_t motor_exec_set_log_cfg(const motor_exec_log_cfg_t *cfg);
esp_err_t motor_exec_get_log_cfg(motor_exec_log_cfg_t *out_cfg);

bool motor_exec_has_pending(void);

motor_exec_submit_result_t motor_exec_brake_pp_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_pp_index_t index,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_pp_set_u8(uint8_t node_id,
                                                      bool ack_requested,
                                                      motor_pp_index_t index,
                                                      uint8_t value,
                                                      const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_ic_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_ic_index_t index,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_ic_set_u16(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_ic_index_t index,
                                                       uint16_t value,
                                                       const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_ie_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_ie_index_t index,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_ie_set_u16(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_ie_index_t index,
                                                       uint16_t value,
                                                       const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_er_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_er_index_t index,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_er_clear_all(uint8_t node_id,
                                                         bool ack_requested,
                                                         const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_qe_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_qe_index_t index,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_qe_set_u16(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_qe_index_t index,
                                                       uint16_t value,
                                                       const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_sy_reboot(uint8_t node_id,
                                                      const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_sy_restore_defaults(uint8_t node_id,
                                                                const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_mt_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_mt_index_t index,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_mt_set_u16(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_mt_index_t index,
                                                       uint16_t value,
                                                       const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_il_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_il_index_t index,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_il_set_u16(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_il_index_t index,
                                                       uint16_t value,
                                                       const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_mo_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_mo_set(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_mo_state_t state,
                                                   const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_bg_begin(uint8_t node_id,
                                                     bool ack_requested,
                                                     const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_st_stop(uint8_t node_id,
                                                    bool ack_requested,
                                                    const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_pa_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_pa_set(uint8_t node_id,
                                                   bool ack_requested,
                                                   int32_t position,
                                                   const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_mp_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_mp_index_t index,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_mp_set_u16(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_mp_index_t index,
                                                       uint16_t value,
                                                       const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_og_set_origin(uint8_t node_id,
                                                          bool ack_requested,
                                                          const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_lm_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   motor_lm_index_t index,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_lm_set_i32(uint8_t node_id,
                                                       bool ack_requested,
                                                       motor_lm_index_t index,
                                                       int32_t value,
                                                       const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_sd_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_sd_set_u32(uint8_t node_id,
                                                       bool ack_requested,
                                                       uint32_t value,
                                                       const motor_exec_submit_opts_t *opts);

motor_exec_submit_result_t motor_exec_brake_bl_get(uint8_t node_id,
                                                   bool ack_requested,
                                                   const motor_exec_submit_opts_t *opts);
motor_exec_submit_result_t motor_exec_brake_bl_set_u32(uint8_t node_id,
                                                       bool ack_requested,
                                                       uint32_t value,
                                                       const motor_exec_submit_opts_t *opts);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_EXEC_H */
