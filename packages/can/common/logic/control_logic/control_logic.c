/**
 * @file control_logic.c
 * @brief Pure decision logic for the Control ESP32 — no hardware dependencies.
 */

#include "control_logic.h"
#include "can_protocol.hh"

#include <limits.h>

static uint8_t sanitize_target_state(uint8_t target_state) {
    switch (target_state) {
        case NODE_STATE_READY:
        case NODE_STATE_ENABLING:
        case NODE_STATE_ACTIVE:
            return target_state;
        default:
            // Safety target commands only use READY/ENABLING/ACTIVE.
            // Treat all other values as READY (safe retreat).
            return NODE_STATE_READY;
    }
}

uint8_t control_check_preconditions_detailed(const precondition_inputs_t *inputs) {
    if (!inputs) return 0xFF;  // all bits set = all failed

    uint8_t fail = PRECONDITION_OK;

    if (inputs->fr_state != FR_STATE_FORWARD)
        fail |= PRECONDITION_FAIL_FR_NOT_FORWARD;
    if (inputs->pedal_pressed)
        fail |= PRECONDITION_FAIL_PEDAL_PRESSED;
    if (!inputs->pedal_rearmed)
        fail |= PRECONDITION_FAIL_PEDAL_NOT_REARMED;
    if (inputs->fault_code != NODE_FAULT_NONE)
        fail |= PRECONDITION_FAIL_ACTIVE_FAULT;

    return fail;
}

bool control_check_preconditions(const precondition_inputs_t *inputs) {
    return control_check_preconditions_detailed(inputs) == PRECONDITION_OK;
}

throttle_slew_result_t control_compute_throttle_slew(const throttle_slew_inputs_t *inputs) {
    throttle_slew_result_t r = { .new_level = 0, .changed = false };

    if (!inputs) return r;

    r.new_level = inputs->current;

    if (inputs->current == inputs->target) return r;

    uint32_t elapsed = inputs->now_ms - inputs->last_change_ms;
    if (elapsed >= inputs->slew_interval_ms) {
        if (inputs->current < inputs->target)
            r.new_level = inputs->current + 1;
        else
            r.new_level = inputs->current - 1;
        // Clamp to valid throttle range [0, 7]
        if (r.new_level < 0) r.new_level = 0;
        if (r.new_level > 7) r.new_level = 7;
        r.changed = (r.new_level != inputs->current);
    }

    return r;
}

control_step_result_t control_compute_step(uint8_t current_state, uint8_t current_fault,
                                            const control_inputs_t *inputs) {
    control_step_result_t r = {
        .new_state = current_state,
        .new_fault_code = current_fault,
        .override_reason = OVERRIDE_REASON_NONE,
        .disable_reason = CONTROL_DISABLE_REASON_NONE,
        .abort_reason = CONTROL_ABORT_REASON_NONE,
        .precondition_fail = PRECONDITION_OK,
        .heartbeat_flags = 0,
        .actions = CONTROL_ACTION_NONE,
        .throttle_level = 0,
        .throttle_change_ms = 0,
        .send_steering = false,
        .send_braking = false,
        .steering_position = 0,
        .braking_position = 0,
        .new_last_steering = 0,
        .new_last_braking = 0,
        .enable_start_ms = 0,
    };

    if (!inputs) return r;

    uint8_t target_state = sanitize_target_state(inputs->target_state);

    // Carry forward dedup trackers and timing by default
    r.new_last_steering = inputs->last_steering_sent;
    r.new_last_braking = inputs->last_braking_sent;
    r.throttle_level = inputs->throttle_current;
    r.throttle_change_ms = inputs->last_throttle_change_ms;
    r.enable_start_ms = inputs->enable_start_ms;
    r.enable_work_done = inputs->enable_work_done;

    // Check for motor fault from CAN RX task (one-shot)
    if (inputs->motor_fault_code != NODE_FAULT_NONE &&
        current_fault == NODE_FAULT_NONE) {
        if (current_state == NODE_STATE_ACTIVE) {
            r.actions |= CONTROL_ACTION_DISABLE_AUTONOMY;
            r.disable_reason = CONTROL_DISABLE_REASON_MOTOR_FAULT;
        } else if (current_state == NODE_STATE_ENABLING) {
            r.actions |= CONTROL_ACTION_ABORT_ENABLE;
            r.abort_reason = CONTROL_ABORT_REASON_MOTOR_FAULT;
        }
        r.new_state = NODE_STATE_FAULT;
        r.new_fault_code = inputs->motor_fault_code;
        r.throttle_level = 0;
        r.new_last_steering = INT16_MIN;
        r.new_last_braking = INT16_MIN;
        return r;
    }

    // Check for F/R INVALID sensor reading
    if (inputs->fr_is_invalid && current_fault != NODE_FAULT_SENSOR_INVALID) {
        if (current_state == NODE_STATE_ACTIVE) {
            r.actions |= CONTROL_ACTION_DISABLE_AUTONOMY;
            r.disable_reason = CONTROL_DISABLE_REASON_SENSOR_INVALID;
        } else if (current_state == NODE_STATE_ENABLING) {
            r.actions |= CONTROL_ACTION_ABORT_ENABLE;
            r.abort_reason = CONTROL_ABORT_REASON_SENSOR_INVALID;
        }
        r.new_state = NODE_STATE_FAULT;
        r.new_fault_code = NODE_FAULT_SENSOR_INVALID;
        r.throttle_level = 0;
        r.new_last_steering = INT16_MIN;
        r.new_last_braking = INT16_MIN;
        return r;
    }

    switch (current_state) {
        case NODE_STATE_INIT:
            if ((inputs->now_ms - inputs->boot_start_ms) >= inputs->init_dwell_ms) {
                r.new_state = NODE_STATE_READY;
            } else {
                r.new_state = NODE_STATE_INIT;
            }
            break;

        case NODE_STATE_READY: {
            // Safety commands us to start enabling?
            if (target_state >= NODE_STATE_ENABLING) {
                precondition_inputs_t pre = {
                    .fr_state = inputs->fr_state,
                    .pedal_pressed = inputs->pedal_pressed,
                    .pedal_rearmed = inputs->pedal_rearmed,
                    .fault_code = current_fault,
                };
                uint8_t fail = control_check_preconditions_detailed(&pre);
                if (fail == PRECONDITION_OK) {
                    r.new_state = NODE_STATE_ENABLING;
                    r.actions |= CONTROL_ACTION_START_ENABLE;
                    r.enable_start_ms = inputs->now_ms;
                    r.enable_work_done = false;
                    r.throttle_level = 0;
                    r.throttle_change_ms = inputs->now_ms;
                } else {
                    r.precondition_fail = fail;
                }
            }
            break;
        }

        case NODE_STATE_ENABLING: {
            // Safety retreated? Abort enable.
            if (target_state < NODE_STATE_ENABLING) {
                r.new_state = NODE_STATE_READY;
                r.actions |= CONTROL_ACTION_ABORT_ENABLE;
                r.abort_reason = CONTROL_ABORT_REASON_SAFETY_RETREAT;
                r.enable_work_done = false;
                break;
            }
            // Pedal pressed during enable? Abort.
            if (inputs->pedal_pressed) {
                r.new_state = NODE_STATE_READY;
                r.actions |= CONTROL_ACTION_ABORT_ENABLE;
                r.abort_reason = CONTROL_ABORT_REASON_PEDAL_PRESSED;
                r.enable_work_done = false;
                break;
            }
            // FR no longer forward? Abort.
            if (inputs->fr_state != FR_STATE_FORWARD) {
                r.new_state = NODE_STATE_READY;
                r.actions |= CONTROL_ACTION_ABORT_ENABLE;
                r.abort_reason = CONTROL_ABORT_REASON_FR_NOT_FORWARD;
                r.enable_work_done = false;
                break;
            }

            // Check if enable work is done (timer expired)
            if ((inputs->now_ms - inputs->enable_start_ms) >= inputs->enable_sequence_ms) {
                // Only fire COMPLETE_ENABLE action once per enable sequence
                if (!inputs->enable_work_done) {
                    r.actions |= CONTROL_ACTION_COMPLETE_ENABLE;
                }
                r.enable_work_done = true;

                // If Safety already says ACTIVE, go directly to ACTIVE
                if (target_state >= NODE_STATE_ACTIVE) {
                    r.new_state = NODE_STATE_ACTIVE;
                    r.override_reason = OVERRIDE_REASON_NONE;
                } else {
                    // Stay ENABLING, set enable_complete flag for heartbeat
                    // Safety will advance us to ACTIVE once both nodes are ready
                    r.new_state = NODE_STATE_ENABLING;
                    r.heartbeat_flags |= HEARTBEAT_FLAG_ENABLE_COMPLETE;
                }
            }
            // else: stay ENABLING, timer not yet expired
            break;
        }

        case NODE_STATE_ACTIVE: {
            // Safety retreated? Disable actuators and return to READY.
            // This is NOT an override (human intervention) — Safety commanded
            // the retreat, so we go directly to READY rather than OVERRIDE.
            if (target_state < NODE_STATE_ACTIVE) {
                r.actions |= CONTROL_ACTION_DISABLE_AUTONOMY;
                r.disable_reason = CONTROL_DISABLE_REASON_SAFETY_RETREAT;
                r.new_state = NODE_STATE_READY;
                r.throttle_level = 0;
                r.new_last_steering = INT16_MIN;
                r.new_last_braking = INT16_MIN;
                break;
            }
            // Check override conditions
            if (inputs->pedal_pressed) {
                r.actions |= CONTROL_ACTION_TRIGGER_OVERRIDE;
                r.override_reason = OVERRIDE_REASON_PEDAL;
                r.new_state = NODE_STATE_OVERRIDE;
                r.throttle_level = 0;
                r.new_last_steering = INT16_MIN;
                r.new_last_braking = INT16_MIN;
                break;
            }
            if (inputs->fr_state != FR_STATE_FORWARD) {
                r.actions |= CONTROL_ACTION_TRIGGER_OVERRIDE;
                r.override_reason = OVERRIDE_REASON_FR_CHANGED;
                r.new_state = NODE_STATE_OVERRIDE;
                r.throttle_level = 0;
                r.new_last_steering = INT16_MIN;
                r.new_last_braking = INT16_MIN;
                break;
            }
            // Steering position error (external force on steering column)
            if (inputs->steering_position_error) {
                r.actions |= CONTROL_ACTION_TRIGGER_OVERRIDE;
                r.override_reason = OVERRIDE_REASON_STEERING;
                r.new_state = NODE_STATE_OVERRIDE;
                r.throttle_level = 0;
                r.new_last_steering = INT16_MIN;
                r.new_last_braking = INT16_MIN;
                break;
            }
            // Braking position error (external force on brake pedal)
            if (inputs->braking_position_error) {
                r.actions |= CONTROL_ACTION_TRIGGER_OVERRIDE;
                r.override_reason = OVERRIDE_REASON_BRAKING;
                r.new_state = NODE_STATE_OVERRIDE;
                r.throttle_level = 0;
                r.new_last_steering = INT16_MIN;
                r.new_last_braking = INT16_MIN;
                break;
            }

            // Throttle slew
            throttle_slew_inputs_t slew = {
                .current = inputs->throttle_current,
                .target = inputs->throttle_target,
                .last_change_ms = inputs->last_throttle_change_ms,
                .now_ms = inputs->now_ms,
                .slew_interval_ms = inputs->throttle_slew_interval_ms,
            };
            throttle_slew_result_t slew_r = control_compute_throttle_slew(&slew);
            if (slew_r.changed) {
                r.throttle_level = slew_r.new_level;
                r.throttle_change_ms = inputs->now_ms;
                r.actions |= CONTROL_ACTION_APPLY_THROTTLE;
            }

            // Clamp steering/braking commands to safe envelopes before dedup.
            // If min == max == 0, envelope is not configured; force neutral (0)
            // instead of passing through raw planner commands.
            int16_t clamped_steering = inputs->steering_cmd;
            int16_t clamped_braking  = inputs->braking_cmd;
            if (inputs->steering_min == 0 && inputs->steering_max == 0) {
                clamped_steering = 0;
            } else {
                clamped_steering = control_clamp_command(inputs->steering_cmd,
                                                         inputs->steering_min,
                                                         inputs->steering_max);
            }
            if (inputs->braking_min == 0 && inputs->braking_max == 0) {
                clamped_braking = 0;
            } else {
                clamped_braking = control_clamp_command(inputs->braking_cmd,
                                                        inputs->braking_min,
                                                        inputs->braking_max);
            }

            // Stepper dedup (uses clamped values)
            if (clamped_steering != inputs->last_steering_sent) {
                r.send_steering = true;
                r.steering_position = clamped_steering;
                r.new_last_steering = clamped_steering;
            }
            if (clamped_braking != inputs->last_braking_sent) {
                r.send_braking = true;
                r.braking_position = clamped_braking;
                r.new_last_braking = clamped_braking;
            }
            break;
        }

        case NODE_STATE_OVERRIDE:
            r.throttle_level = 0;
            r.new_last_steering = INT16_MIN;
            r.new_last_braking = INT16_MIN;

            // Recover to READY when conditions clear
            // Safety will re-advance us through the state machine
            if (target_state >= NODE_STATE_READY &&
                inputs->fr_state == FR_STATE_FORWARD &&
                inputs->pedal_rearmed) {
                r.new_state = NODE_STATE_READY;
                r.override_reason = OVERRIDE_REASON_NONE;
            }
            break;

        case NODE_STATE_FAULT:
            r.throttle_level = 0;
            r.new_last_steering = INT16_MIN;
            r.new_last_braking = INT16_MIN;
            {
                bool fault_cleared = false;
                switch (current_fault) {
                    case NODE_FAULT_NONE:
                        fault_cleared = true;
                        break;
                    case NODE_FAULT_MOTOR_COMM:
                        fault_cleared = (inputs->motor_fault_code == NODE_FAULT_NONE);
                        break;
                    case NODE_FAULT_SENSOR_INVALID:
                        fault_cleared = !inputs->fr_is_invalid;
                        break;
                    default:
                        fault_cleared = false;
                        break;
                }

                if (fault_cleared) {
                    r.new_state = NODE_STATE_READY;
                    r.new_fault_code = NODE_FAULT_NONE;
                } else {
                    r.actions |= CONTROL_ACTION_ATTEMPT_RECOVERY;
                }
            }
            break;

        default:
            r.new_state = NODE_STATE_FAULT;
            r.new_fault_code = NODE_FAULT_GENERAL;
            r.actions |= CONTROL_ACTION_DISABLE_AUTONOMY;
            r.disable_reason = CONTROL_DISABLE_REASON_NONE;
            r.throttle_level = 0;
            r.new_last_steering = INT16_MIN;
            r.new_last_braking = INT16_MIN;
            break;
    }

    return r;
}

int16_t control_clamp_command(int16_t value, int16_t min, int16_t max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

can_tx_track_result_t control_track_can_tx(const can_tx_track_inputs_t *inputs) {
    can_tx_track_result_t r = { .new_fail_count = 0, .trigger_recovery = false };

    if (!inputs) return r;

    if (inputs->tx_ok) {
        r.new_fail_count = 0;
        return r;
    }

    r.new_fail_count = (inputs->fail_count < 255) ? (uint8_t)(inputs->fail_count + 1) : 255;
    if (r.new_fail_count >= inputs->threshold) {
        r.trigger_recovery = true;
    }

    return r;
}
