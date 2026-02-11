/**
 * @file control_logic.c
 * @brief Pure decision logic for the Control ESP32 — no hardware dependencies.
 */

#include "control_logic.h"
#include "can_protocol.hh"

#include <limits.h>

bool control_check_preconditions(const precondition_inputs_t *inputs) {
    if (!inputs) return false;

    // Must be in Forward
    if (inputs->fr_state != FR_STATE_FORWARD) return false;

    // Pedal must NOT be pressed
    if (inputs->pedal_pressed) return false;

    // Pedal must be re-armed (below threshold for 500ms)
    if (!inputs->pedal_rearmed) return false;

    // No active faults
    if (inputs->fault_code != NODE_FAULT_NONE) return false;

    return true;
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
            r.new_state = NODE_STATE_READY;
            break;

        case NODE_STATE_READY: {
            // Safety commands us to start enabling?
            if (inputs->target_state >= NODE_STATE_ENABLING) {
                precondition_inputs_t pre = {
                    .fr_state = inputs->fr_state,
                    .pedal_pressed = inputs->pedal_pressed,
                    .pedal_rearmed = inputs->pedal_rearmed,
                    .fault_code = current_fault,
                };
                if (control_check_preconditions(&pre)) {
                    r.new_state = NODE_STATE_ENABLING;
                    r.actions |= CONTROL_ACTION_START_ENABLE;
                    r.enable_start_ms = inputs->now_ms;
                    r.enable_work_done = false;
                    r.throttle_level = 0;
                    r.throttle_change_ms = inputs->now_ms;
                }
            }
            break;
        }

        case NODE_STATE_ENABLING: {
            // Safety retreated? Abort enable.
            if (inputs->target_state < NODE_STATE_ENABLING) {
                r.new_state = NODE_STATE_READY;
                r.actions |= CONTROL_ACTION_ABORT_ENABLE;
                r.enable_work_done = false;
                break;
            }
            // Pedal pressed during enable? Abort.
            if (inputs->pedal_pressed) {
                r.new_state = NODE_STATE_READY;
                r.actions |= CONTROL_ACTION_ABORT_ENABLE;
                r.enable_work_done = false;
                break;
            }
            // FR no longer forward? Abort.
            if (inputs->fr_state != FR_STATE_FORWARD) {
                r.new_state = NODE_STATE_READY;
                r.actions |= CONTROL_ACTION_ABORT_ENABLE;
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
                if (inputs->target_state >= NODE_STATE_ACTIVE) {
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
            if (inputs->target_state < NODE_STATE_ACTIVE) {
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

            // Stepper dedup
            if (inputs->steering_cmd != inputs->last_steering_sent) {
                r.send_steering = true;
                r.steering_position = inputs->steering_cmd;
                r.new_last_steering = inputs->steering_cmd;
            }
            if (inputs->braking_cmd != inputs->last_braking_sent) {
                r.send_braking = true;
                r.braking_position = inputs->braking_cmd;
                r.new_last_braking = inputs->braking_cmd;
            }
            break;
        }

        case NODE_STATE_OVERRIDE:
            r.throttle_level = 0;
            r.new_last_steering = INT16_MIN;
            r.new_last_braking = INT16_MIN;

            // Recover to READY when conditions clear
            // Safety will re-advance us through the state machine
            if (inputs->target_state >= NODE_STATE_READY &&
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
            r.actions |= CONTROL_ACTION_ATTEMPT_RECOVERY;
            break;

        default:
            break;
    }

    return r;
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
