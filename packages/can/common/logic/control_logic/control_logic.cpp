/**
 * @file control_logic.cpp
 * @brief Pure decision logic for the Control ESP32 — no hardware dependencies.
 */

#include "control_logic.h"
#include "can_protocol.h"

/**
 * @brief Clamp Safety's target state to the valid command set.
 *
 * Safety only sends NOT_READY, READY, ENABLE, or ACTIVE as target
 * commands. Any other value (corruption, version mismatch) is treated
 * as NOT_READY for a safe retreat.
 *
 * @param target_state  Raw target state received from Safety heartbeat
 * @return Sanitized target state (one of NOT_READY/READY/ENABLE/ACTIVE)
 */
static node_state_t sanitize_target_state(node_state_t target_state)
{
	switch (target_state)
	{
	case NODE_STATE_NOT_READY:
	case NODE_STATE_READY:
	case NODE_STATE_ENABLE:
	case NODE_STATE_ACTIVE:
		return target_state;
	default:
		return NODE_STATE_NOT_READY;
	}
}

/**
 * @brief Check whether the sanitized target state requests autonomous enable.
 *
 * Returns true when Safety has commanded ENABLE or ACTIVE, indicating
 * that Control should begin (or continue) the enable sequence.
 *
 * @param target_state  Sanitized target state from Safety
 * @return true if target is ENABLE or ACTIVE, false otherwise
 */
static bool target_requests_enable(node_state_t target_state)
{
	return (target_state == NODE_STATE_ENABLE || target_state == NODE_STATE_ACTIVE);
}

/**
 * @brief Determine READY or NOT_READY based on current preconditions.
 *
 * Evaluates F/R gear position, pedal state, and fault status to decide
 * whether Control can report READY.  Optionally returns the bitmask of
 * precondition failures for diagnostic logging.
 *
 * @param fault_code            Current active fault code (NODE_FAULT_NONE if clear)
 * @param inputs                Sensor and state inputs for this control tick
 * @param out_precondition_fail [out] If non-NULL, receives the precondition failure bitmask
 * @return NODE_STATE_READY if all preconditions pass, NODE_STATE_NOT_READY otherwise
 */
static node_state_t readiness_state_from_inputs(node_fault_t fault_code, const control_inputs_t *inputs,
                                                precondition_fail_t *out_precondition_fail)
{
	precondition_inputs_t pre = {
		.fr_state = inputs->fr_state,
		.pedal_pressed = inputs->pedal_pressed,
		.pedal_rearmed = inputs->pedal_rearmed,
		.fault_code = fault_code,
	};
	precondition_fail_t fail = control_check_preconditions(&pre);
	if (out_precondition_fail)
		*out_precondition_fail = fail;
	return (fail == PRECONDITION_OK) ? NODE_STATE_READY : NODE_STATE_NOT_READY;
}

precondition_fail_t control_check_preconditions(const precondition_inputs_t *inputs)
{
	if (!inputs)
		return PRECONDITION_FAIL_ALL;

	precondition_fail_t fail = PRECONDITION_OK;

	if (inputs->fr_state == FR_STATE_REVERSE || inputs->fr_state == FR_STATE_INVALID)
		fail |= PRECONDITION_FAIL_FR_IN_REVERSE;
	if (inputs->pedal_pressed)
		fail |= PRECONDITION_FAIL_PEDAL_PRESSED;
	if (!inputs->pedal_rearmed)
		fail |= PRECONDITION_FAIL_PEDAL_NOT_REARMED;
	if (inputs->fault_code != NODE_FAULT_NONE)
		fail |= PRECONDITION_FAIL_ACTIVE_FAULT;

	return fail;
}

throttle_slew_result_t control_compute_throttle_slew(const throttle_slew_inputs_t *inputs)
{
	throttle_slew_result_t r = {.new_level = 0, .changed = false};

	if (!inputs)
		return r;

	r.new_level = inputs->current;

	if (inputs->current == inputs->target)
		return r;

	uint32_t elapsed = inputs->now_ms - inputs->last_change_ms;
	if (elapsed >= inputs->slew_interval_ms)
	{
		if (inputs->current < inputs->target)
			r.new_level = inputs->current + 1;
		else
			r.new_level = inputs->current - 1;
		// Clamp to valid throttle range
		if (r.new_level < THROTTLE_LEVEL_MIN)
			r.new_level = THROTTLE_LEVEL_MIN;
		if (r.new_level > THROTTLE_LEVEL_MAX)
			r.new_level = THROTTLE_LEVEL_MAX;
		r.changed = (r.new_level != inputs->current);
	}

	return r;
}

/**
 * @brief Zero throttle and reset stepper dedup trackers to safe defaults.
 *
 * Called when leaving ACTIVE or entering any non-driving state (OVERRIDE,
 * FAULT, retreat, etc.).  Sets throttle to zero and marks both stepper
 * dedup trackers as reset so the next ACTIVE entry re-sends position
 * commands unconditionally.
 *
 * @param r  [out] Step result to modify with safe output values
 */
static void apply_safe_outputs(control_step_result_t *r)
{
	r->throttle_level = 0;
	r->new_last_steering = STEPPER_DEDUP_RESET;
	r->new_last_braking = STEPPER_DEDUP_RESET;
}

/**
 * @brief Transition to OVERRIDE state with safe outputs.
 *
 * Sets the step result to OVERRIDE, applies safe actuator outputs
 * (zero throttle, reset stepper dedup), records the trigger reason,
 * and flags the TRIGGER_OVERRIDE action for the caller to execute.
 *
 * @param r       [out] Step result to modify
 * @param reason  Override trigger (pedal, F/R change, steering/braking error)
 */
static void enter_override(control_step_result_t *r, override_reason_t reason)
{
	r->actions |= CONTROL_ACTION_TRIGGER_OVERRIDE;
	r->override_reason = reason;
	r->new_state = NODE_STATE_OVERRIDE;
	apply_safe_outputs(r);
}

control_step_result_t control_compute_step(node_state_t current_state, node_fault_t current_fault,
                                           const control_inputs_t *inputs)
{
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
		.enable_work_done = false,
	};

	if (!inputs)
		return r;

	node_state_t target_state = sanitize_target_state(inputs->target_state);

	// Carry forward dedup trackers and timing by default
	r.new_last_steering = inputs->last_steering_sent;
	r.new_last_braking = inputs->last_braking_sent;
	r.throttle_level = inputs->throttle_current;
	r.throttle_change_ms = inputs->last_throttle_change_ms;
	r.enable_start_ms = inputs->enable_start_ms;
	r.enable_work_done = inputs->enable_work_done;

	// Check for motor fault from CAN RX task (one-shot)
	if (inputs->motor_fault_code != NODE_FAULT_NONE && current_fault == NODE_FAULT_NONE)
	{
		if (current_state == NODE_STATE_ACTIVE)
		{
			r.actions |= CONTROL_ACTION_DISABLE_AUTONOMY;
			r.disable_reason = CONTROL_DISABLE_REASON_MOTOR_FAULT;
		}
		else if (current_state == NODE_STATE_ENABLE)
		{
			r.actions |= CONTROL_ACTION_ABORT_ENABLE;
			r.abort_reason = CONTROL_ABORT_REASON_MOTOR_FAULT;
		}
		r.new_state = NODE_STATE_FAULT;
		r.new_fault_code = inputs->motor_fault_code;
		apply_safe_outputs(&r);
		return r;
	}

	// Check for F/R INVALID sensor reading (post-bypass only).
	// Pre-bypass (INIT/NOT_READY/READY), INVALID means "reverse" (buzzer
	// active, anti-arc can't conduct) — handled by the precondition check,
	// not as a fault.  Post-bypass (ENABLE/ACTIVE), the anti-arc switch is
	// readable, so INVALID is a genuine wiring fault.
	if (inputs->fr_state == FR_STATE_INVALID && current_fault != NODE_FAULT_SENSOR_INVALID &&
	    (current_state == NODE_STATE_ENABLE || current_state == NODE_STATE_ACTIVE))
	{
		if (current_state == NODE_STATE_ACTIVE)
		{
			r.actions |= CONTROL_ACTION_DISABLE_AUTONOMY;
			r.disable_reason = CONTROL_DISABLE_REASON_SENSOR_INVALID;
		}
		else if (current_state == NODE_STATE_ENABLE)
		{
			r.actions |= CONTROL_ACTION_ABORT_ENABLE;
			r.abort_reason = CONTROL_ABORT_REASON_SENSOR_INVALID;
		}
		r.new_state = NODE_STATE_FAULT;
		r.new_fault_code = NODE_FAULT_SENSOR_INVALID;
		apply_safe_outputs(&r);
		return r;
	}

	switch (current_state)
	{
	case NODE_STATE_INIT:
		if ((inputs->now_ms - inputs->boot_start_ms) >= inputs->init_dwell_ms)
		{
			r.new_state = readiness_state_from_inputs(current_fault, inputs, &r.precondition_fail);
		}
		else
		{
			r.new_state = NODE_STATE_INIT;
		}
		break;

	case NODE_STATE_NOT_READY:
	{
		precondition_fail_t fail = PRECONDITION_OK;
		node_state_t ready_state = readiness_state_from_inputs(current_fault, inputs, &fail);
		r.precondition_fail = fail;
		r.new_state = ready_state;
		break;
	}

	case NODE_STATE_READY:
	{
		precondition_fail_t fail = PRECONDITION_OK;
		if (readiness_state_from_inputs(current_fault, inputs, &fail) != NODE_STATE_READY)
		{
			r.new_state = NODE_STATE_NOT_READY;
			r.precondition_fail = fail;
			break;
		}

		// Safety commands us to start enable sequence?
		if (target_requests_enable(target_state))
		{
			r.new_state = NODE_STATE_ENABLE;
			r.actions |= CONTROL_ACTION_START_ENABLE;
			r.enable_start_ms = inputs->now_ms;
			r.enable_work_done = false;
			r.throttle_level = 0;
			r.throttle_change_ms = inputs->now_ms;
		}
		break;
	}

	case NODE_STATE_ENABLE:
	{
		// Safety retreated? Abort enable.
		if (!target_requests_enable(target_state))
		{
			r.new_state = readiness_state_from_inputs(current_fault, inputs, &r.precondition_fail);
			r.actions |= CONTROL_ACTION_ABORT_ENABLE;
			r.abort_reason = CONTROL_ABORT_REASON_SAFETY_RETREAT;
			r.enable_work_done = false;
			break;
		}
		// Pedal pressed during enable? Abort.
		if (inputs->pedal_pressed)
		{
			r.new_state = readiness_state_from_inputs(current_fault, inputs, &r.precondition_fail);
			r.actions |= CONTROL_ACTION_ABORT_ENABLE;
			r.abort_reason = CONTROL_ABORT_REASON_PEDAL_PRESSED;
			r.enable_work_done = false;
			break;
		}
		// FR moved to reverse? Abort.
		// FR_INVALID during ENABLE is caught by the pre-switch guard above
		// (post-bypass wiring fault).  NEUTRAL is allowed (pre-bypass, FORWARD
		// and NEUTRAL are indistinguishable).
		if (inputs->fr_state == FR_STATE_REVERSE)
		{
			r.new_state = readiness_state_from_inputs(current_fault, inputs, &r.precondition_fail);
			r.actions |= CONTROL_ACTION_ABORT_ENABLE;
			r.abort_reason = CONTROL_ABORT_REASON_FR_IN_REVERSE;
			r.enable_work_done = false;
			break;
		}

		// Check if enable work is done (timer expired)
		if ((inputs->now_ms - inputs->enable_start_ms) >= inputs->enable_sequence_ms)
		{
			// Only fire COMPLETE_ENABLE action once per enable sequence
			if (!inputs->enable_work_done)
			{
				r.actions |= CONTROL_ACTION_COMPLETE_ENABLE;
			}
			r.enable_work_done = true;

			// If Safety already says ACTIVE, go directly to ACTIVE
			if (target_state == NODE_STATE_ACTIVE)
			{
				r.new_state = NODE_STATE_ACTIVE;
			}
			else
			{
				// Stay ENABLE, set enable_complete flag for heartbeat
				// Safety will advance us to ACTIVE once both nodes are ready
				r.new_state = NODE_STATE_ENABLE;
				r.heartbeat_flags |= HEARTBEAT_FLAG_ENABLE_COMPLETE;
			}
		}
		// else: stay ENABLE, timer not yet expired
		break;
	}

	case NODE_STATE_ACTIVE:
	{
		// Safety retreated? Disable actuators and return to readiness state.
		// This is NOT an override (human intervention) — Safety commanded
		// the retreat, so we do not emit OVERRIDE.
		if (target_state != NODE_STATE_ACTIVE)
		{
			r.actions |= CONTROL_ACTION_DISABLE_AUTONOMY;
			r.disable_reason = CONTROL_DISABLE_REASON_SAFETY_RETREAT;
			r.new_state = readiness_state_from_inputs(current_fault, inputs, &r.precondition_fail);
			apply_safe_outputs(&r);
			break;
		}
		// Check override conditions
		if (inputs->pedal_pressed)
		{
			enter_override(&r, OVERRIDE_REASON_PEDAL);
			break;
		}
		if (inputs->fr_state == FR_STATE_REVERSE)
		{
			enter_override(&r, OVERRIDE_REASON_FR_CHANGED);
			break;
		}
		// FR_INVALID in ACTIVE is caught by the pre-switch guard (wiring fault → FAULT).
		// FR_NEUTRAL in ACTIVE: cart is not in gear — zero throttle but stay ACTIVE
		// so steering/braking continue and the system resumes when FORWARD is engaged.
		// Steering position error (external force on steering column)
		if (inputs->steering_position_error)
		{
			enter_override(&r, OVERRIDE_REASON_STEERING);
			break;
		}
		// Braking position error (external force on brake pedal)
		if (inputs->braking_position_error)
		{
			enter_override(&r, OVERRIDE_REASON_BRAKING);
			break;
		}

		// Throttle slew — zero target when FR is NEUTRAL (cart not in gear)
		int8_t effective_throttle_target = (inputs->fr_state == FR_STATE_NEUTRAL) ? 0 : inputs->throttle_cmd;
		throttle_slew_inputs_t slew = {
			.current = inputs->throttle_current,
			.target = effective_throttle_target,
			.last_change_ms = inputs->last_throttle_change_ms,
			.now_ms = inputs->now_ms,
			.slew_interval_ms = inputs->throttle_slew_interval_ms,
		};
		throttle_slew_result_t slew_r = control_compute_throttle_slew(&slew);
		if (slew_r.changed)
		{
			r.throttle_level = slew_r.new_level;
			r.throttle_change_ms = inputs->now_ms;
			r.actions |= CONTROL_ACTION_APPLY_THROTTLE;
		}

		// Clamp steering/braking commands to safe envelopes before dedup.
		// If min == max == 0, envelope is not configured; force neutral (0)
		// instead of passing through raw planner commands.
		int32_t clamped_steering = inputs->steering_cmd;
		int16_t clamped_braking = inputs->braking_cmd;
		if (inputs->steering_min == 0 && inputs->steering_max == 0)
		{
			clamped_steering = 0;
		}
		else
		{
			clamped_steering = control_clamp_command(inputs->steering_cmd, inputs->steering_min, inputs->steering_max);
		}
		if (inputs->braking_min == 0 && inputs->braking_max == 0)
		{
			clamped_braking = 0;
		}
		else
		{
			clamped_braking = (int16_t)control_clamp_command(inputs->braking_cmd, inputs->braking_min, inputs->braking_max);
		}

		// Stepper dedup (uses clamped values)
		if (clamped_steering != inputs->last_steering_sent)
		{
			r.send_steering = true;
			r.steering_position = clamped_steering;
			r.new_last_steering = clamped_steering;
		}
		if (clamped_braking != inputs->last_braking_sent)
		{
			r.send_braking = true;
			r.braking_position = clamped_braking;
			r.new_last_braking = clamped_braking;
		}
		break;
	}

	case NODE_STATE_OVERRIDE:
		apply_safe_outputs(&r);

		// Recover to readiness state when override conditions clear.
		// FR must not be in REVERSE or INVALID (same as precondition check).
		if (inputs->fr_state != FR_STATE_REVERSE && inputs->fr_state != FR_STATE_INVALID && inputs->pedal_rearmed)
		{
			r.new_state = readiness_state_from_inputs(current_fault, inputs, &r.precondition_fail);
			r.override_reason = OVERRIDE_REASON_NONE;
		}
		break;

	case NODE_STATE_FAULT:
		apply_safe_outputs(&r);
		{
			bool fault_cleared = false;
			switch (current_fault)
			{
			case NODE_FAULT_NONE:
				fault_cleared = true;
				break;
			case NODE_FAULT_MOTOR_COMM:
				fault_cleared = (inputs->motor_fault_code == NODE_FAULT_NONE);
				break;
			case NODE_FAULT_SENSOR_INVALID:
				fault_cleared = (inputs->fr_state != FR_STATE_INVALID);
				break;
			default:
				fault_cleared = false;
				break;
			}

			if (fault_cleared)
			{
				r.new_fault_code = NODE_FAULT_NONE;
				r.new_state = readiness_state_from_inputs(NODE_FAULT_NONE, inputs, &r.precondition_fail);
			}
			else
			{
				r.actions |= CONTROL_ACTION_ATTEMPT_RECOVERY;
			}
		}
		break;

	default:
		r.new_state = NODE_STATE_FAULT;
		r.new_fault_code = NODE_FAULT_GENERAL;
		r.actions |= CONTROL_ACTION_DISABLE_AUTONOMY;
		r.disable_reason = CONTROL_DISABLE_REASON_NONE;
		apply_safe_outputs(&r);
		break;
	}

	return r;
}

int32_t control_clamp_command(int32_t value, int32_t min, int32_t max)
{
	if (value < min)
		return min;
	if (value > max)
		return max;
	return value;
}
