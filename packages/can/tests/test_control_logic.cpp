/**
 * @file test_control_logic.cpp
 * @brief Unit tests for Control pure state machine logic.
 */

#include "test_harness.h"
#include <stdbool.h>
#include <stdint.h>

#include "control_logic.h"
#include "can_protocol.h"

static control_inputs_t default_inputs(void)
{
	control_inputs_t in = {
		.target_state = NODE_STATE_READY,
		.throttle_cmd = 0,
		.steering_cmd = 0,
		.braking_cmd = 0,
		.motor_fault_code = NODE_FAULT_NONE,
		.stop_flags = NODE_STOP_NONE,
		.fr_state = FR_STATE_FORWARD,
		.pedal_pressed = false,
		.pedal_rearmed = true,
		.steering_position_error = false,
		.braking_position_error = false,
		.now_ms = 1000,
		.boot_start_ms = 0,
		.init_dwell_ms = 500,
		.enable_start_ms = 0,
		.enable_sequence_ms = 200,
		.enable_work_done = false,
		.throttle_current = 0,
		.last_throttle_change_ms = 0,
		.throttle_slew_interval_ms = 100,
		.throttle_slew_step = 1,
		.throttle_min = 0,
		.throttle_max = 255,
		.last_steering_sent = STEPPER_DEDUP_RESET_STEERING,
		.last_braking_sent = STEPPER_DEDUP_RESET_BRAKING,
		.steering_min = -3000,
		.steering_max = 3000,
		.braking_min = -3000,
		.braking_max = 3000,
	};
	return in;
}

// ============================================================================
// Original state transition tests (16)
// ============================================================================

static void test_init_to_ready_when_preconditions_pass(void)
{
	control_inputs_t in = default_inputs();
	control_step_result_t r = control_compute_step(NODE_STATE_INIT, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_READY);
}

static void test_init_to_not_ready_when_preconditions_fail(void)
{
	control_inputs_t in = default_inputs();
	in.pedal_pressed = true;
	control_step_result_t r = control_compute_step(NODE_STATE_INIT, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert((r.precondition_fail & PRECONDITION_FAIL_PEDAL_PRESSED) != 0);
}

static void test_not_ready_to_ready_when_clear(void)
{
	control_inputs_t in = default_inputs();
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_READY);
	assert(r.precondition_fail == PRECONDITION_OK);
}

static void test_not_ready_stays_when_pedal_pressed(void)
{
	control_inputs_t in = default_inputs();
	in.pedal_pressed = true;
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert((r.precondition_fail & PRECONDITION_FAIL_PEDAL_PRESSED) != 0);
}

static void test_ready_to_enable_on_safety_target(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ENABLE;
	control_step_result_t r = control_compute_step(NODE_STATE_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ENABLE);
	assert((r.actions & CONTROL_ACTION_START_ENABLE) != 0);
}

static void test_ready_to_not_ready_when_precondition_drops(void)
{
	control_inputs_t in = default_inputs();
	in.pedal_pressed = true;
	control_step_result_t r = control_compute_step(NODE_STATE_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert((r.precondition_fail & PRECONDITION_FAIL_PEDAL_PRESSED) != 0);
}

static void test_enable_to_active_when_complete_and_target_active(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.enable_start_ms = 100;
	in.now_ms = 400;
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert((r.actions & CONTROL_ACTION_COMPLETE_ENABLE) != 0);
}

static void test_enable_stays_enable_with_complete_flag(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ENABLE;
	in.enable_start_ms = 100;
	in.now_ms = 400;
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ENABLE);
	assert((r.status_flags & NODE_STATUS_FLAG_ENABLE_COMPLETE) != 0);
}

static void test_enable_abort_to_not_ready_on_pedal(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ENABLE;
	in.pedal_pressed = true;
	in.pedal_rearmed = false;
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert((r.actions & CONTROL_ACTION_ABORT_ENABLE) != 0);
	assert(r.abort_reason == CONTROL_ABORT_REASON_PEDAL_PRESSED);
}

static void test_active_safety_retreat_to_not_ready(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_NOT_READY;
	in.pedal_pressed = true;
	in.pedal_rearmed = false;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) != 0);
}

static void test_active_override_on_pedal(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.pedal_pressed = true;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert((r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE) != 0);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
	assert((r.new_stop_flags & NODE_STOP_OPERATOR_THROTTLE) != 0);
}

static void test_override_recovery_to_ready(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_READY;
	in.pedal_pressed = false;
	in.pedal_rearmed = true;
	in.fr_state = FR_STATE_FORWARD;
	in.stop_flags = NODE_STOP_OPERATOR_THROTTLE;
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_READY);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
	assert(r.new_stop_flags == NODE_STOP_NONE);
}

static void test_fault_clear_to_not_ready(void)
{
	control_inputs_t in = default_inputs();
	in.pedal_pressed = true;
	in.pedal_rearmed = false;
	in.motor_fault_code = NODE_FAULT_NONE;
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_CONTROL_MOTOR_COMM, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
}

static void test_fault_clear_to_ready(void)
{
	control_inputs_t in = default_inputs();
	in.motor_fault_code = NODE_FAULT_NONE;
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_CONTROL_MOTOR_COMM, &in);
	assert(r.new_state == NODE_STATE_READY);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
}

static void test_fault_injection_from_not_ready(void)
{
	control_inputs_t in = default_inputs();
	in.motor_fault_code = NODE_FAULT_CONTROL_MOTOR_COMM;
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_CONTROL_MOTOR_COMM);
}

static void test_unknown_state_defaults_to_fault(void)
{
	control_inputs_t in = default_inputs();
	control_step_result_t r = control_compute_step(0xFF, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_GENERAL);
}

// ============================================================================
// Precondition checker (standalone function)
// ============================================================================

static void test_preconditions_null_returns_all_fail(void)
{
	precondition_fail_t fail = control_check_preconditions(NULL);
	assert(fail == PRECONDITION_FAIL_ALL);
}

static void test_preconditions_fr_reverse_blocks(void)
{
	precondition_inputs_t pre = {
		.fr_state = FR_STATE_REVERSE,
		.pedal_pressed = false,
		.pedal_rearmed = true,
		.fault_code = NODE_FAULT_NONE,
	};
	precondition_fail_t fail = control_check_preconditions(&pre);
	assert((fail & PRECONDITION_FAIL_FR_IN_REVERSE) != 0);
	assert((fail & PRECONDITION_FAIL_PEDAL_PRESSED) == 0);
}

static void test_preconditions_fr_invalid_blocks(void)
{
	precondition_inputs_t pre = {
		.fr_state = FR_STATE_INVALID,
		.pedal_pressed = false,
		.pedal_rearmed = true,
		.fault_code = NODE_FAULT_NONE,
	};
	precondition_fail_t fail = control_check_preconditions(&pre);
	assert((fail & PRECONDITION_FAIL_FR_IN_REVERSE) != 0);
}

static void test_preconditions_fr_neutral_ok(void)
{
	// NEUTRAL should NOT fail preconditions (pre-bypass, FORWARD reads as NEUTRAL)
	precondition_inputs_t pre = {
		.fr_state = FR_STATE_NEUTRAL,
		.pedal_pressed = false,
		.pedal_rearmed = true,
		.fault_code = NODE_FAULT_NONE,
	};
	precondition_fail_t fail = control_check_preconditions(&pre);
	assert((fail & PRECONDITION_FAIL_FR_IN_REVERSE) == 0);
	assert(fail == PRECONDITION_OK);
}

static void test_preconditions_pedal_not_rearmed(void)
{
	precondition_inputs_t pre = {
		.fr_state = FR_STATE_FORWARD,
		.pedal_pressed = false,
		.pedal_rearmed = false,
		.fault_code = NODE_FAULT_NONE,
	};
	precondition_fail_t fail = control_check_preconditions(&pre);
	assert(fail == PRECONDITION_FAIL_PEDAL_NOT_REARMED);
}

static void test_preconditions_active_fault(void)
{
	precondition_inputs_t pre = {
		.fr_state = FR_STATE_FORWARD,
		.pedal_pressed = false,
		.pedal_rearmed = true,
		.fault_code = NODE_FAULT_CONTROL_MOTOR_COMM,
	};
	precondition_fail_t fail = control_check_preconditions(&pre);
	assert(fail == PRECONDITION_FAIL_ACTIVE_FAULT);
}

static void test_preconditions_multiple_combine(void)
{
	precondition_inputs_t pre = {
		.fr_state = FR_STATE_REVERSE,
		.pedal_pressed = true,
		.pedal_rearmed = false,
		.fault_code = NODE_FAULT_CONTROL_MOTOR_COMM,
	};
	precondition_fail_t fail = control_check_preconditions(&pre);
	assert(fail == (PRECONDITION_FAIL_FR_IN_REVERSE | PRECONDITION_FAIL_PEDAL_PRESSED |
	                PRECONDITION_FAIL_PEDAL_NOT_REARMED | PRECONDITION_FAIL_ACTIVE_FAULT));
	assert((fail & PRECONDITION_FAIL_FR_IN_REVERSE) != 0);
}

// ============================================================================
// Throttle slew (standalone function)
// ============================================================================

static void test_slew_at_target_no_change(void)
{
	throttle_slew_inputs_t slew = {
		.current = 3,
		.target = 3,
		.last_change_ms = 0,
		.now_ms = 1000,
		.slew_interval_ms = 100,
		.slew_step = 1,
	};
	throttle_slew_result_t r = control_compute_throttle_slew(&slew);
	assert(r.new_level == 3);
	assert(r.changed == false);
}

static void test_slew_step_up(void)
{
	throttle_slew_inputs_t slew = {
		.current = 3,
		.target = 5,
		.last_change_ms = 0,
		.now_ms = 200,
		.slew_interval_ms = 100,
		.slew_step = 1,
	};
	throttle_slew_result_t r = control_compute_throttle_slew(&slew);
	assert(r.new_level == 4);
	assert(r.changed == true);
}

static void test_slew_step_down(void)
{
	throttle_slew_inputs_t slew = {
		.current = 5,
		.target = 2,
		.last_change_ms = 0,
		.now_ms = 200,
		.slew_interval_ms = 100,
		.slew_step = 1,
	};
	throttle_slew_result_t r = control_compute_throttle_slew(&slew);
	assert(r.new_level == 4);
	assert(r.changed == true);
}

static void test_slew_rate_limited(void)
{
	throttle_slew_inputs_t slew = {
		.current = 3,
		.target = 7,
		.last_change_ms = 0,
		.now_ms = 50,
		.slew_interval_ms = 100,
		.slew_step = 1,
	};
	throttle_slew_result_t r = control_compute_throttle_slew(&slew);
	assert(r.new_level == 3);
	assert(r.changed == false);
}

static void test_slew_null_inputs(void)
{
	throttle_slew_result_t r = control_compute_throttle_slew(NULL);
	assert(r.new_level == 0);
	assert(r.changed == false);
}

static void test_slew_timer_overflow(void)
{
	// last_change near UINT32_MAX, now wrapped past 0
	// elapsed = 50 - (UINT32_MAX - 50) = 100 (unsigned wrap)
	throttle_slew_inputs_t slew = {
		.current = 2,
		.target = 5,
		.last_change_ms = UINT32_MAX - 50,
		.now_ms = 50,
		.slew_interval_ms = 100,
		.slew_step = 1,
	};
	throttle_slew_result_t r = control_compute_throttle_slew(&slew);
	assert(r.new_level == 3);
	assert(r.changed == true);
}

// ============================================================================
// Command envelope clamping
// ============================================================================

static void test_clamp_below_min(void)
{
	assert(control_clamp_command(-5000, -3000, 3000) == -3000);
}

static void test_clamp_above_max(void)
{
	assert(control_clamp_command(5000, -3000, 3000) == 3000);
}

static void test_clamp_in_range(void)
{
	assert(control_clamp_command(1500, -3000, 3000) == 1500);
}

static void test_clamp_misconfigured_returns_neutral(void)
{
	// When min > max (misconfigured envelope), should force neutral (0)
	assert(control_clamp_command(50, 200, 100) == 0);
	assert(control_clamp_command(0, 100, -100) == 0);
	assert(control_clamp_command(-50, 50, -50) == 0);
}

// ============================================================================
// INIT state edge cases
// ============================================================================

static void test_init_stays_when_dwell_not_expired(void)
{
	control_inputs_t in = default_inputs();
	in.boot_start_ms = 800;
	in.init_dwell_ms = 500;
	in.now_ms = 1000; // elapsed = 200 < 500
	control_step_result_t r = control_compute_step(NODE_STATE_INIT, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_INIT);
}

static void test_init_dwell_timer_overflow(void)
{
	control_inputs_t in = default_inputs();
	in.boot_start_ms = UINT32_MAX - 100;
	in.init_dwell_ms = 500;
	in.now_ms = 500; // elapsed = 500 - (MAX-100) = 601 (unsigned wrap) >= 500
	control_step_result_t r = control_compute_step(NODE_STATE_INIT, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_READY);
}

// ============================================================================
// ENABLE abort scenarios
// ============================================================================

static void test_enable_abort_safety_retreat(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_NOT_READY; // safety retreated
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert((r.actions & CONTROL_ACTION_ABORT_ENABLE) != 0);
	assert(r.abort_reason == CONTROL_ABORT_REASON_SAFETY_RETREAT);
	assert(r.enable_work_done == false);
}

static void test_enable_abort_fr_reverse(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ENABLE;
	in.fr_state = FR_STATE_REVERSE;
	in.pedal_pressed = false;
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert((r.actions & CONTROL_ACTION_ABORT_ENABLE) != 0);
	assert(r.abort_reason == CONTROL_ABORT_REASON_FR_IN_REVERSE);
}

static void test_enable_neutral_does_not_abort(void)
{
	// NEUTRAL during ENABLE should NOT abort — pre-bypass, FORWARD reads as NEUTRAL
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.fr_state = FR_STATE_NEUTRAL;
	in.enable_start_ms = 100;
	in.now_ms = 400; // timer expired
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert((r.actions & CONTROL_ACTION_ABORT_ENABLE) == 0);
}

static void test_enable_stays_when_timer_not_expired(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ENABLE;
	in.enable_start_ms = 900;
	in.enable_sequence_ms = 200;
	in.now_ms = 1000; // elapsed = 100 < 200
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ENABLE);
	assert(r.actions == CONTROL_ACTION_NONE);
}

static void test_enable_complete_fires_once(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ENABLE;
	in.enable_start_ms = 100;
	in.now_ms = 400;
	in.enable_work_done = true; // already fired once
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ENABLE);
	assert((r.actions & CONTROL_ACTION_COMPLETE_ENABLE) == 0); // should NOT fire again
	assert((r.status_flags & NODE_STATUS_FLAG_ENABLE_COMPLETE) != 0);
}

static void test_enable_timer_exact_boundary(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.enable_start_ms = 100;
	in.enable_sequence_ms = 200;
	in.now_ms = 300; // elapsed = 200 == 200, should pass (>=)
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert((r.actions & CONTROL_ACTION_COMPLETE_ENABLE) != 0);
}

// ============================================================================
// ACTIVE override variants
// ============================================================================

static void test_active_override_fr_changed(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.fr_state = FR_STATE_REVERSE;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert((r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE) != 0);
	assert(r.override_reason == OVERRIDE_REASON_REVERSE);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
	assert((r.new_stop_flags & NODE_STOP_OPERATOR_REVERSE) != 0);
}

static void test_active_neutral_zeros_throttle(void)
{
	// NEUTRAL in ACTIVE: stay ACTIVE but throttle target forced to 0
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.fr_state = FR_STATE_NEUTRAL;
	in.throttle_cmd = 5;
	in.throttle_current = 3;
	in.last_throttle_change_ms = 0;
	in.now_ms = 1000;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	// Throttle should slew toward 0, not toward 5
	assert((r.actions & CONTROL_ACTION_APPLY_THROTTLE) != 0);
	assert(r.throttle_level == 2); // current=3, target=0, slew down by 1
}

static void test_active_neutral_at_zero_no_change(void)
{
	// NEUTRAL in ACTIVE with throttle already at 0: no throttle action
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.fr_state = FR_STATE_NEUTRAL;
	in.throttle_cmd = 5;
	in.throttle_current = 0;
	in.last_throttle_change_ms = 0;
	in.now_ms = 1000;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert((r.actions & CONTROL_ACTION_APPLY_THROTTLE) == 0);
	assert(r.throttle_level == 0);
}

static void test_active_override_steering_error(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.steering_position_error = true;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert((r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE) != 0);
	assert(r.override_reason == OVERRIDE_REASON_STEERING);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
	assert((r.new_stop_flags & NODE_STOP_OPERATOR_STEER) != 0);
}

static void test_active_override_braking_error(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.braking_position_error = true;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert((r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE) != 0);
	assert(r.override_reason == OVERRIDE_REASON_BRAKING);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
	assert((r.new_stop_flags & NODE_STOP_OPERATOR_BRAKE) != 0);
}

static void test_active_safety_retreat_priority_over_pedal(void)
{
	// When target != ACTIVE AND pedal_pressed, safety retreat wins (checked first)
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_NOT_READY;
	in.pedal_pressed = true;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	// Should get DISABLE_AUTONOMY (safety retreat), NOT TRIGGER_OVERRIDE
	assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) != 0);
	assert((r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE) == 0);
	assert(r.disable_reason == CONTROL_DISABLE_REASON_SAFETY_RETREAT);
}

// ============================================================================
// ACTIVE throttle + steering
// ============================================================================

static void test_active_throttle_slew_applies(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.throttle_cmd = 5;
	in.throttle_current = 0;
	in.last_throttle_change_ms = 0;
	in.now_ms = 1000; // elapsed >> slew_interval
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert((r.actions & CONTROL_ACTION_APPLY_THROTTLE) != 0);
	assert(r.throttle_level == 1); // steps by 1 (slew)
	assert(r.throttle_change_ms == 1000);
}

static void test_active_throttle_at_target_no_action(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.throttle_cmd = 3;
	in.throttle_current = 3;
	in.now_ms = 1000;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert((r.actions & CONTROL_ACTION_APPLY_THROTTLE) == 0);
	assert(r.throttle_level == 3); // carried forward, unchanged
}

static void test_active_throttle_clamps_to_envelope(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.throttle_cmd = 500; // above envelope max
	in.throttle_current = 255;
	in.now_ms = 1000;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert((r.actions & CONTROL_ACTION_APPLY_THROTTLE) == 0);
	assert(r.throttle_level == 255);
}

static void test_active_throttle_unconfigured_forces_neutral(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.throttle_cmd = 100;
	in.throttle_current = 50;
	in.now_ms = 1000;
	in.throttle_min = 0;
	in.throttle_max = 0;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert((r.actions & CONTROL_ACTION_APPLY_THROTTLE) != 0);
	assert(r.throttle_level == 0);
}

static void test_active_steering_dedup_same_skips(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.steering_cmd = 1500;
	in.last_steering_sent = 1500; // same as cmd after clamping
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert(r.send_steering == false);
}

static void test_active_steering_changed_sends(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.steering_cmd = 1500;
	in.last_steering_sent = 1000; // different
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert(r.send_steering == true);
	assert(r.steering_position == 1500);
	assert(r.new_last_steering == 1500);
}

static void test_active_braking_dedup_reset_always_sends(void)
{
	// After exiting override/fault, dedup tracker is STEPPER_DEDUP_RESET_BRAKING.
	// Even a zero command should be sent since it doesn't match the sentinel.
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.braking_cmd = 0;
	in.last_braking_sent = STEPPER_DEDUP_RESET_BRAKING;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.send_braking == true);
	assert(r.braking_position == 0);
	assert(r.new_last_braking == 0);
}

static void test_active_envelope_unconfigured_forces_neutral(void)
{
	// When min == max == 0, envelope is unconfigured; command forced to 0
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.steering_cmd = 2000;
	in.steering_min = 0;
	in.steering_max = 0;
	in.last_steering_sent = STEPPER_DEDUP_RESET_STEERING;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.send_steering == true);
	assert(r.steering_position == 0); // forced neutral
}

static void test_active_envelope_clamps_command(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.steering_cmd = 5000; // exceeds max of 3000
	in.steering_min = -3000;
	in.steering_max = 3000;
	in.last_steering_sent = STEPPER_DEDUP_RESET_STEERING;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.send_steering == true);
	assert(r.steering_position == 3000); // clamped to max
}

// ============================================================================
// Motor fault injection (pre-switch)
// ============================================================================

static void test_motor_fault_from_active(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.motor_fault_code = NODE_FAULT_CONTROL_MOTOR_COMM;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_CONTROL_MOTOR_COMM);
	assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) != 0);
	assert(r.disable_reason == CONTROL_DISABLE_REASON_MOTOR_FAULT);
	assert(r.throttle_level == 0); // safe outputs applied
}

static void test_motor_fault_from_enable(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ENABLE;
	in.motor_fault_code = NODE_FAULT_CONTROL_MOTOR_COMM;
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_CONTROL_MOTOR_COMM);
	assert((r.actions & CONTROL_ACTION_ABORT_ENABLE) != 0);
	assert(r.abort_reason == CONTROL_ABORT_REASON_MOTOR_FAULT);
}

static void test_motor_fault_from_ready(void)
{
	control_inputs_t in = default_inputs();
	in.motor_fault_code = NODE_FAULT_CONTROL_MOTOR_COMM;
	control_step_result_t r = control_compute_step(NODE_STATE_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_CONTROL_MOTOR_COMM);
	// No DISABLE or ABORT actions — nothing was enabled
	assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) == 0);
	assert((r.actions & CONTROL_ACTION_ABORT_ENABLE) == 0);
}

static void test_motor_fault_ignored_when_already_faulted(void)
{
	control_inputs_t in = default_inputs();
	in.motor_fault_code = NODE_FAULT_CONTROL_MOTOR_COMM;
	// Already in a MOTOR_COMM fault — should NOT re-trigger
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_CONTROL_MOTOR_COMM, &in);
	// Should stay in NOT_READY with existing fault code, not re-enter
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_CONTROL_MOTOR_COMM);
	assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) == 0);
	assert((r.actions & CONTROL_ACTION_ABORT_ENABLE) == 0);
}

// ============================================================================
// FR_INVALID sensor fault injection (pre-switch)
// ============================================================================

static void test_fr_invalid_from_active(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.fr_state = FR_STATE_INVALID;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_CONTROL_SENSOR_INVALID);
	assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) != 0);
	assert(r.disable_reason == CONTROL_DISABLE_REASON_SENSOR_INVALID);
}

static void test_fr_invalid_from_enable(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ENABLE;
	in.fr_state = FR_STATE_INVALID;
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
	assert((r.actions & CONTROL_ACTION_ABORT_ENABLE) != 0);
	assert(r.abort_reason == CONTROL_ABORT_REASON_FR_IN_REVERSE);
}

static void test_fr_invalid_from_not_ready_no_fault(void)
{
	// Pre-bypass: FR_INVALID means reverse, not a wiring fault.
	// Should stay NOT_READY (blocked by precondition), NOT enter FAULT.
	control_inputs_t in = default_inputs();
	in.fr_state = FR_STATE_INVALID;
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_NONE); // no fault triggered
	assert((r.precondition_fail & PRECONDITION_FAIL_FR_IN_REVERSE) != 0);
}

static void test_fr_invalid_from_ready_no_fault(void)
{
	// Pre-bypass: FR_INVALID in READY drops to NOT_READY, no fault.
	control_inputs_t in = default_inputs();
	in.fr_state = FR_STATE_INVALID;
	control_step_result_t r = control_compute_step(NODE_STATE_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
}

static void test_fr_invalid_no_retrigger(void)
{
	control_inputs_t in = default_inputs();
	in.fr_state = FR_STATE_INVALID;
	// Already in SENSOR_INVALID fault — should NOT re-trigger pre-switch path
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_CONTROL_SENSOR_INVALID, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_CONTROL_SENSOR_INVALID);
	assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) == 0);
}

// ============================================================================
// OVERRIDE recovery edge cases
// ============================================================================

static void test_override_stays_when_pedal_not_rearmed(void)
{
	control_inputs_t in = default_inputs();
	in.fr_state = FR_STATE_FORWARD;
	in.pedal_rearmed = false; // not rearmed
	in.stop_flags = NODE_STOP_OPERATOR_THROTTLE;
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
	assert((r.new_stop_flags & NODE_STOP_OPERATOR_THROTTLE) != 0);
}

static void test_override_stays_when_fr_reverse(void)
{
	control_inputs_t in = default_inputs();
	in.fr_state = FR_STATE_REVERSE;
	in.pedal_rearmed = true;
	in.stop_flags = NODE_STOP_OPERATOR_REVERSE;
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
	assert((r.new_stop_flags & NODE_STOP_OPERATOR_REVERSE) != 0);
}

static void test_override_recovers_when_fr_neutral(void)
{
	// NEUTRAL should allow override recovery (not blocked like REVERSE)
	control_inputs_t in = default_inputs();
	in.fr_state = FR_STATE_NEUTRAL;
	in.pedal_rearmed = true;
	in.stop_flags = NODE_STOP_OPERATOR_REVERSE;
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_READY); // NEUTRAL passes preconditions
	assert(r.new_fault_flags == NODE_FAULT_NONE);
	assert((r.new_stop_flags & NODE_STOP_OPERATOR_REVERSE) == 0);
}

// ============================================================================
// FAULT recovery
// ============================================================================

static void test_fault_motor_comm_clears_on_recovery(void)
{
	control_inputs_t in = default_inputs();
	in.motor_fault_code = NODE_FAULT_NONE; // fault condition cleared
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_CONTROL_MOTOR_COMM, &in);
	assert(r.new_state == NODE_STATE_READY);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
	assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) == 0);
}

static void test_fault_sensor_invalid_clears_when_fr_valid(void)
{
	control_inputs_t in = default_inputs();
	in.fr_state = FR_STATE_FORWARD; // no longer INVALID
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_CONTROL_SENSOR_INVALID, &in);
	assert(r.new_state == NODE_STATE_READY);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
}

static void test_fault_unknown_type_stays_in_fault(void)
{
	// An unrecognized fault code (e.g. NODE_FAULT_GENERAL) should not clear
	control_inputs_t in = default_inputs();
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_GENERAL, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_GENERAL);
	assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) == 0);
}

// ============================================================================
// Target state sanitization
// ============================================================================

static void test_target_state_sanitized(void)
{
	// Invalid target (0xFF) should be treated as NOT_READY
	control_inputs_t in = default_inputs();
	in.target_state = 0xFF;
	// From ACTIVE, target != ACTIVE triggers safety retreat
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) != 0);
	assert(r.disable_reason == CONTROL_DISABLE_REASON_SAFETY_RETREAT);
}

// ============================================================================
// Safe outputs
// ============================================================================

static void test_override_applies_safe_outputs(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.pedal_pressed = true;
	in.throttle_current = 5;
	in.last_steering_sent = 1000;
	in.last_braking_sent = 500;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.throttle_level == 0);
	assert(r.new_last_steering == STEPPER_DEDUP_RESET_STEERING);
	assert(r.new_last_braking == STEPPER_DEDUP_RESET_BRAKING);
}

// ============================================================================
// NULL inputs guard
// ============================================================================

static void test_null_inputs_preserves_current_state(void)
{
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, NULL);
	// NULL inputs returns zero-init struct with current_state/fault carried forward
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert(r.new_fault_flags == NODE_FAULT_NONE);
	assert(r.actions == CONTROL_ACTION_NONE);
}

// ============================================================================
// Simultaneous faults and multi-step slew
// ============================================================================

static void test_active_simultaneous_steering_braking_errors(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.steering_position_error = true;
	in.braking_position_error = true;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert((r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE) != 0);
	// At least one override reason should be set
	assert(r.override_reason != OVERRIDE_REASON_NONE);
	assert(r.throttle_level == 0); // safe outputs
}

static void test_enable_motor_fault_before_timer_expires(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ENABLE;
	in.enable_start_ms = 900;
	in.enable_sequence_ms = 200;
	in.now_ms = 1000; // timer NOT expired (elapsed=100 < 200)
	in.motor_fault_code = NODE_FAULT_CONTROL_MOTOR_COMM;
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_CONTROL_MOTOR_COMM);
	assert((r.actions & CONTROL_ACTION_ABORT_ENABLE) != 0);
	assert(r.abort_reason == CONTROL_ABORT_REASON_MOTOR_FAULT);
}

static void test_slew_multi_step(void)
{
	throttle_slew_inputs_t slew = {
		.current = 0,
		.target = 50,
		.last_change_ms = 0,
		.now_ms = 200,
		.slew_interval_ms = 100,
		.slew_step = 12, // max 12 steps per interval (production value)
	};
	throttle_slew_result_t r = control_compute_throttle_slew(&slew);
	assert(r.new_level == 12); // steps by 12 (not 1)
	assert(r.changed == true);
}

static void test_slew_step_defaults_when_zero(void)
{
	throttle_slew_inputs_t slew = {
		.current = 0,
		.target = 10,
		.last_change_ms = 0,
		.now_ms = 200,
		.slew_interval_ms = 100,
		.slew_step = 0, // should default to 1
	};
	throttle_slew_result_t r = control_compute_throttle_slew(&slew);
	assert(r.new_level == 1);
	assert(r.changed == true);
}

static void test_slew_step_defaults_when_negative(void)
{
	throttle_slew_inputs_t slew = {
		.current = 5,
		.target = 10,
		.last_change_ms = 0,
		.now_ms = 200,
		.slew_interval_ms = 100,
		.slew_step = -5, // should default to 1
	};
	throttle_slew_result_t r = control_compute_throttle_slew(&slew);
	assert(r.new_level == 6);
	assert(r.changed == true);
}

static void test_slew_multi_step_clamps_to_target(void)
{
	// When step > distance to target, should clamp to target
	throttle_slew_inputs_t slew = {
		.current = 5,
		.target = 8,
		.last_change_ms = 0,
		.now_ms = 200,
		.slew_interval_ms = 100,
		.slew_step = 12,
	};
	throttle_slew_result_t r = control_compute_throttle_slew(&slew);
	assert(r.new_level == 8); // clamped to target, not 5+12=17
	assert(r.changed == true);
}

static void test_active_fr_neutral_while_slewing_up(void)
{
	// FR goes NEUTRAL while throttle is slewing up — throttle should target 0
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.fr_state = FR_STATE_NEUTRAL;
	in.throttle_cmd = 100;
	in.throttle_current = 50;
	in.last_throttle_change_ms = 0;
	in.now_ms = 1000;
	in.throttle_slew_step = 1;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert((r.actions & CONTROL_ACTION_APPLY_THROTTLE) != 0);
	assert(r.throttle_level == 49); // slewing toward 0, not toward 100
}

static void test_enable_to_active_skips_complete_when_already_done(void)
{
	// enable_work_done=true + target=ACTIVE: transition to ACTIVE without
	// re-firing COMPLETE_ENABLE (work was already done in a prior tick)
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.enable_start_ms = 100;
	in.enable_sequence_ms = 200;
	in.now_ms = 400;            // timer expired
	in.enable_work_done = true; // already fired
	control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_ACTIVE);
	assert((r.actions & CONTROL_ACTION_COMPLETE_ENABLE) == 0); // must NOT re-fire
}

static void test_active_braking_envelope_unconfigured_forces_neutral(void)
{
	control_inputs_t in = default_inputs();
	in.target_state = NODE_STATE_ACTIVE;
	in.braking_cmd = 5000;
	in.braking_min = 0;
	in.braking_max = 0; // unconfigured
	in.last_braking_sent = STEPPER_DEDUP_RESET_BRAKING;
	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
	assert(r.send_braking == true);
	assert(r.braking_position == 0); // forced neutral
}

static void test_override_steering_stop_clears_when_error_resolves(void)
{
	control_inputs_t in = default_inputs();
	in.fr_state = FR_STATE_FORWARD;
	in.pedal_rearmed = true;
	in.steering_position_error = false; // error resolved
	in.stop_flags = NODE_STOP_OPERATOR_STEER;
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_READY);
	assert((r.new_stop_flags & NODE_STOP_OPERATOR_STEER) == 0); // cleared
}

static void test_override_braking_stop_clears_when_error_resolves(void)
{
	control_inputs_t in = default_inputs();
	in.fr_state = FR_STATE_FORWARD;
	in.pedal_rearmed = true;
	in.braking_position_error = false; // error resolved
	in.stop_flags = NODE_STOP_OPERATOR_BRAKE;
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_READY);
	assert((r.new_stop_flags & NODE_STOP_OPERATOR_BRAKE) == 0); // cleared
}

static void test_override_steering_stop_stays_when_error_persists(void)
{
	control_inputs_t in = default_inputs();
	in.steering_position_error = true; // still errored
	in.stop_flags = NODE_STOP_OPERATOR_STEER;
	control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert((r.new_stop_flags & NODE_STOP_OPERATOR_STEER) != 0); // NOT cleared
}

static void test_clamp_negative_range_values(void)
{
	assert(control_clamp_command(-100, -200, -50) == -100); // in range
	assert(control_clamp_command(-300, -200, -50) == -200); // below min
	assert(control_clamp_command(0, -200, -50) == -50);     // above max
}

// ============================================================================
// Main
// ============================================================================

int main(void)
{
	printf("=== control_logic tests ===\n\n");

	// Original state transition tests (16)
	printf("  --- state transitions ---\n");
	TEST(test_init_to_ready_when_preconditions_pass);
	TEST(test_init_to_not_ready_when_preconditions_fail);
	TEST(test_not_ready_to_ready_when_clear);
	TEST(test_not_ready_stays_when_pedal_pressed);
	TEST(test_ready_to_enable_on_safety_target);
	TEST(test_ready_to_not_ready_when_precondition_drops);
	TEST(test_enable_to_active_when_complete_and_target_active);
	TEST(test_enable_stays_enable_with_complete_flag);
	TEST(test_enable_abort_to_not_ready_on_pedal);
	TEST(test_active_safety_retreat_to_not_ready);
	TEST(test_active_override_on_pedal);
	TEST(test_override_recovery_to_ready);
	TEST(test_fault_clear_to_not_ready);
	TEST(test_fault_clear_to_ready);
	TEST(test_fault_injection_from_not_ready);
	TEST(test_unknown_state_defaults_to_fault);

	// Precondition checker (7)
	printf("\n  --- precondition checker ---\n");
	TEST(test_preconditions_null_returns_all_fail);
	TEST(test_preconditions_fr_reverse_blocks);
	TEST(test_preconditions_fr_invalid_blocks);
	TEST(test_preconditions_fr_neutral_ok);
	TEST(test_preconditions_pedal_not_rearmed);
	TEST(test_preconditions_active_fault);
	TEST(test_preconditions_multiple_combine);

	// Throttle slew (6)
	printf("\n  --- throttle slew ---\n");
	TEST(test_slew_at_target_no_change);
	TEST(test_slew_step_up);
	TEST(test_slew_step_down);
	TEST(test_slew_rate_limited);
	TEST(test_slew_null_inputs);
	TEST(test_slew_timer_overflow);

	// Command clamping (3)
	printf("\n  --- command clamping ---\n");
	TEST(test_clamp_below_min);
	TEST(test_clamp_above_max);
	TEST(test_clamp_in_range);
	TEST(test_clamp_misconfigured_returns_neutral);

	// INIT edge cases (2)
	printf("\n  --- INIT edge cases ---\n");
	TEST(test_init_stays_when_dwell_not_expired);
	TEST(test_init_dwell_timer_overflow);

	// ENABLE abort scenarios (6)
	printf("\n  --- ENABLE abort ---\n");
	TEST(test_enable_abort_safety_retreat);
	TEST(test_enable_abort_fr_reverse);
	TEST(test_enable_neutral_does_not_abort);
	TEST(test_enable_stays_when_timer_not_expired);
	TEST(test_enable_complete_fires_once);
	TEST(test_enable_timer_exact_boundary);

	// ACTIVE override variants (6)
	printf("\n  --- ACTIVE override ---\n");
	TEST(test_active_override_fr_changed);
	TEST(test_active_neutral_zeros_throttle);
	TEST(test_active_neutral_at_zero_no_change);
	TEST(test_active_override_steering_error);
	TEST(test_active_override_braking_error);
	TEST(test_active_safety_retreat_priority_over_pedal);

	// ACTIVE throttle + steering (9)
	printf("\n  --- ACTIVE throttle + steering ---\n");
	TEST(test_active_throttle_slew_applies);
	TEST(test_active_throttle_at_target_no_action);
	TEST(test_active_throttle_clamps_to_envelope);
	TEST(test_active_throttle_unconfigured_forces_neutral);
	TEST(test_active_steering_dedup_same_skips);
	TEST(test_active_steering_changed_sends);
	TEST(test_active_braking_dedup_reset_always_sends);
	TEST(test_active_envelope_unconfigured_forces_neutral);
	TEST(test_active_envelope_clamps_command);

	// Motor fault injection (4)
	printf("\n  --- motor fault injection ---\n");
	TEST(test_motor_fault_from_active);
	TEST(test_motor_fault_from_enable);
	TEST(test_motor_fault_from_ready);
	TEST(test_motor_fault_ignored_when_already_faulted);

	// FR_INVALID sensor fault (5)
	printf("\n  --- FR_INVALID sensor fault ---\n");
	TEST(test_fr_invalid_from_active);
	TEST(test_fr_invalid_from_enable);
	TEST(test_fr_invalid_from_not_ready_no_fault);
	TEST(test_fr_invalid_from_ready_no_fault);
	TEST(test_fr_invalid_no_retrigger);

	// OVERRIDE recovery edge cases (3)
	printf("\n  --- OVERRIDE recovery ---\n");
	TEST(test_override_stays_when_pedal_not_rearmed);
	TEST(test_override_stays_when_fr_reverse);
	TEST(test_override_recovers_when_fr_neutral);

	// FAULT recovery (3)
	printf("\n  --- FAULT recovery ---\n");
	TEST(test_fault_motor_comm_clears_on_recovery);
	TEST(test_fault_sensor_invalid_clears_when_fr_valid);
	TEST(test_fault_unknown_type_stays_in_fault);

	// Target sanitization (1)
	printf("\n  --- target sanitization ---\n");
	TEST(test_target_state_sanitized);

	// Safe outputs (1)
	printf("\n  --- safe outputs ---\n");
	TEST(test_override_applies_safe_outputs);

	// NULL inputs (1)
	printf("\n  --- NULL inputs ---\n");
	TEST(test_null_inputs_preserves_current_state);

	// Simultaneous faults and multi-step slew
	printf("\n  --- simultaneous faults + multi-step slew ---\n");
	TEST(test_active_simultaneous_steering_braking_errors);
	TEST(test_enable_motor_fault_before_timer_expires);
	TEST(test_slew_multi_step);
	TEST(test_slew_step_defaults_when_zero);
	TEST(test_slew_step_defaults_when_negative);
	TEST(test_slew_multi_step_clamps_to_target);
	TEST(test_active_fr_neutral_while_slewing_up);
	TEST(test_clamp_negative_range_values);
	TEST(test_enable_to_active_skips_complete_when_already_done);
	TEST(test_active_braking_envelope_unconfigured_forces_neutral);
	TEST(test_override_steering_stop_clears_when_error_resolves);
	TEST(test_override_braking_stop_clears_when_error_resolves);
	TEST(test_override_steering_stop_stays_when_error_persists);

	TEST_REPORT();
	TEST_EXIT();
}
