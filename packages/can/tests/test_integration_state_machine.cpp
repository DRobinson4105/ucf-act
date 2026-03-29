/**
 * @file test_integration_state_machine.cpp
 * @brief Integration tests for the full state machine round-trip.
 *
 * Exercises both system_state_step (Safety's perspective) and
 * control_compute_step (Control's perspective) together, simulating
 * the message exchange between Safety and Control across the full
 * INIT -> NOT_READY -> READY -> ENABLE -> ACTIVE -> NOT_READY cycle.
 */

#include "test_harness.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "system_state.h"
#include "control_logic.h"
#include "can_protocol.h"

// ============================================================================
// Default input builders
// ============================================================================

static system_state_inputs_t default_ss_inputs(void)
{
	system_state_inputs_t in = {};
	memset(&in, 0, sizeof(in));
	in.current_target = NODE_STATE_NOT_READY;
	in.now_ms = 0;
	in.boot_start_ms = 0;
	in.init_dwell_ms = 500;
	in.stop_active = false;
	in.planner_state = NODE_STATE_READY;
	in.control_state = NODE_STATE_READY;
	in.planner_alive = true;
	in.control_alive = true;
	in.planner_enable_complete = false;
	in.control_enable_complete = false;
	in.autonomy_request = false;
	in.autonomy_hold = true;
	in.active_entry_grace = false;
	in.enable_elapsed_ms = 0;
	in.enable_timeout_ms = 5000;
	return in;
}

static control_inputs_t default_ctrl_inputs(void)
{
	control_inputs_t in = {};
	memset(&in, 0, sizeof(in));
	in.target_state = NODE_STATE_NOT_READY;
	in.throttle_cmd = 0;
	in.steering_cmd = 0;
	in.braking_cmd = 0;
	in.motor_fault_code = NODE_FAULT_NONE;
	in.stop_flags = NODE_STOP_NONE;
	in.fr_state = FR_STATE_FORWARD;
	in.pedal_pressed = false;
	in.pedal_rearmed = true;
	in.steering_position_error = false;
	in.braking_position_error = false;
	in.now_ms = 0;
	in.boot_start_ms = 0;
	in.init_dwell_ms = 500;
	in.enable_start_ms = 0;
	in.enable_sequence_ms = 200;
	in.enable_work_done = false;
	in.throttle_current = 0;
	in.last_throttle_change_ms = 0;
	in.throttle_slew_interval_ms = 100;
	in.throttle_slew_step = 12;
	in.throttle_min = 0;
	in.throttle_max = 255;
	in.last_steering_sent = STEPPER_DEDUP_RESET_STEERING;
	in.last_braking_sent = STEPPER_DEDUP_RESET_BRAKING;
	in.steering_min = -3000;
	in.steering_max = 3000;
	in.braking_min = -3000;
	in.braking_max = 3000;
	return in;
}

// ============================================================================
// Full round-trip: INIT -> NOT_READY -> READY -> ENABLE -> ACTIVE -> NOT_READY
// ============================================================================

static void test_full_round_trip(void)
{
	// --- Shared state ---
	node_state_t safety_target = NODE_STATE_INIT;
	node_state_t control_state = NODE_STATE_INIT;
	node_fault_t control_fault = NODE_FAULT_NONE;
	bool control_enable_complete = false;
	bool enable_work_done = false;
	uint32_t enable_start_ms = 0;
	uint32_t enable_entered_ms = 0;

	// --- Phase 1: INIT dwell (Safety holds INIT) ---
	{
		system_state_inputs_t ss = default_ss_inputs();
		ss.current_target = NODE_STATE_INIT;
		ss.now_ms = 200; // < 500ms dwell
		ss.planner_state = NODE_STATE_NOT_READY;
		ss.control_state = NODE_STATE_INIT;
		system_state_result_t r = system_state_step(&ss);
		assert(r.new_target == NODE_STATE_INIT);
		safety_target = r.new_target;
	}

	// Control also in INIT dwell
	{
		control_inputs_t ci = default_ctrl_inputs();
		ci.target_state = safety_target;
		ci.now_ms = 200;
		control_step_result_t r = control_compute_step(control_state, control_fault, &ci);
		assert(r.new_state == NODE_STATE_INIT);
		control_state = r.new_state;
	}

	// --- Phase 2: INIT dwell expires, Control -> READY ---
	{
		control_inputs_t ci = default_ctrl_inputs();
		ci.target_state = safety_target; // still INIT
		ci.now_ms = 600;
		control_step_result_t r = control_compute_step(control_state, control_fault, &ci);
		assert(r.new_state == NODE_STATE_READY);
		control_state = r.new_state;
	}

	// Safety dwell expires, both nodes READY -> READY
	{
		system_state_inputs_t ss = default_ss_inputs();
		ss.current_target = NODE_STATE_INIT;
		ss.now_ms = 600;
		ss.planner_state = NODE_STATE_READY; // Planner is also READY
		ss.control_state = control_state;    // READY
		system_state_result_t r = system_state_step(&ss);
		assert(r.new_target == NODE_STATE_READY);
		safety_target = r.new_target;
	}

	// --- Phase 3: READY -> ENABLE (autonomy request) ---
	{
		system_state_inputs_t ss = default_ss_inputs();
		ss.current_target = NODE_STATE_READY;
		ss.now_ms = 700;
		ss.planner_state = NODE_STATE_READY;
		ss.control_state = control_state;
		ss.autonomy_request = true;
		system_state_result_t r = system_state_step(&ss);
		assert(r.new_target == NODE_STATE_ENABLE);
		safety_target = r.new_target;
		enable_entered_ms = 700;
	}

	// Control sees ENABLE target -> starts enable sequence
	{
		control_inputs_t ci = default_ctrl_inputs();
		ci.target_state = safety_target;
		ci.now_ms = 750;
		control_step_result_t r = control_compute_step(control_state, control_fault, &ci);
		assert(r.new_state == NODE_STATE_ENABLE);
		assert((r.actions & CONTROL_ACTION_START_ENABLE) != 0);
		control_state = r.new_state;
		enable_start_ms = r.enable_start_ms;
		enable_work_done = r.enable_work_done;
	}

	// Control enable timer not yet expired -> stays ENABLE
	{
		control_inputs_t ci = default_ctrl_inputs();
		ci.target_state = safety_target;
		ci.now_ms = 850; // 100ms into 200ms enable sequence
		ci.enable_start_ms = enable_start_ms;
		ci.enable_work_done = enable_work_done;
		control_step_result_t r = control_compute_step(control_state, control_fault, &ci);
		assert(r.new_state == NODE_STATE_ENABLE);
		assert((r.status_flags & NODE_STATUS_FLAG_ENABLE_COMPLETE) == 0);
		control_state = r.new_state;
	}

	// --- Phase 4: ENABLE -> ACTIVE (enable timer expires) ---
	// Control enable timer expires -> COMPLETE_ENABLE + enable_complete flag
	{
		control_inputs_t ci = default_ctrl_inputs();
		ci.target_state = safety_target;
		ci.now_ms = 960; // >= 200ms after enable_start
		ci.enable_start_ms = enable_start_ms;
		ci.enable_work_done = enable_work_done;
		control_step_result_t r = control_compute_step(control_state, control_fault, &ci);
		assert(r.new_state == NODE_STATE_ENABLE);
		assert((r.actions & CONTROL_ACTION_COMPLETE_ENABLE) != 0);
		assert((r.status_flags & NODE_STATUS_FLAG_ENABLE_COMPLETE) != 0);
		control_state = r.new_state;
		control_enable_complete = true;
		enable_work_done = r.enable_work_done;
	}

	// Safety sees both nodes ENABLE + enable_complete -> ACTIVE
	{
		system_state_inputs_t ss = default_ss_inputs();
		ss.current_target = NODE_STATE_ENABLE;
		ss.now_ms = 1000;
		ss.planner_state = NODE_STATE_ENABLE;
		ss.control_state = control_state;
		ss.planner_enable_complete = true;
		ss.control_enable_complete = control_enable_complete;
		ss.autonomy_hold = true;
		ss.enable_elapsed_ms = 1000 - enable_entered_ms;
		system_state_result_t r = system_state_step(&ss);
		assert(r.new_target == NODE_STATE_ACTIVE);
		safety_target = r.new_target;
	}

	// Control sees ACTIVE target -> enters ACTIVE
	{
		control_inputs_t ci = default_ctrl_inputs();
		ci.target_state = safety_target;
		ci.now_ms = 1050;
		ci.enable_start_ms = enable_start_ms;
		ci.enable_work_done = enable_work_done;
		control_step_result_t r = control_compute_step(control_state, control_fault, &ci);
		assert(r.new_state == NODE_STATE_ACTIVE);
		control_state = r.new_state;
	}

	// --- Phase 5: ACTIVE -> NOT_READY (e-stop) ---
	// Safety sees stop -> NOT_READY
	{
		system_state_inputs_t ss = default_ss_inputs();
		ss.current_target = NODE_STATE_ACTIVE;
		ss.now_ms = 2000;
		ss.planner_state = NODE_STATE_ACTIVE;
		ss.control_state = control_state;
		ss.autonomy_hold = true;
		ss.stop_active = true;
		system_state_result_t r = system_state_step(&ss);
		assert(r.new_target == NODE_STATE_NOT_READY);
		safety_target = r.new_target;
	}

	// Control sees NOT_READY target -> disables actuators
	{
		control_inputs_t ci = default_ctrl_inputs();
		ci.target_state = safety_target;
		ci.now_ms = 2050;
		control_step_result_t r = control_compute_step(control_state, control_fault, &ci);
		assert(r.new_state == NODE_STATE_READY || r.new_state == NODE_STATE_NOT_READY);
		assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) != 0);
	}
}

// ============================================================================
// ENABLE timeout integration
// ============================================================================

static void test_enable_timeout_integration(void)
{
	// Advance to ENABLE, then one node never completes -> timeout retreat
	system_state_inputs_t ss = default_ss_inputs();
	ss.current_target = NODE_STATE_ENABLE;
	ss.now_ms = 6000;
	ss.planner_state = NODE_STATE_ENABLE;
	ss.control_state = NODE_STATE_ENABLE;
	ss.planner_enable_complete = true;
	ss.control_enable_complete = false; // Control never completes
	ss.autonomy_hold = true;
	ss.enable_elapsed_ms = 5000; // exactly at timeout
	ss.enable_timeout_ms = 5000;

	system_state_result_t r = system_state_step(&ss);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

// ============================================================================
// Override during ACTIVE integration
// ============================================================================

static void test_override_during_active_integration(void)
{
	// Control in ACTIVE, pedal pressed -> override retreat
	control_inputs_t ci = default_ctrl_inputs();
	ci.target_state = NODE_STATE_ACTIVE;
	ci.now_ms = 5000;
	ci.pedal_pressed = true;
	ci.enable_work_done = true;

	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &ci);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert((r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE) != 0);
	assert(r.override_reason == OVERRIDE_REASON_THROTTLE);

	// Safety sees Control went NOT_READY -> retreats
	system_state_inputs_t ss = default_ss_inputs();
	ss.current_target = NODE_STATE_ACTIVE;
	ss.now_ms = 5050;
	ss.planner_state = NODE_STATE_ACTIVE;
	ss.control_state = r.new_state; // NOT_READY
	ss.autonomy_hold = true;

	system_state_result_t ssr = system_state_step(&ss);
	assert(ssr.new_target == NODE_STATE_NOT_READY);
}

// ============================================================================
// Autonomy hold drop during ACTIVE
// ============================================================================

static void test_autonomy_halt_during_active(void)
{
	system_state_inputs_t ss = default_ss_inputs();
	ss.current_target = NODE_STATE_ACTIVE;
	ss.now_ms = 3000;
	ss.planner_state = NODE_STATE_ACTIVE;
	ss.control_state = NODE_STATE_ACTIVE;
	ss.autonomy_hold = false; // Planner drops hold

	system_state_result_t r = system_state_step(&ss);
	// Both nodes are ACTIVE (not READY), so retreat to NOT_READY
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

// ============================================================================
// FR_INVALID during ACTIVE triggers override
// ============================================================================

static void test_fr_invalid_during_active_integration(void)
{
	control_inputs_t ci = default_ctrl_inputs();
	ci.target_state = NODE_STATE_ACTIVE;
	ci.now_ms = 4000;
	ci.fr_state = FR_STATE_INVALID;
	ci.enable_work_done = true;

	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &ci);
	assert(r.new_state == NODE_STATE_NOT_READY);
	assert(r.new_fault_flags == NODE_FAULT_CONTROL_SENSOR_INVALID);
	assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) != 0);
	assert(r.disable_reason == CONTROL_DISABLE_REASON_SENSOR_INVALID);
}

// ============================================================================
// Stale planner command — throttle zeroed during ACTIVE
// ============================================================================

static void test_stale_planner_command_zeros_throttle(void)
{
	// When Planner commands go stale, main.cpp zeros throttle_cmd before
	// calling control_compute_step.  Verify the control logic applies the
	// zeroed throttle via slew and emits APPLY_THROTTLE.
	control_inputs_t ci = default_ctrl_inputs();
	ci.target_state = NODE_STATE_ACTIVE;
	ci.now_ms = 5000;
	ci.enable_work_done = true;
	ci.throttle_current = 100;         // cart is currently driving
	ci.throttle_cmd = 0;               // stale detection zeroed the command
	ci.last_throttle_change_ms = 4800; // last change 200ms ago (past slew interval)

	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &ci);
	assert(r.new_state == NODE_STATE_ACTIVE); // stays ACTIVE (stale != fault)
	assert((r.actions & CONTROL_ACTION_APPLY_THROTTLE) != 0);
	assert(r.throttle_level < ci.throttle_current); // slewing toward 0
}

static void test_stale_planner_command_preserves_steering(void)
{
	// Stale detection zeros throttle but keeps last steering/braking.
	// Verify steering dedup sees no change (same position sent again).
	control_inputs_t ci = default_ctrl_inputs();
	ci.target_state = NODE_STATE_ACTIVE;
	ci.now_ms = 5000;
	ci.enable_work_done = true;
	ci.throttle_cmd = 0;
	ci.steering_cmd = 1500;
	ci.last_steering_sent = 1500; // already at this position

	control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &ci);
	assert(r.send_steering == false); // no change, dedup suppresses
}

// ============================================================================
// Main
// ============================================================================

int main(void)
{
	printf("=== integration state machine tests ===\n\n");

	printf("  --- full round-trip ---\n");
	TEST(test_full_round_trip);

	printf("\n  --- enable timeout ---\n");
	TEST(test_enable_timeout_integration);

	printf("\n  --- override during active ---\n");
	TEST(test_override_during_active_integration);

	printf("\n  --- autonomy halt ---\n");
	TEST(test_autonomy_halt_during_active);

	printf("\n  --- FR_INVALID during active ---\n");
	TEST(test_fr_invalid_during_active_integration);

	printf("\n  --- stale planner command ---\n");
	TEST(test_stale_planner_command_zeros_throttle);
	TEST(test_stale_planner_command_preserves_steering);

	TEST_REPORT();
	TEST_EXIT();
}
