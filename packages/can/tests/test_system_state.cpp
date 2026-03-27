/**
 * @file test_system_state.cpp
 * @brief Unit tests for Safety target-state logic (system_state.c).
 */

#include "test_harness.h"
#include <stdbool.h>
#include <stdint.h>

#include "system_state.h"
#include "can_protocol.h"

static system_state_inputs_t default_inputs(void)
{
	system_state_inputs_t in = {
		.current_target = NODE_STATE_NOT_READY,
		.now_ms = 0,
		.boot_start_ms = 0,
		.init_dwell_ms = 0,
		.stop_active = false,
		.planner_state = NODE_STATE_READY,
		.control_state = NODE_STATE_READY,
		.planner_alive = true,
		.control_alive = true,
		.planner_enable_complete = false,
		.control_enable_complete = false,
		.autonomy_request = false,
		.autonomy_hold = true,
		.active_entry_grace = false,
		.enable_elapsed_ms = 0,
		.enable_timeout_ms = 0,
	};
	return in;
}

static void test_init_dwell_holds_then_not_ready(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_INIT;
	in.boot_start_ms = 1000;
	in.init_dwell_ms = 500;
	in.planner_state = NODE_STATE_NOT_READY;
	in.control_state = NODE_STATE_NOT_READY;

	in.now_ms = 1499;
	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_INIT);
	assert(r.target_changed == false);

	in.now_ms = 1500;
	r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

static void test_init_dwell_can_transition_to_ready(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_INIT;
	in.boot_start_ms = 1000;
	in.init_dwell_ms = 500;
	in.now_ms = 1700;
	in.planner_state = NODE_STATE_READY;
	in.control_state = NODE_STATE_READY;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_READY);
	assert(r.target_changed == true);
}

static void test_not_ready_to_ready_when_both_nodes_ready(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_NOT_READY;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_READY);
	assert(r.target_changed == true);
}

static void test_not_ready_stays_when_node_not_ready(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_NOT_READY;
	in.control_state = NODE_STATE_NOT_READY;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == false);
}

static void test_ready_to_enable_on_request(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_READY;
	in.autonomy_request = true;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_ENABLE);
	assert(r.target_changed == true);
}

static void test_ready_to_not_ready_when_node_drops_ready(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_READY;
	in.control_state = NODE_STATE_NOT_READY;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

static void test_enable_to_active_on_dual_complete(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ENABLE;
	in.planner_state = NODE_STATE_ENABLE;
	in.control_state = NODE_STATE_ENABLE;
	in.planner_enable_complete = true;
	in.control_enable_complete = true;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_ACTIVE);
	assert(r.target_changed == true);
}

static void test_enable_stays_while_working(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ENABLE;
	in.planner_state = NODE_STATE_ENABLE;
	in.control_state = NODE_STATE_ENABLE;
	in.planner_enable_complete = true;
	in.control_enable_complete = false;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_ENABLE);
	assert(r.target_changed == false);
}

static void test_enable_to_not_ready_if_node_reboots(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ENABLE;
	in.planner_state = NODE_STATE_INIT;
	in.control_state = NODE_STATE_ENABLE;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

static void test_autonomy_halt_enable_to_ready_when_nodes_ready(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ENABLE;
	in.planner_state = NODE_STATE_READY;
	in.control_state = NODE_STATE_READY;
	in.autonomy_hold = false;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_READY);
	assert(r.target_changed == true);
}

static void test_autonomy_halt_active_to_not_ready_when_nodes_not_ready(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ACTIVE;
	in.control_state = NODE_STATE_NOT_READY;
	in.autonomy_hold = false;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

static void test_hard_negative_retreats_to_not_ready(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ACTIVE;
	in.stop_active = true;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);

	in = default_inputs();
	in.current_target = NODE_STATE_ACTIVE;
	in.control_alive = false;
	r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

static void test_active_stays_active_when_nominal(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ACTIVE;
	in.planner_state = NODE_STATE_ACTIVE;
	in.control_state = NODE_STATE_ACTIVE;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_ACTIVE);
	assert(r.target_changed == false);
}

static void test_active_retreats_when_node_not_active(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ACTIVE;
	in.planner_state = NODE_STATE_ACTIVE;
	in.control_state = NODE_STATE_ENABLE;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

static void test_active_grace_allows_enable_handoff(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ACTIVE;
	in.active_entry_grace = true;
	in.planner_state = NODE_STATE_ENABLE;
	in.control_state = NODE_STATE_ACTIVE;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_ACTIVE);
	assert(r.target_changed == false);
}

static void test_unknown_target_retreats_to_not_ready(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = 0xFF;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

// ============================================================================
// Additional edge cases
// ============================================================================

static void test_ready_retreats_when_node_drops_despite_request(void)
{
	// Autonomy request asserted, but control is NOT_READY — retreat wins over request
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_READY;
	in.autonomy_request = true;
	in.control_state = NODE_STATE_NOT_READY;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

static void test_init_dwell_exact_boundary(void)
{
	// At exactly the dwell boundary (elapsed == dwell), should transition to READY
	// (both nodes READY, no stops, both alive per default_inputs)
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_INIT;
	in.boot_start_ms = 1000;
	in.init_dwell_ms = 500;
	in.now_ms = 1500; // elapsed = 500 == 500

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_READY);
	assert(r.target_changed == true);
}

static void test_planner_alive_control_dead_retreats(void)
{
	// Planner alive but control timed out — must retreat
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ACTIVE;
	in.planner_state = NODE_STATE_ACTIVE;
	in.control_state = NODE_STATE_ACTIVE;
	in.planner_alive = true;
	in.control_alive = false;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

static void test_enable_retreats_on_stop_active(void)
{
	// stop_active during ENABLE should retreat to NOT_READY
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ENABLE;
	in.planner_state = NODE_STATE_ENABLE;
	in.control_state = NODE_STATE_ENABLE;
	in.stop_active = true;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

static void test_null_inputs_returns_not_ready(void)
{
	system_state_result_t r = system_state_step(NULL);
	assert(r.new_target == NODE_STATE_NOT_READY);
}

static void test_planner_not_ready_blocks_ready_transition(void)
{
	// Planner NOT_READY, control READY — should stay NOT_READY
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_NOT_READY;
	in.planner_state = NODE_STATE_NOT_READY;
	in.control_state = NODE_STATE_READY;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == false);
}

// ============================================================================
// ENABLE timeout tests
// ============================================================================

static void test_enable_timeout_retreats_to_not_ready(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ENABLE;
	in.planner_state = NODE_STATE_ENABLE;
	in.control_state = NODE_STATE_ENABLE;
	in.planner_enable_complete = false;
	in.control_enable_complete = false;
	in.enable_elapsed_ms = 5000;
	in.enable_timeout_ms = 5000;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_NOT_READY);
	assert(r.target_changed == true);
}

static void test_enable_no_timeout_when_within_window(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ENABLE;
	in.planner_state = NODE_STATE_ENABLE;
	in.control_state = NODE_STATE_ENABLE;
	in.planner_enable_complete = false;
	in.control_enable_complete = false;
	in.enable_elapsed_ms = 4999;
	in.enable_timeout_ms = 5000;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_ENABLE);
	assert(r.target_changed == false);
}

static void test_enable_timeout_zero_disables_watchdog(void)
{
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ENABLE;
	in.planner_state = NODE_STATE_ENABLE;
	in.control_state = NODE_STATE_ENABLE;
	in.planner_enable_complete = false;
	in.control_enable_complete = false;
	in.enable_elapsed_ms = 999999;
	in.enable_timeout_ms = 0; // disabled

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_ENABLE);
	assert(r.target_changed == false);
}

static void test_enable_timeout_does_not_block_normal_advance(void)
{
	// Both nodes complete before timeout — should advance to ACTIVE
	system_state_inputs_t in = default_inputs();
	in.current_target = NODE_STATE_ENABLE;
	in.planner_state = NODE_STATE_ENABLE;
	in.control_state = NODE_STATE_ENABLE;
	in.planner_enable_complete = true;
	in.control_enable_complete = true;
	in.enable_elapsed_ms = 200;
	in.enable_timeout_ms = 5000;

	system_state_result_t r = system_state_step(&in);
	assert(r.new_target == NODE_STATE_ACTIVE);
	assert(r.target_changed == true);
}

// ============================================================================
// Main
// ============================================================================

int main(void)
{
	printf("=== system_state tests ===\n\n");

	TEST(test_init_dwell_holds_then_not_ready);
	TEST(test_init_dwell_can_transition_to_ready);
	TEST(test_not_ready_to_ready_when_both_nodes_ready);
	TEST(test_not_ready_stays_when_node_not_ready);
	TEST(test_ready_to_enable_on_request);
	TEST(test_ready_to_not_ready_when_node_drops_ready);
	TEST(test_enable_to_active_on_dual_complete);
	TEST(test_enable_stays_while_working);
	TEST(test_enable_to_not_ready_if_node_reboots);
	TEST(test_autonomy_halt_enable_to_ready_when_nodes_ready);
	TEST(test_autonomy_halt_active_to_not_ready_when_nodes_not_ready);
	TEST(test_hard_negative_retreats_to_not_ready);
	TEST(test_active_stays_active_when_nominal);
	TEST(test_active_retreats_when_node_not_active);
	TEST(test_active_grace_allows_enable_handoff);
	TEST(test_unknown_target_retreats_to_not_ready);

	printf("\n  --- additional edge cases ---\n");
	TEST(test_ready_retreats_when_node_drops_despite_request);
	TEST(test_init_dwell_exact_boundary);
	TEST(test_planner_alive_control_dead_retreats);
	TEST(test_enable_retreats_on_stop_active);
	TEST(test_null_inputs_returns_not_ready);
	TEST(test_planner_not_ready_blocks_ready_transition);

	printf("\n  --- enable timeout ---\n");
	TEST(test_enable_timeout_retreats_to_not_ready);
	TEST(test_enable_no_timeout_when_within_window);
	TEST(test_enable_timeout_zero_disables_watchdog);
	TEST(test_enable_timeout_does_not_block_normal_advance);

	TEST_REPORT();
	TEST_EXIT();
}
