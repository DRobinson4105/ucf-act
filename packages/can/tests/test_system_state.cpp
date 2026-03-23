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

	TEST_REPORT();
	TEST_EXIT();
}
