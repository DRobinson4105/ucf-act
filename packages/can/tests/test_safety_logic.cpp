/**
 * @file test_safety_logic.cpp
 * @brief Unit tests for pure safety decision logic.
 */

#include "test_harness.h"

#include <stdbool.h>
#include <stdint.h>

#include "can_protocol.h"
#include "safety_logic.h"

static safety_inputs_t safe_inputs(void)
{
	safety_inputs_t in = {
		.push_button_active = false,
		.rf_remote_active = false,
		.ultrasonic_too_close = false,
		.ultrasonic_healthy = true,
		.planner_alive = true,
		.control_alive = true,
		.planner_issue = false,
		.control_issue = false,
		.planner_stop = NODE_STOP_NONE,
		.control_stop = NODE_STOP_NONE,
	};
	return in;
}

static void test_ultrasonic_trigger_fail_safe(void)
{
	assert(!safety_compute_ultrasonic_trigger(false, true));
	assert(safety_compute_ultrasonic_trigger(true, true));
	assert(safety_compute_ultrasonic_trigger(false, false));
	assert(safety_compute_ultrasonic_trigger(true, false));
}

static void test_all_clear(void)
{
	safety_inputs_t in = safe_inputs();
	safety_decision_t d = safety_evaluate(&in);
	assert(!d.stop_active);
	assert(d.stop_flags == NODE_STOP_NONE);
	assert(d.fault_code == NODE_FAULT_NONE);
	assert(d.relay_enable);
}

static void test_stop_only_local_inputs(void)
{
	safety_inputs_t in = safe_inputs();
	in.push_button_active = true;
	in.rf_remote_active = true;
	in.ultrasonic_too_close = true;
	safety_decision_t d = safety_evaluate(&in);

	assert(d.stop_active);
	assert(d.stop_flags == (NODE_STOP_PUSH_BUTTON | NODE_STOP_REMOTE | NODE_STOP_ULTRASONIC_OBSTACLE));
	assert(d.fault_code == NODE_FAULT_NONE);
	assert(!d.relay_enable);
}

static void test_stop_only_forwarded_from_nodes(void)
{
	safety_inputs_t in = safe_inputs();
	in.planner_stop = NODE_STOP_APP_REQUEST;
	in.control_stop = NODE_STOP_OPERATOR_PEDAL | NODE_STOP_OPERATOR_FR;
	safety_decision_t d = safety_evaluate(&in);

	assert(d.stop_active);
	assert(d.stop_flags == (NODE_STOP_APP_REQUEST | NODE_STOP_OPERATOR_PEDAL | NODE_STOP_OPERATOR_FR));
	assert(d.fault_code == NODE_FAULT_NONE);
	assert(!d.relay_enable);
}

static void test_fault_only_timeouts_and_issues(void)
{
	safety_inputs_t in = safe_inputs();
	in.ultrasonic_healthy = false;
	in.planner_issue = true;
	in.planner_alive = false;
	in.control_issue = true;
	in.control_alive = false;
	safety_decision_t d = safety_evaluate(&in);

	assert(d.stop_active);
	assert(d.stop_flags == NODE_STOP_NONE);
	assert(d.fault_code ==
	       (NODE_FAULT_SAFETY_ULTRASONIC_UNHEALTHY | NODE_FAULT_SAFETY_PLANNER_ISSUE |
	        NODE_FAULT_SAFETY_PLANNER_TIMEOUT | NODE_FAULT_SAFETY_CONTROL_ISSUE | NODE_FAULT_SAFETY_CONTROL_TIMEOUT));
	assert(!d.relay_enable);
}

static void test_combined_stop_and_fault_channels(void)
{
	safety_inputs_t in = safe_inputs();
	in.push_button_active = true;
	in.control_stop = NODE_STOP_OPERATOR_STEER;
	in.planner_issue = true;
	in.control_alive = false;
	safety_decision_t d = safety_evaluate(&in);

	assert(d.stop_active);
	assert(d.stop_flags == (NODE_STOP_PUSH_BUTTON | NODE_STOP_OPERATOR_STEER));
	assert(d.fault_code == (NODE_FAULT_SAFETY_PLANNER_ISSUE | NODE_FAULT_SAFETY_CONTROL_TIMEOUT));
	assert(!d.relay_enable);
}

static void test_null_input_is_fail_safe(void)
{
	safety_decision_t d = safety_evaluate(NULL);
	assert(d.stop_active);
	assert(d.stop_flags == NODE_STOP_NONE);
	assert(d.fault_code == NODE_FAULT_NONE);
	assert(!d.relay_enable);
}

int main(void)
{
	printf("\n=== safety_logic unit tests ===\n\n");

	TEST(test_ultrasonic_trigger_fail_safe);
	TEST(test_all_clear);
	TEST(test_stop_only_local_inputs);
	TEST(test_stop_only_forwarded_from_nodes);
	TEST(test_fault_only_timeouts_and_issues);
	TEST(test_combined_stop_and_fault_channels);
	TEST(test_null_input_is_fail_safe);

	TEST_REPORT();
	TEST_EXIT();
}
