/**
 * @file safety_logic.cpp
 * @brief Pure decision logic for the Safety ESP32 — no hardware dependencies.
 */

#include "safety_logic.h"
#include "can_protocol.h"

bool safety_compute_ultrasonic_trigger(bool too_close, bool healthy)
{
	// Fail-safe: trigger if obstacle detected OR sensor not healthy
	return too_close || !healthy;
}

safety_decision_t safety_evaluate(const safety_inputs_t *inputs)
{
	safety_decision_t d = {
		.stop_active = true,
		.stop_flags = NODE_STOP_NONE,
		.fault_flags = NODE_FAULT_NONE,
	};

	// Fail-safe: NULL input returns stop_active=true (blocks everything)
	if (!inputs)
		return d;

	// Stop causes (non-fault)
	node_stop_t stops = NODE_STOP_NONE;
	if (inputs->push_button_active)
		stops |= NODE_STOP_PUSH_BUTTON;
	if (inputs->rf_remote_active)
		stops |= NODE_STOP_REMOTE;
	if (inputs->ultrasonic_too_close)
		stops |= NODE_STOP_ULTRASONIC_OBSTACLE;
	stops |= inputs->planner_stop_flags;
	stops |= inputs->control_stop_flags;

	// Fault causes (issues/timeouts)
	node_fault_t faults = NODE_FAULT_NONE;
	if (!inputs->ultrasonic_healthy)
		faults |= NODE_FAULT_SAFETY_ULTRASONIC_UNHEALTHY;
	if (node_fault_is_planner(inputs->planner_fault_flags))
		faults |= NODE_FAULT_SAFETY_PLANNER_ISSUE;
	if (!inputs->planner_alive)
		faults |= NODE_FAULT_SAFETY_PLANNER_TIMEOUT;
	if (node_fault_is_control(inputs->control_fault_flags))
		faults |= NODE_FAULT_SAFETY_CONTROL_ISSUE;
	if (!inputs->control_alive)
		faults |= NODE_FAULT_SAFETY_CONTROL_TIMEOUT;

	d.stop_flags = stops;
	d.fault_flags = faults;
	d.stop_active = (stops != NODE_STOP_NONE || faults != NODE_FAULT_NONE);

	return d;
}
