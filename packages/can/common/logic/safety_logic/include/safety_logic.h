/**
 * @file safety_logic.h
 * @brief Pure decision logic for the Safety ESP32.
 *
 * This module contains the safety-critical stop/fault evaluation, extracted as
 * pure functions with no hardware dependencies. This allows comprehensive
 * unit testing on the host without mocking any ESP-IDF or FreeRTOS APIs.
 *
 * The system state machine (advancing NOT_READY -> READY -> ENABLE -> ACTIVE) is now
 * handled by the system_state component. safety_logic focuses solely on
 * stop/fault evaluation.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "can_protocol.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================================
// Safety Inputs (all sensor / heartbeat state, read by the caller)
// ============================================================================

typedef struct
{
	// Local sensor inputs
	bool push_button_active;   // Physical push button pressed
	bool rf_remote_active;     // RF remote kill engaged
	bool ultrasonic_too_close; // Obstacle within stop distance
	bool ultrasonic_healthy;   // Sensor responding within timeout

	// Heartbeat liveness
	bool planner_alive; // Planner heartbeat received within timeout
	bool control_alive; // Control ESP32 heartbeat within timeout

	// Remote node cause channels (from decoded heartbeats)
	node_fault_t planner_fault_flags; // Planner fault flags (NODE_FAULT_PLANNER_*, bitmask)
	node_fault_t control_fault_flags; // Control fault flags (NODE_FAULT_CONTROL_*, bitmask)
	node_stop_t planner_stop_flags;   // Planner stop flags (NODE_STOP_*, bitmask)
	node_stop_t control_stop_flags;   // Control stop flags (NODE_STOP_*, bitmask)
} safety_inputs_t;

// ============================================================================
// Safety Decision (output of the evaluation)
// ============================================================================

typedef struct
{
	bool stop_active;         // true = any stop or fault condition present
	node_stop_t stop_flags;   // NODE_STOP_* bitmask (non-fault stop causes)
	node_fault_t fault_flags; // NODE_FAULT_SAFETY_* bitmask (fault causes)
} safety_decision_t;

// ============================================================================
// Pure Decision Functions
// ============================================================================

/**
 * @brief Compute the ultrasonic trigger state (fail-safe logic).
 *
 * Returns true if obstacle detected OR sensor is unhealthy.
 * The fail-safe assumption is: if we can't confirm the path is clear,
 * treat it as blocked.
 *
 * @param too_close  true if obstacle within threshold distance
 * @param healthy    true if sensor data is recent / valid
 * @return true if ultrasonic should trigger a stop
 */
bool safety_compute_ultrasonic_trigger(bool too_close, bool healthy);

/**
 * @brief Evaluate all safety inputs and produce a decision.
 *
 * Builds separate stop/fault bitmasks:
 *   stop_flags:  push_button | rf_remote | ultrasonic_obstacle | planner_stop | control_stop
 *   fault_flags: ultrasonic_unhealthy | planner_issue | planner_timeout |
 *                control_issue | control_timeout
 *
 * @param inputs  All sensor and heartbeat readings
 * @return Decision: stop_active, stop_flags, fault_flags
 */
safety_decision_t safety_evaluate(const safety_inputs_t *inputs);

#ifdef __cplusplus
}
#endif
