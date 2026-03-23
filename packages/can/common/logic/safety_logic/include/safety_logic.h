/**
 * @file safety_logic.h
 * @brief Pure decision logic for the Safety ESP32.
 *
 * This module contains the safety-critical e-stop evaluation, extracted as
 * pure functions with no hardware dependencies. This allows comprehensive
 * unit testing on the host without mocking any ESP-IDF or FreeRTOS APIs.
 *
 * The system state machine (advancing NOT_READY -> READY -> ENABLE -> ACTIVE) is now
 * handled by the system_state component. safety_logic focuses solely on
 * e-stop evaluation and relay decisions.
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
	bool push_button_active;   // Physical push button e-stop pressed
	bool rf_remote_active;     // RF remote e-stop engaged
	bool ultrasonic_too_close; // Obstacle within stop distance
	bool ultrasonic_healthy;   // Sensor responding within timeout
	bool planner_alive;        // Planner heartbeat received within timeout
	bool control_alive;        // Control ESP32 heartbeat within timeout
	bool planner_issue;        // Planner reported an unexpected issue cause
	bool control_issue;        // Control reported an unexpected issue cause
	node_stop_t planner_stop;  // Planner non-fault stop flags (NODE_STOP_*)
	node_stop_t control_stop;  // Control non-fault stop flags (NODE_STOP_*)
} safety_inputs_t;

// ============================================================================
// Safety Decision (output of the evaluation)
// ============================================================================

typedef struct
{
	bool stop_active;        // true = any stop or fault condition present
	node_stop_t stop_flags;  // NODE_STOP_* bitmask (non-fault stop causes)
	node_fault_t fault_code; // NODE_FAULT_SAFETY_* bitmask (fault causes)
	bool relay_enable;       // true = power relay should be ON
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
 * @return true if ultrasonic should trigger an e-stop
 */
bool safety_compute_ultrasonic_trigger(bool too_close, bool healthy);

/**
 * @brief Evaluate all safety inputs and produce a decision.
 *
 * Builds separate stop/fault bitmasks:
 *   stop_flags: push_button | rf_remote | ultrasonic_obstacle | planner_stop | control_stop
 *   fault_code: ultrasonic_unhealthy | planner_issue | planner_timeout |
 *               control_issue | control_timeout
 *
 * @param inputs  All sensor and heartbeat readings
 * @return Decision: stop state, stop_flags, fault_code, relay command
 */
safety_decision_t safety_evaluate(const safety_inputs_t *inputs);

#ifdef __cplusplus
}
#endif
