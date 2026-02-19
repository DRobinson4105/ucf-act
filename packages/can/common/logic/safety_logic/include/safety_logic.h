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

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Safety Inputs (all sensor / heartbeat state, read by the caller)
// ============================================================================

typedef struct {
    bool push_button_active;     // Physical push button e-stop pressed
    bool rf_remote_active;       // RF remote e-stop engaged
    bool ultrasonic_too_close;   // Obstacle within stop distance
    bool ultrasonic_healthy;     // Sensor responding within timeout
    bool planner_alive;          // Planner heartbeat received within timeout
    bool control_alive;          // Control ESP32 heartbeat within timeout
    bool planner_error;          // Planner reported FAULT state
    bool control_error;          // Control reported FAULT state
} safety_inputs_t;

// ============================================================================
// Safety Decision (output of the evaluation)
// ============================================================================

typedef struct {
    bool estop_active;           // true = e-stop condition present
    uint8_t fault_code;          // NODE_FAULT_ESTOP_* from can_protocol.hh (0 when safe)
    bool relay_enable;           // true = power relay should be ON
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
 * Builds an e-stop fault bitmask where each active condition sets its bit:
 *   push_button | rf_remote | ultrasonic | planner_error |
 *   planner_timeout | control_error | control_timeout
 *
 * @param inputs  All sensor and heartbeat readings
 * @return Decision: estop state, fault_code, relay command
 */
safety_decision_t safety_evaluate(const safety_inputs_t *inputs);

#ifdef __cplusplus
}
#endif
