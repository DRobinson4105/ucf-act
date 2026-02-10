/**
 * @file safety_logic.h
 * @brief Pure decision logic for the Safety ESP32.
 *
 * This module contains the safety-critical e-stop evaluation and
 * auto-allowed computation, extracted as pure functions with no
 * hardware dependencies. This allows comprehensive unit testing
 * on the host without mocking any ESP-IDF or FreeRTOS APIs.
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
    bool push_button_active;     // Physical mushroom e-stop pressed
    bool rf_remote_active;       // RF remote e-stop engaged
    bool ultrasonic_too_close;   // Obstacle within stop distance
    bool ultrasonic_healthy;     // Sensor responding within timeout
    bool orin_alive;             // Orin heartbeat received within timeout
    bool control_alive;          // Control ESP32 heartbeat within timeout
    bool orin_error;             // Orin reported FAULT state
    bool control_error;          // Control reported FAULT state
} safety_inputs_t;

// ============================================================================
// Safety Decision (output of the evaluation)
// ============================================================================

typedef struct {
    bool estop_active;           // true = e-stop condition present
    uint8_t estop_reason;        // ESTOP_REASON_* from can_protocol.hh
    bool auto_allowed;           // true = autonomous mode may proceed
    bool relay_enable;           // true = power relay should be ON
} safety_decision_t;

// ============================================================================
// Broadcast Timing
// ============================================================================

typedef struct {
    bool should_broadcast;       // true = send CAN frame this cycle
    bool state_changed;          // true = estop state toggled since last broadcast
} safety_broadcast_t;

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
 * Applies the e-stop priority chain:
 *   push_button > rf_remote > ultrasonic > orin_error >
 *   orin_timeout > control_timeout > control_error
 *
 * @param inputs  All sensor and heartbeat readings
 * @return Decision: estop state, reason, auto_allowed, relay command
 */
safety_decision_t safety_evaluate(const safety_inputs_t *inputs);

/**
 * @brief Determine whether to broadcast the auto-allowed CAN frame.
 *
 * Broadcasts on state change or periodically while estop is active.
 *
 * @param current_estop    Current estop state (from safety_evaluate)
 * @param last_estop       Previous estop state
 * @param now_ticks        Current tick count
 * @param last_broadcast   Tick count of last broadcast
 * @param periodic_ticks   Periodic interval for re-broadcast during estop
 * @return Broadcast decision
 */
safety_broadcast_t safety_should_broadcast(bool current_estop, bool last_estop,
                                            uint32_t now_ticks, uint32_t last_broadcast,
                                            uint32_t periodic_ticks);

#ifdef __cplusplus
}
#endif
