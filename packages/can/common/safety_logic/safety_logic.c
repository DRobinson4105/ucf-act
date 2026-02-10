/**
 * @file safety_logic.c
 * @brief Pure decision logic for the Safety ESP32 — no hardware dependencies.
 */

#include "safety_logic.h"
#include "can_protocol.hh"

bool safety_compute_ultrasonic_trigger(bool too_close, bool healthy) {
    // Fail-safe: trigger if obstacle detected OR sensor not healthy
    return too_close || !healthy;
}

safety_decision_t safety_evaluate(const safety_inputs_t *inputs) {
    safety_decision_t d = {
        .estop_active = true,
        .fault_code = NODE_FAULT_NONE,
        .relay_enable = false,
    };

    // Fail-safe: NULL input returns estop_active=true (blocks everything)
    if (!inputs) return d;

    // Compute ultrasonic trigger with fail-safe
    bool ultrasonic_triggered = safety_compute_ultrasonic_trigger(
        inputs->ultrasonic_too_close, inputs->ultrasonic_healthy);

    // Heartbeat timeouts
    bool planner_timeout = !inputs->planner_alive;
    bool control_timeout = !inputs->control_alive;

    // E-stop priority chain (highest priority first)
    if (inputs->push_button_active) {
        d.estop_active = true;
        d.fault_code = NODE_FAULT_ESTOP_MUSHROOM;
    } else if (inputs->rf_remote_active) {
        d.estop_active = true;
        d.fault_code = NODE_FAULT_ESTOP_REMOTE;
    } else if (ultrasonic_triggered) {
        d.estop_active = true;
        d.fault_code = NODE_FAULT_ESTOP_ULTRASONIC;
    } else if (inputs->planner_error) {
        d.estop_active = true;
        d.fault_code = NODE_FAULT_ESTOP_PLANNER;
    } else if (planner_timeout) {
        d.estop_active = true;
        d.fault_code = NODE_FAULT_ESTOP_PLANNER_TIMEOUT;
    } else if (inputs->control_error) {
        d.estop_active = true;
        d.fault_code = NODE_FAULT_ESTOP_CONTROL;
    } else if (control_timeout) {
        d.estop_active = true;
        d.fault_code = NODE_FAULT_ESTOP_CONTROL_TIMEOUT;
    } else {
        // No faults detected — safe
        d.estop_active = false;
        d.fault_code = NODE_FAULT_NONE;
    }

    // Relay ON only when no e-stop
    d.relay_enable = !d.estop_active;

    return d;
}
