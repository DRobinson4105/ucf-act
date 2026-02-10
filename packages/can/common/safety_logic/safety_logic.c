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

    // Build fault bitmask — all active faults are OR'd together
    uint8_t faults = 0;

    if (inputs->push_button_active)  faults |= NODE_FAULT_ESTOP_BUTTON;
    if (inputs->rf_remote_active)    faults |= NODE_FAULT_ESTOP_REMOTE;
    if (ultrasonic_triggered)        faults |= NODE_FAULT_ESTOP_ULTRASONIC;
    if (inputs->planner_error)       faults |= NODE_FAULT_ESTOP_PLANNER;
    if (!inputs->planner_alive)      faults |= NODE_FAULT_ESTOP_PLANNER_TIMEOUT;
    if (inputs->control_error)       faults |= NODE_FAULT_ESTOP_CONTROL;
    if (!inputs->control_alive)      faults |= NODE_FAULT_ESTOP_CONTROL_TIMEOUT;

    d.fault_code = faults;
    d.estop_active = (faults != NODE_FAULT_NONE);

    // Relay ON only when no e-stop
    d.relay_enable = !d.estop_active;

    return d;
}
