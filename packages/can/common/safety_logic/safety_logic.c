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
        .estop_active = false,
        .estop_reason = ESTOP_REASON_NONE,
        .auto_allowed = false,
        .relay_enable = false,
    };

    if (!inputs) return d;

    // Compute ultrasonic trigger with fail-safe
    bool ultrasonic_triggered = safety_compute_ultrasonic_trigger(
        inputs->ultrasonic_too_close, inputs->ultrasonic_healthy);

    // Heartbeat timeouts
    bool orin_timeout = !inputs->orin_alive;
    bool control_timeout = !inputs->control_alive;

    // E-stop priority chain (highest priority first)
    if (inputs->push_button_active) {
        d.estop_active = true;
        d.estop_reason = ESTOP_REASON_MUSHROOM;
    } else if (inputs->rf_remote_active) {
        d.estop_active = true;
        d.estop_reason = ESTOP_REASON_REMOTE;
    } else if (ultrasonic_triggered) {
        d.estop_active = true;
        d.estop_reason = ESTOP_REASON_ULTRASONIC;
    } else if (inputs->orin_error) {
        d.estop_active = true;
        d.estop_reason = ESTOP_REASON_ORIN_ERROR;
    } else if (orin_timeout) {
        d.estop_active = true;
        d.estop_reason = ESTOP_REASON_ORIN_TIMEOUT;
    } else if (control_timeout) {
        d.estop_active = true;
        d.estop_reason = ESTOP_REASON_CONTROL_TIMEOUT;
    } else if (inputs->control_error) {
        d.estop_active = true;
        d.estop_reason = ESTOP_REASON_CONTROL_ERROR;
    }

    // Relay ON only when no e-stop
    d.relay_enable = !d.estop_active;

    // Autonomous allowed only when no e-stop AND all nodes alive
    d.auto_allowed = !d.estop_active && !orin_timeout && !control_timeout;

    return d;
}

safety_broadcast_t safety_should_broadcast(bool current_estop, bool last_estop,
                                            uint32_t now_ticks, uint32_t last_broadcast,
                                            uint32_t periodic_ticks) {
    safety_broadcast_t b = {
        .should_broadcast = false,
        .state_changed = (current_estop != last_estop),
    };

    if (b.state_changed) {
        b.should_broadcast = true;
    } else if (current_estop && (now_ticks - last_broadcast) >= periodic_ticks) {
        b.should_broadcast = true;
    }

    return b;
}
