/**
 * @file system_state.c
 * @brief Pure decision logic for Safety ESP32 system state machine.
 */

#include "system_state.h"
#include "can_protocol.hh"

system_state_result_t system_state_step(const system_state_inputs_t *inputs) {
    system_state_result_t r = {
        .new_target = NODE_STATE_READY,
        .target_changed = false,
    };

    if (!inputs) return r;

    uint8_t current = inputs->current_target;

    // ----------------------------------------------------------------
    // Safety check: any negative condition forces retreat to READY
    // ----------------------------------------------------------------
    if (inputs->estop_active ||
        !inputs->planner_alive ||
        !inputs->control_alive ||
        inputs->planner_state == NODE_STATE_FAULT ||
        inputs->control_state == NODE_STATE_FAULT ||
        inputs->planner_state == NODE_STATE_OVERRIDE ||
        inputs->control_state == NODE_STATE_OVERRIDE) {

        r.new_target = NODE_STATE_READY;
        r.target_changed = (current != NODE_STATE_READY);
        return r;
    }

    // ----------------------------------------------------------------
    // Forward transitions (only when no negative conditions)
    // ----------------------------------------------------------------
    switch (current) {
        case NODE_STATE_INIT:
            // Safety always advances from INIT to READY
            r.new_target = NODE_STATE_READY;
            r.target_changed = true;
            break;

        case NODE_STATE_READY:
            // Advance to ENABLING when both nodes are READY
            if (inputs->planner_state == NODE_STATE_READY &&
                inputs->control_state == NODE_STATE_READY) {
                r.new_target = NODE_STATE_ENABLING;
                r.target_changed = true;
            } else {
                r.new_target = NODE_STATE_READY;
                r.target_changed = false;
            }
            break;

        case NODE_STATE_ENABLING:
            // Advance to ACTIVE when both nodes are ENABLING and
            // both have signaled enable_complete
            if (inputs->planner_state == NODE_STATE_ENABLING &&
                inputs->control_state == NODE_STATE_ENABLING &&
                inputs->planner_enable_complete &&
                inputs->control_enable_complete) {
                r.new_target = NODE_STATE_ACTIVE;
                r.target_changed = true;
            } else {
                // Stay in ENABLING — nodes are still working
                r.new_target = NODE_STATE_ENABLING;
                r.target_changed = false;
            }
            break;

        case NODE_STATE_ACTIVE:
            // Stay in ACTIVE — no further forward transition
            r.new_target = NODE_STATE_ACTIVE;
            r.target_changed = false;
            break;

        default:
            // Unknown state — retreat to READY
            r.new_target = NODE_STATE_READY;
            r.target_changed = (current != NODE_STATE_READY);
            break;
    }

    return r;
}
