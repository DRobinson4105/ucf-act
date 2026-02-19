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

    // INIT dwell handling: hold INIT for a minimum stabilization period.
    // This is evaluated before retreat checks so Safety can complete startup
    // sequencing deterministically.
    if (current == NODE_STATE_INIT) {
        uint32_t elapsed = inputs->now_ms - inputs->boot_start_ms;
        if (elapsed >= inputs->init_dwell_ms) {
            r.new_target = NODE_STATE_READY;
            r.target_changed = true;
        } else {
            r.new_target = NODE_STATE_INIT;
            r.target_changed = false;
        }
        return r;
    }

    bool autonomy_halt = ((current == NODE_STATE_ENABLING || current == NODE_STATE_ACTIVE) &&
                          !inputs->autonomy_hold);

    // ----------------------------------------------------------------
    // Safety check: any negative condition forces retreat to READY
    // (including Planner autonomy-halt while ENABLING/ACTIVE)
    // ----------------------------------------------------------------
    if (inputs->estop_active ||
        !inputs->planner_alive ||
        !inputs->control_alive ||
        inputs->planner_state == NODE_STATE_FAULT ||
        inputs->control_state == NODE_STATE_FAULT ||
        inputs->planner_state == NODE_STATE_OVERRIDE ||
        inputs->control_state == NODE_STATE_OVERRIDE ||
        autonomy_halt) {

        r.new_target = NODE_STATE_READY;
        r.target_changed = (current != NODE_STATE_READY);
        return r;
    }

    // ----------------------------------------------------------------
    // Forward transitions (only when no negative conditions)
    // ----------------------------------------------------------------
    switch (current) {
        case NODE_STATE_READY:
            // Advance to ENABLING when both nodes are READY and
            // Planner/Orin has requested autonomy enable
            if (inputs->planner_state == NODE_STATE_READY &&
                inputs->control_state == NODE_STATE_READY &&
                inputs->autonomy_request) {
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
