/**
 * @file test_system_state.c
 * @brief Unit tests for pure system state machine logic (system_state.c).
 *
 * Compile:
 *   gcc -Wall -Wextra -Werror -std=c11 -g -O0 \
 *       -I../common/system_state/include \
 *       -I../common/can_protocol/include \
 *       -o test_system_state \
 *       test_system_state.c \
 *       ../common/system_state/system_state.c
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "system_state.h"
#include "can_protocol.hh"

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name) do { \
    tests_run++; \
    printf("  [TEST] %-55s ", #name); \
    name(); \
    tests_passed++; \
    printf("PASS\n"); \
} while (0)

// ============================================================================
// Helper: returns an all-clear inputs struct (current_target=READY)
// ============================================================================

static system_state_inputs_t default_inputs(void) {
    system_state_inputs_t in = {
        .current_target = NODE_STATE_READY,
        .estop_active = false,
        .planner_state = NODE_STATE_READY,
        .control_state = NODE_STATE_READY,
        .planner_alive = true,
        .control_alive = true,
        .planner_enable_complete = false,
        .control_enable_complete = false,
        .autonomy_request = false,
    };
    return in;
}

// ============================================================================
// 1. INIT -> READY (always advances, target_changed=true)
// ============================================================================

static void test_init_to_ready(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_INIT;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == true);
}

// ============================================================================
// 2. READY -> ENABLING (both nodes READY, no estop, both alive, request=true)
// ============================================================================

static void test_ready_to_enabling(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_READY;
    in.planner_state = NODE_STATE_READY;
    in.control_state = NODE_STATE_READY;
    in.autonomy_request = true;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_ENABLING);
    assert(r.target_changed == true);
}

// ============================================================================
// 3. READY stays READY when autonomy request is not asserted
// ============================================================================

static void test_ready_stays_without_autonomy_request(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_READY;
    in.planner_state = NODE_STATE_READY;
    in.control_state = NODE_STATE_READY;
    in.autonomy_request = false;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == false);
}

// ============================================================================
// 3. READY stays READY (one node not READY — control still INIT)
// ============================================================================

static void test_ready_stays_control_not_ready(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_READY;
    in.planner_state = NODE_STATE_READY;
    in.control_state = NODE_STATE_INIT;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == false);
}

// ============================================================================
// 4. READY stays READY (planner INIT, control READY)
// ============================================================================

static void test_ready_stays_planner_not_ready(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_READY;
    in.planner_state = NODE_STATE_INIT;
    in.control_state = NODE_STATE_READY;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == false);
}

// ============================================================================
// 5. ENABLING -> ACTIVE (both ENABLING + both enable_complete)
// ============================================================================

static void test_enabling_to_active(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ENABLING;
    in.planner_state = NODE_STATE_ENABLING;
    in.control_state = NODE_STATE_ENABLING;
    in.planner_enable_complete = true;
    in.control_enable_complete = true;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_ACTIVE);
    assert(r.target_changed == true);
}

// ============================================================================
// 6. ENABLING stays ENABLING (both ENABLING, only planner enable_complete)
// ============================================================================

static void test_enabling_stays_one_complete(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ENABLING;
    in.planner_state = NODE_STATE_ENABLING;
    in.control_state = NODE_STATE_ENABLING;
    in.planner_enable_complete = true;
    in.control_enable_complete = false;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_ENABLING);
    assert(r.target_changed == false);
}

// ============================================================================
// 7. ENABLING stays ENABLING (both ENABLING, neither enable_complete)
// ============================================================================

static void test_enabling_stays_none_complete(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ENABLING;
    in.planner_state = NODE_STATE_ENABLING;
    in.control_state = NODE_STATE_ENABLING;
    in.planner_enable_complete = false;
    in.control_enable_complete = false;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_ENABLING);
    assert(r.target_changed == false);
}

// ============================================================================
// 8. ACTIVE stays ACTIVE (normal operation)
// ============================================================================

static void test_active_stays_active(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ACTIVE;
    in.planner_state = NODE_STATE_ACTIVE;
    in.control_state = NODE_STATE_ACTIVE;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_ACTIVE);
    assert(r.target_changed == false);
}

// ============================================================================
// 9. ANY -> READY on estop (from READY, ENABLING, ACTIVE)
// ============================================================================

static void test_estop_from_ready(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_READY;
    in.estop_active = true;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == false);
}

static void test_estop_from_enabling(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ENABLING;
    in.estop_active = true;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == true);
}

static void test_estop_from_active(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ACTIVE;
    in.estop_active = true;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == true);
}

// ============================================================================
// 10. ANY -> READY on planner fault (planner_state=FAULT)
// ============================================================================

static void test_planner_fault_from_active(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ACTIVE;
    in.planner_state = NODE_STATE_FAULT;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == true);
}

// ============================================================================
// 11. ANY -> READY on control fault (control_state=FAULT)
// ============================================================================

static void test_control_fault_from_active(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ACTIVE;
    in.control_state = NODE_STATE_FAULT;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == true);
}

// ============================================================================
// 12. ANY -> READY on planner override (planner_state=OVERRIDE)
// ============================================================================

static void test_planner_override_from_active(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ACTIVE;
    in.planner_state = NODE_STATE_OVERRIDE;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == true);
}

// ============================================================================
// 13. ANY -> READY on control override (control_state=OVERRIDE)
// ============================================================================

static void test_control_override_from_enabling(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ENABLING;
    in.control_state = NODE_STATE_OVERRIDE;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == true);
}

// ============================================================================
// 14. ANY -> READY on planner timeout (!planner_alive)
// ============================================================================

static void test_planner_timeout_from_active(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ACTIVE;
    in.planner_alive = false;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == true);
}

// ============================================================================
// 15. ANY -> READY on control timeout (!control_alive)
// ============================================================================

static void test_control_timeout_from_enabling(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ENABLING;
    in.control_alive = false;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == true);
}

// ============================================================================
// 16. target_changed=false when already READY and retreat condition present
// ============================================================================

static void test_already_ready_estop_no_change(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_READY;
    in.estop_active = true;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == false);
}

// ============================================================================
// 17. NULL input returns READY, target_changed=false
// ============================================================================

static void test_null_input(void) {
    system_state_result_t r = system_state_step(NULL);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == false);
}

// ============================================================================
// 18. Unknown current_target value -> retreats to READY
// ============================================================================

static void test_unknown_state_retreats(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = 99;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == true);
}

// ============================================================================
// 19. ENABLING stays ENABLING (control_enable_complete only — symmetric case)
// ============================================================================

static void test_enabling_stays_control_complete_only(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_ENABLING;
    in.planner_state = NODE_STATE_ENABLING;
    in.control_state = NODE_STATE_ENABLING;
    in.planner_enable_complete = false;
    in.control_enable_complete = true;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_ENABLING);
    assert(r.target_changed == false);
}

// ============================================================================
// 20. OVERRIDE and FAULT as current_target -> retreat to READY
// ============================================================================

static void test_override_as_current_target(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_OVERRIDE;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == true);
}

static void test_fault_as_current_target(void) {
    system_state_inputs_t in = default_inputs();
    in.current_target = NODE_STATE_FAULT;
    system_state_result_t r = system_state_step(&in);
    assert(r.new_target == NODE_STATE_READY);
    assert(r.target_changed == true);
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    printf("\n=== system_state unit tests ===\n\n");

    // Forward transitions
    TEST(test_init_to_ready);
    TEST(test_ready_to_enabling);
    TEST(test_ready_stays_without_autonomy_request);
    TEST(test_ready_stays_control_not_ready);
    TEST(test_ready_stays_planner_not_ready);
    TEST(test_enabling_to_active);
    TEST(test_enabling_stays_one_complete);
    TEST(test_enabling_stays_none_complete);
    TEST(test_enabling_stays_control_complete_only);
    TEST(test_active_stays_active);

    // Retreat: estop
    TEST(test_estop_from_ready);
    TEST(test_estop_from_enabling);
    TEST(test_estop_from_active);

    // Retreat: faults
    TEST(test_planner_fault_from_active);
    TEST(test_control_fault_from_active);

    // Retreat: override
    TEST(test_planner_override_from_active);
    TEST(test_control_override_from_enabling);

    // Retreat: timeout
    TEST(test_planner_timeout_from_active);
    TEST(test_control_timeout_from_enabling);

    // Edge cases
    TEST(test_already_ready_estop_no_change);
    TEST(test_null_input);
    TEST(test_unknown_state_retreats);
    TEST(test_override_as_current_target);
    TEST(test_fault_as_current_target);

    printf("\n  %d / %d tests passed\n\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
