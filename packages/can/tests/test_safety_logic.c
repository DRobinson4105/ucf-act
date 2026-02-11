/**
 * @file test_safety_logic.c
 * @brief Unit tests for pure safety decision logic (safety_logic.c).
 *
 * Compile:
 *   gcc -Wall -Wextra -Werror -std=c11 -g -O0 \
 *       -I../common/safety_logic/include \
 *       -I../common/can_protocol/include \
 *       -o test_safety_logic \
 *       test_safety_logic.c \
 *       ../common/safety_logic/safety_logic.c
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "safety_logic.h"
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
// Helper: returns an all-clear (safe) inputs struct
// ============================================================================

static safety_inputs_t safe_inputs(void) {
    safety_inputs_t in = {
        .push_button_active = false,
        .rf_remote_active = false,
        .ultrasonic_too_close = false,
        .ultrasonic_healthy = true,
        .planner_alive = true,
        .control_alive = true,
        .planner_error = false,
        .control_error = false,
    };
    return in;
}

// ============================================================================
// Ultrasonic trigger tests (4)
// ============================================================================

// 1. !too_close && healthy -> false (no trigger)
static void test_ultrasonic_clear(void) {
    assert(safety_compute_ultrasonic_trigger(false, true) == false);
}

// 2. too_close && healthy -> true (obstacle detected)
static void test_ultrasonic_too_close(void) {
    assert(safety_compute_ultrasonic_trigger(true, true) == true);
}

// 3. !too_close && !healthy -> true (fail-safe: sensor unhealthy)
static void test_ultrasonic_unhealthy(void) {
    assert(safety_compute_ultrasonic_trigger(false, false) == true);
}

// 4. too_close && !healthy -> true (both bad)
static void test_ultrasonic_both_bad(void) {
    assert(safety_compute_ultrasonic_trigger(true, false) == true);
}

// ============================================================================
// E-stop priority chain tests (8)
// ============================================================================

// 5. All clear -> no estop, relay on
static void test_all_clear(void) {
    safety_inputs_t in = safe_inputs();
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == false);
    assert(d.fault_code == NODE_FAULT_NONE);
    assert(d.relay_enable == true);
}

// 6. All faults active -> all bits set in fault_code (bitmask, not priority)
static void test_all_faults_bitmask(void) {
    safety_inputs_t in = {
        .push_button_active = true,
        .rf_remote_active = true,
        .ultrasonic_too_close = true,
        .ultrasonic_healthy = false,
        .planner_alive = false,
        .control_alive = false,
        .planner_error = true,
        .control_error = true,
    };
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    // All 7 bits should be set
    assert(d.fault_code & NODE_FAULT_ESTOP_BUTTON);
    assert(d.fault_code & NODE_FAULT_ESTOP_REMOTE);
    assert(d.fault_code & NODE_FAULT_ESTOP_ULTRASONIC);
    assert(d.fault_code & NODE_FAULT_ESTOP_PLANNER);
    assert(d.fault_code & NODE_FAULT_ESTOP_PLANNER_TIMEOUT);
    assert(d.fault_code & NODE_FAULT_ESTOP_CONTROL);
    assert(d.fault_code & NODE_FAULT_ESTOP_CONTROL_TIMEOUT);
}

// 7. rf_remote + ultrasonic + planner_error -> all three bits set
static void test_multiple_faults_combined(void) {
    safety_inputs_t in = safe_inputs();
    in.rf_remote_active = true;
    in.ultrasonic_too_close = true;
    in.planner_error = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.fault_code == (NODE_FAULT_ESTOP_REMOTE | NODE_FAULT_ESTOP_ULTRASONIC | NODE_FAULT_ESTOP_PLANNER));
}

// 8. ultrasonic triggered (too_close + healthy) + planner_error -> both bits set
static void test_ultrasonic_and_planner(void) {
    safety_inputs_t in = safe_inputs();
    in.ultrasonic_too_close = true;
    in.ultrasonic_healthy = true;
    in.planner_error = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.fault_code == (NODE_FAULT_ESTOP_ULTRASONIC | NODE_FAULT_ESTOP_PLANNER));
}

// 9. planner_error + planner_timeout + control_timeout -> three bits set
static void test_planner_and_control_faults(void) {
    safety_inputs_t in = safe_inputs();
    in.planner_error = true;
    in.planner_alive = false;     // planner_timeout
    in.control_alive = false;     // control_timeout
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.fault_code == (NODE_FAULT_ESTOP_PLANNER | NODE_FAULT_ESTOP_PLANNER_TIMEOUT | NODE_FAULT_ESTOP_CONTROL_TIMEOUT));
}

// 10. planner_alive=false only -> ESTOP_PLANNER_TIMEOUT
static void test_planner_timeout(void) {
    safety_inputs_t in = safe_inputs();
    in.planner_alive = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.fault_code == NODE_FAULT_ESTOP_PLANNER_TIMEOUT);
}

// 11. control_error only -> ESTOP_CONTROL
static void test_control_error(void) {
    safety_inputs_t in = safe_inputs();
    in.control_error = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.fault_code == NODE_FAULT_ESTOP_CONTROL);
}

// 12. control_alive=false only -> ESTOP_CONTROL_TIMEOUT (lowest priority)
static void test_control_timeout_lowest(void) {
    safety_inputs_t in = safe_inputs();
    in.control_alive = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.fault_code == NODE_FAULT_ESTOP_CONTROL_TIMEOUT);
}

// ============================================================================
// Ultrasonic fail-safe in evaluate (2)
// ============================================================================

// 13. ultrasonic_healthy=false (not too_close) -> estop from ultrasonic
static void test_ultrasonic_unhealthy_triggers_estop(void) {
    safety_inputs_t in = safe_inputs();
    in.ultrasonic_healthy = false;
    in.ultrasonic_too_close = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.fault_code == NODE_FAULT_ESTOP_ULTRASONIC);
}

// 14. ultrasonic_healthy=true, !too_close -> no estop from ultrasonic
static void test_ultrasonic_healthy_clear(void) {
    safety_inputs_t in = safe_inputs();
    in.ultrasonic_healthy = true;
    in.ultrasonic_too_close = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == false);
    assert(d.fault_code == NODE_FAULT_NONE);
}

// ============================================================================
// Relay output (3)
// ============================================================================

// 15. Safe -> relay_enable=true
static void test_relay_enabled_when_safe(void) {
    safety_inputs_t in = safe_inputs();
    safety_decision_t d = safety_evaluate(&in);
    assert(d.relay_enable == true);
}

// 16. Any estop -> relay_enable=false
static void test_relay_disabled_on_estop(void) {
    safety_inputs_t in = safe_inputs();
    in.push_button_active = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.relay_enable == false);
}

// 17. planner_alive=false -> relay_enable=false
static void test_relay_disabled_on_timeout(void) {
    safety_inputs_t in = safe_inputs();
    in.planner_alive = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.relay_enable == false);
}

// ============================================================================
// NULL input safety (1)
// ============================================================================

// 18. NULL pointer -> returns zeroed/default struct
static void test_evaluate_null_input(void) {
    safety_decision_t d = safety_evaluate(NULL);
    assert(d.estop_active == true);
    assert(d.fault_code == NODE_FAULT_NONE);
    assert(d.relay_enable == false);
}

// ============================================================================
// Combined scenario tests (3)
// ============================================================================

// 19. push_button + rf + ultrasonic + planner_error -> all four bits set
static void test_multiple_faults_all_bits(void) {
    safety_inputs_t in = safe_inputs();
    in.push_button_active = true;
    in.rf_remote_active = true;
    in.ultrasonic_too_close = true;
    in.planner_error = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.fault_code == (NODE_FAULT_ESTOP_BUTTON | NODE_FAULT_ESTOP_REMOTE |
                            NODE_FAULT_ESTOP_ULTRASONIC | NODE_FAULT_ESTOP_PLANNER));
    assert(d.relay_enable == false);
}

// 20. Evaluate with estop, then evaluate with all clear — verify both results
static void test_transition_estop_on_then_off(void) {
    // First: estop active
    safety_inputs_t in1 = safe_inputs();
    in1.push_button_active = true;
    safety_decision_t d1 = safety_evaluate(&in1);
    assert(d1.estop_active == true);
    assert(d1.fault_code == NODE_FAULT_ESTOP_BUTTON);
    assert(d1.relay_enable == false);

    // Second: all clear
    safety_inputs_t in2 = safe_inputs();
    safety_decision_t d2 = safety_evaluate(&in2);
    assert(d2.estop_active == false);
    assert(d2.fault_code == NODE_FAULT_NONE);
    assert(d2.relay_enable == true);
}

// 21. Only ultrasonic_healthy=false, everything else fine -> estop=true, reason=ULTRASONIC
static void test_ultrasonic_fault_alone_blocks(void) {
    safety_inputs_t in = safe_inputs();
    in.ultrasonic_healthy = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.fault_code == NODE_FAULT_ESTOP_ULTRASONIC);
    assert(d.relay_enable == false);
}

// ============================================================================
// Single-fault isolation tests (2)
// ============================================================================

// 22. rf_remote_active alone -> exactly ESTOP_REMOTE
static void test_rf_remote_alone(void) {
    safety_inputs_t in = safe_inputs();
    in.rf_remote_active = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.fault_code == NODE_FAULT_ESTOP_REMOTE);
    assert(d.relay_enable == false);
}

// 23. planner_error alone -> exactly ESTOP_PLANNER
static void test_planner_error_alone(void) {
    safety_inputs_t in = safe_inputs();
    in.planner_error = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.fault_code == NODE_FAULT_ESTOP_PLANNER);
    assert(d.relay_enable == false);
}

// ============================================================================
// Estop string helper tests (1)
// ============================================================================

// 24. Full 7-bit bitmask 0x7F -> all flags in string
static void test_full_bitmask_string(void) {
    const char *s = node_estop_to_string(0x7F);
    // Must contain all 7 flag names
    assert(strstr(s, "button") != NULL);
    assert(strstr(s, "remote") != NULL);
    assert(strstr(s, "ultrasonic") != NULL);
    assert(strstr(s, "planner_timeout") != NULL);
    assert(strstr(s, "control_timeout") != NULL);
    // "planner" appears in "planner_timeout" — verify standalone planner is present
    // by checking for "+planner+" or "planner+" at start
    assert(strstr(s, "planner+planner_timeout") != NULL);
    assert(strstr(s, "control+control_timeout") != NULL);
}

// ============================================================================
// main
// ============================================================================

int main(void) {
    printf("\n=== safety_logic unit tests ===\n\n");

    // Ultrasonic trigger (4)
    printf("--- Ultrasonic trigger ---\n");
    TEST(test_ultrasonic_clear);
    TEST(test_ultrasonic_too_close);
    TEST(test_ultrasonic_unhealthy);
    TEST(test_ultrasonic_both_bad);

    // E-stop bitmask (8)
    printf("\n--- E-stop bitmask ---\n");
    TEST(test_all_clear);
    TEST(test_all_faults_bitmask);
    TEST(test_multiple_faults_combined);
    TEST(test_ultrasonic_and_planner);
    TEST(test_planner_and_control_faults);
    TEST(test_planner_timeout);
    TEST(test_control_error);
    TEST(test_control_timeout_lowest);

    // Ultrasonic fail-safe in evaluate (2)
    printf("\n--- Ultrasonic fail-safe in evaluate ---\n");
    TEST(test_ultrasonic_unhealthy_triggers_estop);
    TEST(test_ultrasonic_healthy_clear);

    // Relay output (3)
    printf("\n--- Relay output ---\n");
    TEST(test_relay_enabled_when_safe);
    TEST(test_relay_disabled_on_estop);
    TEST(test_relay_disabled_on_timeout);

    // NULL input safety (1)
    printf("\n--- NULL input safety ---\n");
    TEST(test_evaluate_null_input);

    // Combined scenarios (3)
    printf("\n--- Combined scenarios ---\n");
    TEST(test_multiple_faults_all_bits);
    TEST(test_transition_estop_on_then_off);
    TEST(test_ultrasonic_fault_alone_blocks);

    // Single-fault isolation (2)
    printf("\n--- Single-fault isolation ---\n");
    TEST(test_rf_remote_alone);
    TEST(test_planner_error_alone);

    // Estop string helper (1)
    printf("\n--- Estop string helper ---\n");
    TEST(test_full_bitmask_string);

    printf("\n=== %d / %d tests passed ===\n\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
