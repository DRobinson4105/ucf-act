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
        .orin_alive = true,
        .control_alive = true,
        .orin_error = false,
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

// 5. All clear -> no estop, auto allowed, relay on
static void test_all_clear(void) {
    safety_inputs_t in = safe_inputs();
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == false);
    assert(d.estop_reason == ESTOP_REASON_NONE);
    assert(d.auto_allowed == true);
    assert(d.relay_enable == true);
}

// 6. All faults active + push_button -> MUSHROOM (highest priority wins)
static void test_push_button_highest_priority(void) {
    safety_inputs_t in = {
        .push_button_active = true,
        .rf_remote_active = true,
        .ultrasonic_too_close = true,
        .ultrasonic_healthy = false,
        .orin_alive = false,
        .control_alive = false,
        .orin_error = true,
        .control_error = true,
    };
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.estop_reason == ESTOP_REASON_MUSHROOM);
}

// 7. rf_remote + ultrasonic + orin_error -> REMOTE
static void test_rf_remote_priority(void) {
    safety_inputs_t in = safe_inputs();
    in.rf_remote_active = true;
    in.ultrasonic_too_close = true;
    in.orin_error = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.estop_reason == ESTOP_REASON_REMOTE);
}

// 8. ultrasonic triggered (too_close + healthy) + orin_error -> ULTRASONIC
static void test_ultrasonic_priority(void) {
    safety_inputs_t in = safe_inputs();
    in.ultrasonic_too_close = true;
    in.ultrasonic_healthy = true;
    in.orin_error = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.estop_reason == ESTOP_REASON_ULTRASONIC);
}

// 9. orin_error + orin_timeout + control_timeout -> ORIN_ERROR
static void test_orin_error_priority(void) {
    safety_inputs_t in = safe_inputs();
    in.orin_error = true;
    in.orin_alive = false;    // orin_timeout
    in.control_alive = false; // control_timeout
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.estop_reason == ESTOP_REASON_ORIN_ERROR);
}

// 10. orin_alive=false only -> ORIN_TIMEOUT
static void test_orin_timeout(void) {
    safety_inputs_t in = safe_inputs();
    in.orin_alive = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.estop_reason == ESTOP_REASON_ORIN_TIMEOUT);
}

// 11. control_alive=false only -> CONTROL_TIMEOUT
static void test_control_timeout(void) {
    safety_inputs_t in = safe_inputs();
    in.control_alive = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.estop_reason == ESTOP_REASON_CONTROL_TIMEOUT);
}

// 12. control_error only -> CONTROL_ERROR (lowest priority)
static void test_control_error_lowest(void) {
    safety_inputs_t in = safe_inputs();
    in.control_error = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.estop_reason == ESTOP_REASON_CONTROL_ERROR);
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
    assert(d.estop_reason == ESTOP_REASON_ULTRASONIC);
}

// 14. ultrasonic_healthy=true, !too_close -> no estop from ultrasonic
static void test_ultrasonic_healthy_clear(void) {
    safety_inputs_t in = safe_inputs();
    in.ultrasonic_healthy = true;
    in.ultrasonic_too_close = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == false);
    assert(d.estop_reason == ESTOP_REASON_NONE);
}

// ============================================================================
// Auto-allowed logic (4)
// ============================================================================

// 15. Safe inputs -> auto_allowed=true
static void test_auto_allowed_when_all_clear(void) {
    safety_inputs_t in = safe_inputs();
    safety_decision_t d = safety_evaluate(&in);
    assert(d.auto_allowed == true);
}

// 16. push_button active -> auto_allowed=false
static void test_auto_blocked_by_estop(void) {
    safety_inputs_t in = safe_inputs();
    in.push_button_active = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.auto_allowed == false);
}

// 17. orin_alive=false -> auto_allowed=false (also triggers estop)
static void test_auto_blocked_by_orin_timeout(void) {
    safety_inputs_t in = safe_inputs();
    in.orin_alive = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.auto_allowed == false);
    assert(d.estop_active == true);
}

// 18. control_alive=false -> auto_allowed=false
static void test_auto_blocked_by_control_timeout(void) {
    safety_inputs_t in = safe_inputs();
    in.control_alive = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.auto_allowed == false);
    assert(d.estop_active == true);
}

// ============================================================================
// Relay output (3)
// ============================================================================

// 19. Safe -> relay_enable=true
static void test_relay_enabled_when_safe(void) {
    safety_inputs_t in = safe_inputs();
    safety_decision_t d = safety_evaluate(&in);
    assert(d.relay_enable == true);
}

// 20. Any estop -> relay_enable=false
static void test_relay_disabled_on_estop(void) {
    safety_inputs_t in = safe_inputs();
    in.push_button_active = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.relay_enable == false);
}

// 21. orin_alive=false -> relay_enable=false
static void test_relay_disabled_on_timeout(void) {
    safety_inputs_t in = safe_inputs();
    in.orin_alive = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.relay_enable == false);
}

// ============================================================================
// NULL input safety (1)
// ============================================================================

// 22. NULL pointer -> returns zeroed/default struct
static void test_evaluate_null_input(void) {
    safety_decision_t d = safety_evaluate(NULL);
    assert(d.estop_active == false);
    assert(d.estop_reason == ESTOP_REASON_NONE);
    assert(d.auto_allowed == false);
    assert(d.relay_enable == false);
}

// ============================================================================
// Broadcast timing tests (5)
// ============================================================================

// 23. false->true (state change activate) -> should_broadcast=true, state_changed=true
static void test_broadcast_on_state_change_activate(void) {
    safety_broadcast_t b = safety_should_broadcast(true, false, 1000, 0, 500);
    assert(b.should_broadcast == true);
    assert(b.state_changed == true);
}

// 24. true->false (state change deactivate) -> should_broadcast=true, state_changed=true
static void test_broadcast_on_state_change_deactivate(void) {
    safety_broadcast_t b = safety_should_broadcast(false, true, 1000, 0, 500);
    assert(b.should_broadcast == true);
    assert(b.state_changed == true);
}

// 25. estop=true, same state, elapsed>=periodic -> should_broadcast=true, state_changed=false
static void test_broadcast_periodic_during_estop(void) {
    safety_broadcast_t b = safety_should_broadcast(true, true, 1500, 1000, 500);
    assert(b.should_broadcast == true);
    assert(b.state_changed == false);
}

// 26. estop=false, same state -> should_broadcast=false
static void test_no_broadcast_when_stable_safe(void) {
    safety_broadcast_t b = safety_should_broadcast(false, false, 5000, 1000, 500);
    assert(b.should_broadcast == false);
    assert(b.state_changed == false);
}

// 27. estop=true, same state, elapsed<periodic -> should_broadcast=false
static void test_no_broadcast_periodic_too_soon(void) {
    safety_broadcast_t b = safety_should_broadcast(true, true, 1200, 1000, 500);
    assert(b.should_broadcast == false);
    assert(b.state_changed == false);
}

// ============================================================================
// Combined scenario tests (3)
// ============================================================================

// 28. push_button + rf + ultrasonic + orin_error -> MUSHROOM (highest wins)
static void test_multiple_faults_highest_wins(void) {
    safety_inputs_t in = safe_inputs();
    in.push_button_active = true;
    in.rf_remote_active = true;
    in.ultrasonic_too_close = true;
    in.orin_error = true;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.estop_reason == ESTOP_REASON_MUSHROOM);
    assert(d.auto_allowed == false);
    assert(d.relay_enable == false);
}

// 29. Evaluate with estop, then evaluate with all clear — verify both results
static void test_transition_estop_on_then_off(void) {
    // First: estop active
    safety_inputs_t in1 = safe_inputs();
    in1.push_button_active = true;
    safety_decision_t d1 = safety_evaluate(&in1);
    assert(d1.estop_active == true);
    assert(d1.estop_reason == ESTOP_REASON_MUSHROOM);
    assert(d1.auto_allowed == false);
    assert(d1.relay_enable == false);

    // Second: all clear
    safety_inputs_t in2 = safe_inputs();
    safety_decision_t d2 = safety_evaluate(&in2);
    assert(d2.estop_active == false);
    assert(d2.estop_reason == ESTOP_REASON_NONE);
    assert(d2.auto_allowed == true);
    assert(d2.relay_enable == true);
}

// 30. Only ultrasonic_healthy=false, everything else fine -> estop=true, reason=ULTRASONIC
static void test_ultrasonic_fault_alone_blocks(void) {
    safety_inputs_t in = safe_inputs();
    in.ultrasonic_healthy = false;
    safety_decision_t d = safety_evaluate(&in);
    assert(d.estop_active == true);
    assert(d.estop_reason == ESTOP_REASON_ULTRASONIC);
    assert(d.auto_allowed == false);
    assert(d.relay_enable == false);
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

    // E-stop priority chain (8)
    printf("\n--- E-stop priority chain ---\n");
    TEST(test_all_clear);
    TEST(test_push_button_highest_priority);
    TEST(test_rf_remote_priority);
    TEST(test_ultrasonic_priority);
    TEST(test_orin_error_priority);
    TEST(test_orin_timeout);
    TEST(test_control_timeout);
    TEST(test_control_error_lowest);

    // Ultrasonic fail-safe in evaluate (2)
    printf("\n--- Ultrasonic fail-safe in evaluate ---\n");
    TEST(test_ultrasonic_unhealthy_triggers_estop);
    TEST(test_ultrasonic_healthy_clear);

    // Auto-allowed logic (4)
    printf("\n--- Auto-allowed logic ---\n");
    TEST(test_auto_allowed_when_all_clear);
    TEST(test_auto_blocked_by_estop);
    TEST(test_auto_blocked_by_orin_timeout);
    TEST(test_auto_blocked_by_control_timeout);

    // Relay output (3)
    printf("\n--- Relay output ---\n");
    TEST(test_relay_enabled_when_safe);
    TEST(test_relay_disabled_on_estop);
    TEST(test_relay_disabled_on_timeout);

    // NULL input safety (1)
    printf("\n--- NULL input safety ---\n");
    TEST(test_evaluate_null_input);

    // Broadcast timing (5)
    printf("\n--- Broadcast timing ---\n");
    TEST(test_broadcast_on_state_change_activate);
    TEST(test_broadcast_on_state_change_deactivate);
    TEST(test_broadcast_periodic_during_estop);
    TEST(test_no_broadcast_when_stable_safe);
    TEST(test_no_broadcast_periodic_too_soon);

    // Combined scenarios (3)
    printf("\n--- Combined scenarios ---\n");
    TEST(test_multiple_faults_highest_wins);
    TEST(test_transition_estop_on_then_off);
    TEST(test_ultrasonic_fault_alone_blocks);

    printf("\n=== %d / %d tests passed ===\n\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
