/**
 * @file test_control_logic.c
 * @brief Unit tests for pure control decision logic (control_logic.c).
 *
 * Compile:
 *   gcc -Wall -Wextra -Werror -std=c11 -g -O0 \
 *       -I../common/control_logic/include \
 *       -I../common/can_protocol/include \
 *       -o test_control_logic \
 *       test_control_logic.c \
 *       ../common/control_logic/control_logic.c
 */

#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>

#include "control_logic.h"
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
// Helper: returns a default "all clear" control_inputs_t
// ============================================================================

static control_inputs_t default_inputs(void) {
    control_inputs_t in;
    memset(&in, 0, sizeof(in));
    in.auto_allowed = true;
    in.fr_state = CONTROL_FR_FORWARD;
    in.pedal_pressed = false;
    in.pedal_rearmed = true;
    in.enable_sequence_ms = 200;
    in.throttle_slew_interval_ms = 100;
    in.last_steering_sent = INT16_MIN;
    in.last_braking_sent = INT16_MIN;
    return in;
}

// ============================================================================
// Precondition tests (6)
// ============================================================================

// 1. All good -> true
static void test_preconditions_all_good(void) {
    precondition_inputs_t p = {
        .fr_state = CONTROL_FR_FORWARD,
        .pedal_pressed = false,
        .pedal_rearmed = true,
        .fault_code = CONTROL_FAULT_NONE,
    };
    assert(control_check_preconditions(&p) == true);
}

// 2. FR not forward -> false
static void test_preconditions_fr_not_forward(void) {
    precondition_inputs_t p = {
        .fr_state = CONTROL_FR_REVERSE,
        .pedal_pressed = false,
        .pedal_rearmed = true,
        .fault_code = CONTROL_FAULT_NONE,
    };
    assert(control_check_preconditions(&p) == false);
}

// 3. Pedal pressed -> false
static void test_preconditions_pedal_pressed(void) {
    precondition_inputs_t p = {
        .fr_state = CONTROL_FR_FORWARD,
        .pedal_pressed = true,
        .pedal_rearmed = true,
        .fault_code = CONTROL_FAULT_NONE,
    };
    assert(control_check_preconditions(&p) == false);
}

// 4. Pedal not rearmed -> false
static void test_preconditions_pedal_not_rearmed(void) {
    precondition_inputs_t p = {
        .fr_state = CONTROL_FR_FORWARD,
        .pedal_pressed = false,
        .pedal_rearmed = false,
        .fault_code = CONTROL_FAULT_NONE,
    };
    assert(control_check_preconditions(&p) == false);
}

// 5. Active fault -> false
static void test_preconditions_active_fault(void) {
    precondition_inputs_t p = {
        .fr_state = CONTROL_FR_FORWARD,
        .pedal_pressed = false,
        .pedal_rearmed = true,
        .fault_code = CONTROL_FAULT_MOTOR_COMM,
    };
    assert(control_check_preconditions(&p) == false);
}

// 6. NULL input -> false
static void test_preconditions_null(void) {
    assert(control_check_preconditions(NULL) == false);
}

// ============================================================================
// Throttle slew tests (4)
// ============================================================================

// 7. At target -> no change
static void test_slew_at_target(void) {
    throttle_slew_inputs_t s = {
        .current = 3, .target = 3, .last_change_ms = 0,
        .now_ms = 1000, .slew_interval_ms = 100,
    };
    throttle_slew_result_t r = control_compute_throttle_slew(&s);
    assert(r.new_level == 3);
    assert(r.changed == false);
}

// 8. Below target + elapsed -> step up
static void test_slew_step_up(void) {
    throttle_slew_inputs_t s = {
        .current = 2, .target = 5, .last_change_ms = 0,
        .now_ms = 100, .slew_interval_ms = 100,
    };
    throttle_slew_result_t r = control_compute_throttle_slew(&s);
    assert(r.new_level == 3);
    assert(r.changed == true);
}

// 9. Above target + elapsed -> step down
static void test_slew_step_down(void) {
    throttle_slew_inputs_t s = {
        .current = 5, .target = 2, .last_change_ms = 0,
        .now_ms = 100, .slew_interval_ms = 100,
    };
    throttle_slew_result_t r = control_compute_throttle_slew(&s);
    assert(r.new_level == 4);
    assert(r.changed == true);
}

// 10. Below target but too soon -> no change
static void test_slew_too_soon(void) {
    throttle_slew_inputs_t s = {
        .current = 2, .target = 5, .last_change_ms = 50,
        .now_ms = 100, .slew_interval_ms = 100,
    };
    throttle_slew_result_t r = control_compute_throttle_slew(&s);
    assert(r.new_level == 2);
    assert(r.changed == false);
}

// ============================================================================
// State: INIT -> READY (2)
// ============================================================================

// 11. INIT always transitions to READY
static void test_init_to_ready(void) {
    control_inputs_t in = default_inputs();
    control_step_result_t r = control_compute_step(CONTROL_STATE_INIT, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_READY);
    assert(r.actions == CONTROL_ACTION_NONE);
}

// 12. INIT -> READY even if auto not allowed
static void test_init_to_ready_regardless(void) {
    control_inputs_t in = default_inputs();
    in.auto_allowed = false;
    control_step_result_t r = control_compute_step(CONTROL_STATE_INIT, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_READY);
}

// ============================================================================
// State: READY -> ENABLING (3)
// ============================================================================

// 13. READY + auto_allowed + preconditions met -> ENABLING
static void test_ready_to_enabling(void) {
    control_inputs_t in = default_inputs();
    in.now_ms = 5000;
    control_step_result_t r = control_compute_step(CONTROL_STATE_READY, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_ENABLING);
    assert(r.actions & CONTROL_ACTION_START_ENABLE);
    assert(r.enable_start_ms == 5000);
    assert(r.throttle_level == 0);
}

// 14. READY + auto_allowed but pedal pressed -> stays READY
static void test_ready_stays_pedal_pressed(void) {
    control_inputs_t in = default_inputs();
    in.pedal_pressed = true;
    control_step_result_t r = control_compute_step(CONTROL_STATE_READY, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_READY);
    assert(r.actions == CONTROL_ACTION_NONE);
}

// 15. READY + auto not allowed -> stays READY
static void test_ready_stays_no_auto(void) {
    control_inputs_t in = default_inputs();
    in.auto_allowed = false;
    control_step_result_t r = control_compute_step(CONTROL_STATE_READY, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_READY);
    assert(r.actions == CONTROL_ACTION_NONE);
}

// 16. READY + FR not forward -> stays READY
static void test_ready_stays_fr_wrong(void) {
    control_inputs_t in = default_inputs();
    in.fr_state = CONTROL_FR_REVERSE;
    control_step_result_t r = control_compute_step(CONTROL_STATE_READY, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_READY);
}

// ============================================================================
// State: ENABLING -> ACTIVE (3)
// ============================================================================

// 17. ENABLING + timer expired -> ACTIVE + COMPLETE_ENABLE
static void test_enabling_to_active(void) {
    control_inputs_t in = default_inputs();
    in.enable_start_ms = 1000;
    in.now_ms = 1200;  // >= 200ms enable_sequence_ms
    control_step_result_t r = control_compute_step(CONTROL_STATE_ENABLING, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_ACTIVE);
    assert(r.actions & CONTROL_ACTION_COMPLETE_ENABLE);
    assert(r.override_reason == OVERRIDE_REASON_NONE);
}

// 18. ENABLING + timer not expired -> stays ENABLING
static void test_enabling_stays_timer(void) {
    control_inputs_t in = default_inputs();
    in.enable_start_ms = 1000;
    in.now_ms = 1100;  // < 200ms
    control_step_result_t r = control_compute_step(CONTROL_STATE_ENABLING, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_ENABLING);
    assert(r.actions == CONTROL_ACTION_NONE);
}

// 19. ENABLING at exact boundary -> ACTIVE
static void test_enabling_exact_boundary(void) {
    control_inputs_t in = default_inputs();
    in.enable_start_ms = 1000;
    in.now_ms = 1200;  // exactly 200ms
    control_step_result_t r = control_compute_step(CONTROL_STATE_ENABLING, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_ACTIVE);
}

// ============================================================================
// State: ENABLING -> READY abort (4)
// ============================================================================

// 20. ENABLING + auto blocked -> READY + ABORT_ENABLE
static void test_enabling_abort_no_auto(void) {
    control_inputs_t in = default_inputs();
    in.auto_allowed = false;
    in.enable_start_ms = 1000;
    in.now_ms = 1050;
    control_step_result_t r = control_compute_step(CONTROL_STATE_ENABLING, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_READY);
    assert(r.actions & CONTROL_ACTION_ABORT_ENABLE);
}

// 21. ENABLING + pedal pressed -> READY + ABORT_ENABLE
static void test_enabling_abort_pedal(void) {
    control_inputs_t in = default_inputs();
    in.pedal_pressed = true;
    in.enable_start_ms = 1000;
    in.now_ms = 1050;
    control_step_result_t r = control_compute_step(CONTROL_STATE_ENABLING, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_READY);
    assert(r.actions & CONTROL_ACTION_ABORT_ENABLE);
}

// 22. ENABLING + FR not forward -> READY + ABORT_ENABLE
static void test_enabling_abort_fr(void) {
    control_inputs_t in = default_inputs();
    in.fr_state = CONTROL_FR_NEUTRAL;
    in.enable_start_ms = 1000;
    in.now_ms = 1050;
    control_step_result_t r = control_compute_step(CONTROL_STATE_ENABLING, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_READY);
    assert(r.actions & CONTROL_ACTION_ABORT_ENABLE);
}

// 23. ENABLING abort priority: auto check first (even if timer expired)
static void test_enabling_abort_priority(void) {
    control_inputs_t in = default_inputs();
    in.auto_allowed = false;
    in.enable_start_ms = 1000;
    in.now_ms = 1500;  // timer would have expired
    control_step_result_t r = control_compute_step(CONTROL_STATE_ENABLING, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_READY);
    assert(r.actions & CONTROL_ACTION_ABORT_ENABLE);
}

// ============================================================================
// State: ACTIVE override triggers (4)
// ============================================================================

// 24. ACTIVE + pedal pressed -> OVERRIDE + TRIGGER_OVERRIDE
static void test_active_override_pedal(void) {
    control_inputs_t in = default_inputs();
    in.pedal_pressed = true;
    in.throttle_current = 3;
    control_step_result_t r = control_compute_step(CONTROL_STATE_ACTIVE, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_OVERRIDE);
    assert(r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE);
    assert(r.override_reason == OVERRIDE_REASON_PEDAL);
    assert(r.throttle_level == 0);
    assert(r.new_last_steering == INT16_MIN);
    assert(r.new_last_braking == INT16_MIN);
}

// 25. ACTIVE + FR changed -> OVERRIDE
static void test_active_override_fr(void) {
    control_inputs_t in = default_inputs();
    in.fr_state = CONTROL_FR_REVERSE;
    control_step_result_t r = control_compute_step(CONTROL_STATE_ACTIVE, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_OVERRIDE);
    assert(r.override_reason == OVERRIDE_REASON_FR_CHANGED);
}

// 26. ACTIVE + auto blocked -> OVERRIDE
static void test_active_override_no_auto(void) {
    control_inputs_t in = default_inputs();
    in.auto_allowed = false;
    control_step_result_t r = control_compute_step(CONTROL_STATE_ACTIVE, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_OVERRIDE);
    assert(r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE);
    assert(r.override_reason == OVERRIDE_REASON_NONE);
}

// 27. ACTIVE override priority: pedal > FR > auto
static void test_active_override_priority(void) {
    control_inputs_t in = default_inputs();
    in.pedal_pressed = true;
    in.fr_state = CONTROL_FR_REVERSE;
    in.auto_allowed = false;
    control_step_result_t r = control_compute_step(CONTROL_STATE_ACTIVE, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_OVERRIDE);
    assert(r.override_reason == OVERRIDE_REASON_PEDAL);
}

// ============================================================================
// State: ACTIVE throttle + steering (4)
// ============================================================================

// 28. ACTIVE: throttle slew triggers APPLY_THROTTLE
static void test_active_throttle_slew(void) {
    control_inputs_t in = default_inputs();
    in.throttle_current = 2;
    in.throttle_target = 5;
    in.last_throttle_change_ms = 0;
    in.now_ms = 100;
    control_step_result_t r = control_compute_step(CONTROL_STATE_ACTIVE, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_ACTIVE);
    assert(r.actions & CONTROL_ACTION_APPLY_THROTTLE);
    assert(r.throttle_level == 3);  // one step toward target
    assert(r.throttle_change_ms == 100);
}

// 29. ACTIVE: throttle at target -> no APPLY_THROTTLE
static void test_active_throttle_at_target(void) {
    control_inputs_t in = default_inputs();
    in.throttle_current = 5;
    in.throttle_target = 5;
    in.now_ms = 1000;
    control_step_result_t r = control_compute_step(CONTROL_STATE_ACTIVE, CONTROL_FAULT_NONE, &in);
    assert(!(r.actions & CONTROL_ACTION_APPLY_THROTTLE));
}

// 30. ACTIVE: steering cmd changed -> send_steering=true
static void test_active_steering_changed(void) {
    control_inputs_t in = default_inputs();
    in.steering_cmd = 1000;
    in.last_steering_sent = 500;
    in.braking_cmd = 200;
    in.last_braking_sent = 200;  // unchanged
    control_step_result_t r = control_compute_step(CONTROL_STATE_ACTIVE, CONTROL_FAULT_NONE, &in);
    assert(r.send_steering == true);
    assert(r.steering_position == 1000);
    assert(r.new_last_steering == 1000);
    assert(r.send_braking == false);
}

// 31. ACTIVE: braking cmd changed -> send_braking=true
static void test_active_braking_changed(void) {
    control_inputs_t in = default_inputs();
    in.steering_cmd = 500;
    in.last_steering_sent = 500;
    in.braking_cmd = 300;
    in.last_braking_sent = 100;
    control_step_result_t r = control_compute_step(CONTROL_STATE_ACTIVE, CONTROL_FAULT_NONE, &in);
    assert(r.send_braking == true);
    assert(r.braking_position == 300);
    assert(r.new_last_braking == 300);
    assert(r.send_steering == false);
}

// ============================================================================
// State: OVERRIDE -> READY (3)
// ============================================================================

// 32. OVERRIDE + conditions met -> READY
static void test_override_to_ready(void) {
    control_inputs_t in = default_inputs();
    in.auto_allowed = true;
    in.fr_state = CONTROL_FR_FORWARD;
    in.pedal_rearmed = true;
    control_step_result_t r = control_compute_step(CONTROL_STATE_OVERRIDE, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_READY);
    assert(r.override_reason == OVERRIDE_REASON_NONE);
}

// 33. OVERRIDE + auto blocked -> stays OVERRIDE
static void test_override_stays_no_auto(void) {
    control_inputs_t in = default_inputs();
    in.auto_allowed = false;
    control_step_result_t r = control_compute_step(CONTROL_STATE_OVERRIDE, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_OVERRIDE);
}

// 34. OVERRIDE + pedal not rearmed -> stays OVERRIDE
static void test_override_stays_pedal_not_rearmed(void) {
    control_inputs_t in = default_inputs();
    in.pedal_rearmed = false;
    control_step_result_t r = control_compute_step(CONTROL_STATE_OVERRIDE, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_OVERRIDE);
}

// 35. OVERRIDE resets dedup trackers
static void test_override_resets_trackers(void) {
    control_inputs_t in = default_inputs();
    in.auto_allowed = false;
    in.last_steering_sent = 1000;
    in.last_braking_sent = 2000;
    in.throttle_current = 5;
    control_step_result_t r = control_compute_step(CONTROL_STATE_OVERRIDE, CONTROL_FAULT_NONE, &in);
    assert(r.throttle_level == 0);
    assert(r.new_last_steering == INT16_MIN);
    assert(r.new_last_braking == INT16_MIN);
}

// ============================================================================
// State: FAULT -> recovery (3)
// ============================================================================

// 36. FAULT -> signals ATTEMPT_RECOVERY
static void test_fault_attempts_recovery(void) {
    control_inputs_t in = default_inputs();
    control_step_result_t r = control_compute_step(CONTROL_STATE_FAULT, CONTROL_FAULT_MOTOR_COMM, &in);
    assert(r.new_state == CONTROL_STATE_FAULT);
    assert(r.actions & CONTROL_ACTION_ATTEMPT_RECOVERY);
    assert(r.throttle_level == 0);
}

// 37. FAULT resets dedup trackers
static void test_fault_resets_trackers(void) {
    control_inputs_t in = default_inputs();
    in.last_steering_sent = 1000;
    in.last_braking_sent = 2000;
    control_step_result_t r = control_compute_step(CONTROL_STATE_FAULT, CONTROL_FAULT_MOTOR_COMM, &in);
    assert(r.new_last_steering == INT16_MIN);
    assert(r.new_last_braking == INT16_MIN);
}

// 38. FAULT preserves fault code
static void test_fault_preserves_code(void) {
    control_inputs_t in = default_inputs();
    control_step_result_t r = control_compute_step(CONTROL_STATE_FAULT, CONTROL_FAULT_SENSOR_INVALID, &in);
    assert(r.new_fault_code == CONTROL_FAULT_SENSOR_INVALID);
}

// ============================================================================
// Motor fault injection (2)
// ============================================================================

// 39. Motor fault from ACTIVE -> OVERRIDE + FAULT
static void test_motor_fault_from_active(void) {
    control_inputs_t in = default_inputs();
    in.motor_fault_code = CONTROL_FAULT_MOTOR_COMM;
    control_step_result_t r = control_compute_step(CONTROL_STATE_ACTIVE, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_FAULT);
    assert(r.new_fault_code == CONTROL_FAULT_MOTOR_COMM);
    assert(r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE);
}

// 40. Motor fault from READY -> FAULT (no override)
static void test_motor_fault_from_ready(void) {
    control_inputs_t in = default_inputs();
    in.motor_fault_code = CONTROL_FAULT_MOTOR_COMM;
    control_step_result_t r = control_compute_step(CONTROL_STATE_READY, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_FAULT);
    assert(r.new_fault_code == CONTROL_FAULT_MOTOR_COMM);
    assert(!(r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE));
}

// ============================================================================
// FR INVALID sensor fault (2)
// ============================================================================

// 41. FR invalid from ACTIVE -> OVERRIDE + FAULT
static void test_fr_invalid_from_active(void) {
    control_inputs_t in = default_inputs();
    in.fr_is_invalid = true;
    control_step_result_t r = control_compute_step(CONTROL_STATE_ACTIVE, CONTROL_FAULT_NONE, &in);
    assert(r.new_state == CONTROL_STATE_FAULT);
    assert(r.new_fault_code == CONTROL_FAULT_SENSOR_INVALID);
    assert(r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE);
    assert(r.override_reason == OVERRIDE_REASON_FR_CHANGED);
}

// 42. FR invalid already in FAULT -> no double-fault
static void test_fr_invalid_already_faulted(void) {
    control_inputs_t in = default_inputs();
    in.fr_is_invalid = true;
    control_step_result_t r = control_compute_step(CONTROL_STATE_FAULT, CONTROL_FAULT_SENSOR_INVALID, &in);
    // Should stay in FAULT with recovery action, not re-trigger
    assert(r.new_state == CONTROL_STATE_FAULT);
    assert(r.actions & CONTROL_ACTION_ATTEMPT_RECOVERY);
    assert(!(r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE));
}

// ============================================================================
// CAN TX tracking (3)
// ============================================================================

// 43. TX OK resets count
static void test_can_tx_ok_resets(void) {
    can_tx_track_inputs_t t = { .fail_count = 3, .threshold = 5, .tx_ok = true };
    can_tx_track_result_t r = control_track_can_tx(&t);
    assert(r.new_fail_count == 0);
    assert(r.trigger_recovery == false);
}

// 44. TX fail below threshold -> increment
static void test_can_tx_fail_below_threshold(void) {
    can_tx_track_inputs_t t = { .fail_count = 2, .threshold = 5, .tx_ok = false };
    can_tx_track_result_t r = control_track_can_tx(&t);
    assert(r.new_fail_count == 3);
    assert(r.trigger_recovery == false);
}

// 45. TX fail at threshold -> trigger recovery
static void test_can_tx_fail_at_threshold(void) {
    can_tx_track_inputs_t t = { .fail_count = 4, .threshold = 5, .tx_ok = false };
    can_tx_track_result_t r = control_track_can_tx(&t);
    assert(r.new_fail_count == 5);
    assert(r.trigger_recovery == true);
}

// ============================================================================
// NULL input safety (1)
// ============================================================================

// 46. NULL inputs -> safe defaults
static void test_compute_step_null(void) {
    control_step_result_t r = control_compute_step(CONTROL_STATE_READY, CONTROL_FAULT_NONE, NULL);
    assert(r.new_state == CONTROL_STATE_READY);
    assert(r.actions == CONTROL_ACTION_NONE);
}

// ============================================================================
// Full scenario: READY -> ENABLING -> ACTIVE -> OVERRIDE -> READY (1)
// ============================================================================

// 47. Walk through the full lifecycle
static void test_full_lifecycle(void) {
    control_inputs_t in = default_inputs();
    control_step_result_t r;
    uint8_t state = CONTROL_STATE_READY;
    uint8_t fault = CONTROL_FAULT_NONE;

    // Step 1: READY -> ENABLING
    in.now_ms = 1000;
    r = control_compute_step(state, fault, &in);
    assert(r.new_state == CONTROL_STATE_ENABLING);
    assert(r.actions & CONTROL_ACTION_START_ENABLE);
    state = r.new_state;
    uint32_t enable_start = r.enable_start_ms;

    // Step 2: ENABLING (not yet expired)
    in.now_ms = 1100;
    in.enable_start_ms = enable_start;
    r = control_compute_step(state, fault, &in);
    assert(r.new_state == CONTROL_STATE_ENABLING);

    // Step 3: ENABLING -> ACTIVE (timer expired)
    in.now_ms = 1200;
    r = control_compute_step(state, fault, &in);
    assert(r.new_state == CONTROL_STATE_ACTIVE);
    assert(r.actions & CONTROL_ACTION_COMPLETE_ENABLE);
    state = r.new_state;

    // Step 4: ACTIVE with throttle command
    in.now_ms = 1300;
    in.throttle_current = 0;
    in.throttle_target = 3;
    in.last_throttle_change_ms = 1200;
    r = control_compute_step(state, fault, &in);
    assert(r.new_state == CONTROL_STATE_ACTIVE);
    assert(r.actions & CONTROL_ACTION_APPLY_THROTTLE);
    assert(r.throttle_level == 1);

    // Step 5: Override by pedal
    in.pedal_pressed = true;
    r = control_compute_step(state, fault, &in);
    assert(r.new_state == CONTROL_STATE_OVERRIDE);
    assert(r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE);
    assert(r.override_reason == OVERRIDE_REASON_PEDAL);
    state = r.new_state;

    // Step 6: Override clears -> READY
    in.pedal_pressed = false;
    in.pedal_rearmed = true;
    r = control_compute_step(state, fault, &in);
    assert(r.new_state == CONTROL_STATE_READY);
}

// ============================================================================
// main
// ============================================================================

int main(void) {
    printf("\n=== control_logic unit tests ===\n\n");

    // Preconditions (6)
    printf("--- Preconditions ---\n");
    TEST(test_preconditions_all_good);
    TEST(test_preconditions_fr_not_forward);
    TEST(test_preconditions_pedal_pressed);
    TEST(test_preconditions_pedal_not_rearmed);
    TEST(test_preconditions_active_fault);
    TEST(test_preconditions_null);

    // Throttle slew (4)
    printf("\n--- Throttle slew ---\n");
    TEST(test_slew_at_target);
    TEST(test_slew_step_up);
    TEST(test_slew_step_down);
    TEST(test_slew_too_soon);

    // INIT -> READY (2)
    printf("\n--- INIT -> READY ---\n");
    TEST(test_init_to_ready);
    TEST(test_init_to_ready_regardless);

    // READY -> ENABLING (4)
    printf("\n--- READY -> ENABLING ---\n");
    TEST(test_ready_to_enabling);
    TEST(test_ready_stays_pedal_pressed);
    TEST(test_ready_stays_no_auto);
    TEST(test_ready_stays_fr_wrong);

    // ENABLING -> ACTIVE (3)
    printf("\n--- ENABLING -> ACTIVE ---\n");
    TEST(test_enabling_to_active);
    TEST(test_enabling_stays_timer);
    TEST(test_enabling_exact_boundary);

    // ENABLING -> READY abort (4)
    printf("\n--- ENABLING -> READY abort ---\n");
    TEST(test_enabling_abort_no_auto);
    TEST(test_enabling_abort_pedal);
    TEST(test_enabling_abort_fr);
    TEST(test_enabling_abort_priority);

    // ACTIVE override (4)
    printf("\n--- ACTIVE override ---\n");
    TEST(test_active_override_pedal);
    TEST(test_active_override_fr);
    TEST(test_active_override_no_auto);
    TEST(test_active_override_priority);

    // ACTIVE throttle + steering (4)
    printf("\n--- ACTIVE throttle + steering ---\n");
    TEST(test_active_throttle_slew);
    TEST(test_active_throttle_at_target);
    TEST(test_active_steering_changed);
    TEST(test_active_braking_changed);

    // OVERRIDE -> READY (4)
    printf("\n--- OVERRIDE -> READY ---\n");
    TEST(test_override_to_ready);
    TEST(test_override_stays_no_auto);
    TEST(test_override_stays_pedal_not_rearmed);
    TEST(test_override_resets_trackers);

    // FAULT -> recovery (3)
    printf("\n--- FAULT -> recovery ---\n");
    TEST(test_fault_attempts_recovery);
    TEST(test_fault_resets_trackers);
    TEST(test_fault_preserves_code);

    // Motor fault injection (2)
    printf("\n--- Motor fault injection ---\n");
    TEST(test_motor_fault_from_active);
    TEST(test_motor_fault_from_ready);

    // FR INVALID (2)
    printf("\n--- FR INVALID sensor fault ---\n");
    TEST(test_fr_invalid_from_active);
    TEST(test_fr_invalid_already_faulted);

    // CAN TX tracking (3)
    printf("\n--- CAN TX tracking ---\n");
    TEST(test_can_tx_ok_resets);
    TEST(test_can_tx_fail_below_threshold);
    TEST(test_can_tx_fail_at_threshold);

    // NULL safety (1)
    printf("\n--- NULL safety ---\n");
    TEST(test_compute_step_null);

    // Full lifecycle (1)
    printf("\n--- Full lifecycle ---\n");
    TEST(test_full_lifecycle);

    printf("\n=== %d / %d tests passed ===\n\n", tests_passed, tests_run);
    return (tests_passed == tests_run) ? 0 : 1;
}
