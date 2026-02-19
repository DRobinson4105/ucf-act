/**
 * @file test_control_logic.c
 * @brief Unit tests for Control pure state machine logic.
 */

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

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

static control_inputs_t default_inputs(void) {
    control_inputs_t in = {
        .target_state = NODE_STATE_READY,
        .throttle_target = 0,
        .steering_cmd = 0,
        .braking_cmd = 0,
        .motor_fault_code = NODE_FAULT_NONE,
        .fr_state = FR_STATE_FORWARD,
        .pedal_pressed = false,
        .pedal_rearmed = true,
        .fr_is_invalid = false,
        .steering_position_error = false,
        .braking_position_error = false,
        .now_ms = 1000,
        .boot_start_ms = 0,
        .init_dwell_ms = 500,
        .enable_start_ms = 0,
        .enable_sequence_ms = 200,
        .enable_work_done = false,
        .throttle_current = 0,
        .last_throttle_change_ms = 0,
        .throttle_slew_interval_ms = 100,
        .last_steering_sent = INT16_MIN,
        .last_braking_sent = INT16_MIN,
        .steering_min = -3000,
        .steering_max = 3000,
        .braking_min = -3000,
        .braking_max = 3000,
    };
    return in;
}

static void test_init_to_ready_when_preconditions_pass(void) {
    control_inputs_t in = default_inputs();
    control_step_result_t r = control_compute_step(NODE_STATE_INIT, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_READY);
}

static void test_init_to_not_ready_when_preconditions_fail(void) {
    control_inputs_t in = default_inputs();
    in.pedal_pressed = true;
    control_step_result_t r = control_compute_step(NODE_STATE_INIT, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_NOT_READY);
    assert((r.precondition_fail & PRECONDITION_FAIL_PEDAL_PRESSED) != 0);
}

static void test_not_ready_to_ready_when_clear(void) {
    control_inputs_t in = default_inputs();
    control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_READY);
    assert(r.precondition_fail == PRECONDITION_OK);
}

static void test_not_ready_stays_when_pedal_pressed(void) {
    control_inputs_t in = default_inputs();
    in.pedal_pressed = true;
    control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_NOT_READY);
    assert((r.precondition_fail & PRECONDITION_FAIL_PEDAL_PRESSED) != 0);
}

static void test_ready_to_enable_on_safety_target(void) {
    control_inputs_t in = default_inputs();
    in.target_state = NODE_STATE_ENABLE;
    control_step_result_t r = control_compute_step(NODE_STATE_READY, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_ENABLE);
    assert((r.actions & CONTROL_ACTION_START_ENABLE) != 0);
}

static void test_ready_to_not_ready_when_precondition_drops(void) {
    control_inputs_t in = default_inputs();
    in.pedal_pressed = true;
    control_step_result_t r = control_compute_step(NODE_STATE_READY, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_NOT_READY);
    assert((r.precondition_fail & PRECONDITION_FAIL_PEDAL_PRESSED) != 0);
}

static void test_enable_to_active_when_complete_and_target_active(void) {
    control_inputs_t in = default_inputs();
    in.target_state = NODE_STATE_ACTIVE;
    in.enable_start_ms = 100;
    in.now_ms = 400;
    control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_ACTIVE);
    assert((r.actions & CONTROL_ACTION_COMPLETE_ENABLE) != 0);
}

static void test_enable_stays_enable_with_complete_flag(void) {
    control_inputs_t in = default_inputs();
    in.target_state = NODE_STATE_ENABLE;
    in.enable_start_ms = 100;
    in.now_ms = 400;
    control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_ENABLE);
    assert((r.heartbeat_flags & HEARTBEAT_FLAG_ENABLE_COMPLETE) != 0);
}

static void test_enable_abort_to_not_ready_on_pedal(void) {
    control_inputs_t in = default_inputs();
    in.target_state = NODE_STATE_ENABLE;
    in.pedal_pressed = true;
    in.pedal_rearmed = false;
    control_step_result_t r = control_compute_step(NODE_STATE_ENABLE, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_NOT_READY);
    assert((r.actions & CONTROL_ACTION_ABORT_ENABLE) != 0);
    assert(r.abort_reason == CONTROL_ABORT_REASON_PEDAL_PRESSED);
}

static void test_active_safety_retreat_to_not_ready(void) {
    control_inputs_t in = default_inputs();
    in.target_state = NODE_STATE_NOT_READY;
    in.pedal_pressed = true;
    in.pedal_rearmed = false;
    control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_NOT_READY);
    assert((r.actions & CONTROL_ACTION_DISABLE_AUTONOMY) != 0);
}

static void test_active_override_on_pedal(void) {
    control_inputs_t in = default_inputs();
    in.target_state = NODE_STATE_ACTIVE;
    in.pedal_pressed = true;
    control_step_result_t r = control_compute_step(NODE_STATE_ACTIVE, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_OVERRIDE);
    assert((r.actions & CONTROL_ACTION_TRIGGER_OVERRIDE) != 0);
}

static void test_override_recovery_to_ready(void) {
    control_inputs_t in = default_inputs();
    in.target_state = NODE_STATE_READY;
    in.pedal_pressed = false;
    in.pedal_rearmed = true;
    in.fr_state = FR_STATE_FORWARD;
    control_step_result_t r = control_compute_step(NODE_STATE_OVERRIDE, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_READY);
}

static void test_fault_clear_to_not_ready(void) {
    control_inputs_t in = default_inputs();
    in.pedal_pressed = true;
    in.pedal_rearmed = false;
    in.motor_fault_code = NODE_FAULT_NONE;
    control_step_result_t r = control_compute_step(NODE_STATE_FAULT, NODE_FAULT_MOTOR_COMM, &in);
    assert(r.new_state == NODE_STATE_NOT_READY);
    assert(r.new_fault_code == NODE_FAULT_NONE);
}

static void test_fault_clear_to_ready(void) {
    control_inputs_t in = default_inputs();
    in.motor_fault_code = NODE_FAULT_NONE;
    control_step_result_t r = control_compute_step(NODE_STATE_FAULT, NODE_FAULT_MOTOR_COMM, &in);
    assert(r.new_state == NODE_STATE_READY);
    assert(r.new_fault_code == NODE_FAULT_NONE);
}

static void test_fault_injection_from_not_ready(void) {
    control_inputs_t in = default_inputs();
    in.motor_fault_code = NODE_FAULT_MOTOR_COMM;
    control_step_result_t r = control_compute_step(NODE_STATE_NOT_READY, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_FAULT);
    assert(r.new_fault_code == NODE_FAULT_MOTOR_COMM);
}

static void test_unknown_state_defaults_to_fault(void) {
    control_inputs_t in = default_inputs();
    control_step_result_t r = control_compute_step(0xFF, NODE_FAULT_NONE, &in);
    assert(r.new_state == NODE_STATE_FAULT);
    assert(r.new_fault_code == NODE_FAULT_GENERAL);
}

int main(void) {
    printf("=== control_logic tests ===\n\n");

    TEST(test_init_to_ready_when_preconditions_pass);
    TEST(test_init_to_not_ready_when_preconditions_fail);
    TEST(test_not_ready_to_ready_when_clear);
    TEST(test_not_ready_stays_when_pedal_pressed);
    TEST(test_ready_to_enable_on_safety_target);
    TEST(test_ready_to_not_ready_when_precondition_drops);
    TEST(test_enable_to_active_when_complete_and_target_active);
    TEST(test_enable_stays_enable_with_complete_flag);
    TEST(test_enable_abort_to_not_ready_on_pedal);
    TEST(test_active_safety_retreat_to_not_ready);
    TEST(test_active_override_on_pedal);
    TEST(test_override_recovery_to_ready);
    TEST(test_fault_clear_to_not_ready);
    TEST(test_fault_clear_to_ready);
    TEST(test_fault_injection_from_not_ready);
    TEST(test_unknown_state_defaults_to_fault);

    printf("\n=== Results ===\n");
    printf("%d/%d tests passed\n", tests_passed, tests_run);

    return (tests_run == tests_passed) ? 0 : 1;
}
