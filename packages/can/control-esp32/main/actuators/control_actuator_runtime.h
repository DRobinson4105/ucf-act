/**
 * @file control_actuator_runtime.h
 * @brief Control actuator runtime helpers: enable, disable, override, and PT stream scheduling.
 */
#pragma once

#include <stdint.h>

#include "control_globals.h"
#include "control_logic.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

const char *override_reason_to_string(override_reason_t reason);
const char *disable_reason_to_string(disable_reason_t reason);
const char *abort_reason_to_string(abort_reason_t reason);

esp_err_t feed_pt_stream_if_due(motor_uim2852_state_t *state, uint8_t node_id, int32_t position, TickType_t now_tick,
                                TickType_t *next_feed_tick, bool *fed_frame);
int32_t compute_braking_pt_step_limit(const motor_uim2852_state_t *state);

void disable_autonomous_actuators(void);
void execute_trigger_override(override_reason_t reason);
void execute_disable_autonomy(disable_reason_t reason, node_fault_t safety_fault_flags, node_stop_t safety_stop_flags,
                              node_fault_t control_fault_flags);
void execute_start_enable(void);
void execute_complete_enable(void);
void execute_abort_enable(abort_reason_t reason);
