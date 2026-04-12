/**
 * @file control_health.h
 * @brief Component health evaluation, fault gating, and recovery orchestration for Control ESP32.
 */
#pragma once

#include <stdint.h>

#include "can_protocol.h"
#include "control_driver_inputs.h"
#include "esp_err.h"

bool all_required_components_ready(void);
node_fault_t primary_fault_from_component_health(void);
const char *fr_state_to_string(fr_state_t state);
const char *sensor_fault_detail_string(fr_state_t fr_state, bool has_fr_sample);
const char *fault_detail_string(node_fault_t fault_flags, fr_state_t fr_state, bool has_fr_sample);
void mark_component_lost(volatile bool *ready, const char *name, const char *detail);
void handle_runtime_error(esp_err_t err, volatile bool *ready, const char *name, const char *detail);
void motor_uim2852_shutdown(uint8_t node_id, volatile bool *ready, const char *label);
void update_control_state_from_component_health(void);
void quiesce_can_rx(void);
void retry_failed_components(void);
