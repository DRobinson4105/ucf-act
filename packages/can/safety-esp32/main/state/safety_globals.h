/**
 * @file safety_globals.h
 * @brief Extern declarations for all Safety ESP32 global state.
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "can_protocol.h"
#include "safety_config.h"
#include "safety_can_rx.h"
#include "safety_local_inputs.h"
#include "safety_orin_link.h"
#include "heartbeat_monitor.h"
#include "relay_srd05vdc.h"
#include "ultrasonic_a02yyuw.h"
#include "battery_monitor.h"

// ============================================================================
// Log Tags
// ============================================================================

extern const char *TAG;
extern const char *TAG_INIT;
extern const char *TAG_TX;
extern const char *TAG_RX;

// ============================================================================
// Heartbeat Monitor
// ============================================================================

// Heartbeat monitor (tracks Planner and Control timeouts)
extern heartbeat_monitor_t g_hb_monitor;
extern int g_node_planner;
extern int g_node_control;

// ============================================================================
// HB Mirror State
// ============================================================================

// Node state tracking (set by CAN RX task, read by safety_task).
// Mirror fields are updated/read as grouped snapshots under g_hb_mirror_lock.
extern volatile hb_mirror_snapshot_t g_hb_mirror;

// Spinlock for grouped planner/control mirror snapshots.
extern portMUX_TYPE g_hb_mirror_lock;

// ============================================================================
// System Target State
// ============================================================================

// System target state (Safety is the authority)
extern volatile node_state_t g_target_state;

// ============================================================================
// Heartbeat Sequence
// ============================================================================

// Safety heartbeat sequence counter
extern volatile node_seq_t g_safety_hb_seq;
extern volatile bool g_hb_tx_failing;

// Spinlock for heartbeat sequence (shared by safety_task immediate send + heartbeat_task)
extern portMUX_TYPE g_safety_hb_seq_lock;

// ============================================================================
// Cause Channels
// ============================================================================

// Safety heartbeat cause channels.
extern volatile node_stop_t g_stop_flags;
extern volatile node_fault_t g_fault_flags;
extern volatile uint8_t g_battery_soc;

// ============================================================================
// Component Health Tracking
// ============================================================================

extern volatile bool g_push_button_init_ok;
extern volatile bool g_rf_remote_init_ok;
extern volatile bool g_ultrasonic_init_ok;
#ifndef CONFIG_BYPASS_INPUT_ULTRASONIC
extern TickType_t g_ultrasonic_init_tick;
#endif
extern volatile bool g_relay_init_ok;
extern volatile bool g_battery_monitor_init_ok;
extern volatile bool g_twai_ready;
extern volatile bool g_orin_link_ready;

// ============================================================================
// Task Handle
// ============================================================================

// CAN RX task handle — used to verify quiesce during recovery.
extern TaskHandle_t g_can_rx_task_handle;
extern safety_orin_link_rx_context_t g_orin_rx_ctx;

// ============================================================================
// Local Stop-Input State
// ============================================================================

// Local stop-input state (push button, RF remote, ultrasonic).
extern safety_local_inputs_config_t g_local_inputs_cfg;
extern safety_local_inputs_state_t g_local_inputs;

// ============================================================================
// Component Configurations
// ============================================================================

extern estop_input_config_t g_push_button_cfg;
extern estop_input_config_t g_rf_remote_cfg;
extern relay_srd05vdc_config_t g_relay_cfg;
extern ultrasonic_a02yyuw_config_t g_ultrasonic_cfg;
extern battery_monitor_config_t g_battery_cfg;

// ============================================================================
// Retry/Recovery State
// ============================================================================

// Retry pacing: last timestamp (ms) when retry_failed_components ran.
// Retries are infinite (no cap) but paced at RETRY_INTERVAL_MS to avoid
// hammering init calls on every safety loop tick.
extern uint32_t g_last_retry_ms;
extern uint32_t g_boot_start_ms;

extern uint8_t g_can_tx_fail_count;
extern volatile bool g_can_recovery_in_progress;
extern uint8_t g_can_tx_in_flight;

// Spinlock for CAN TX tracking / recovery (used by safety_task + heartbeat_task)
extern portMUX_TYPE g_can_tx_lock;
