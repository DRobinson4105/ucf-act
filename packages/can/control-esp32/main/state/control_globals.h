/**
 * @file control_globals.h
 * @brief Extern declarations for Control ESP32 shared state.
 */
#pragma once

#include "can_protocol.h"
#include "control_can_rx.h"
#include "control_config.h"
#include "control_driver_inputs.h"
#include "control_orin_link.h"
#include "control_test_mode.h"
#include "dac_mcp4728.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "heartbeat_monitor.h"
#include "relay_dpdt_my5nj.h"
#include "adc_12bitsar.h"
#include "optocoupler_pc817.h"
#include "motor_dispatch.h"
#include "motor_types.h"

extern const char *TAG;
extern const char *TAG_INIT;
extern const char *TAG_TX;
extern const char *TAG_RX;

// Component Configurations
extern dac_mcp4728_config_t g_dac_cfg;
extern relay_dpdt_my5nj_config_t g_dpdt_relay_cfg;
extern adc_12bitsar_config_t g_pedal_adc_cfg;
extern optocoupler_pc817_config_t g_fr_pc817_cfg;
extern control_driver_inputs_config_t g_driver_inputs_cfg;

// Thread-Safe Shared State
extern portMUX_TYPE g_cmd_lock;
extern volatile command_snapshot_t g_cmd;

// State machine
extern volatile node_state_t g_control_state;
extern volatile node_fault_t g_fault_flags;
extern volatile node_stop_t g_stop_flags;
extern volatile node_status_flags_t g_heartbeat_status_flags;

// Throttle
struct control_throttle_state_t
{
	int16_t current_level;
	uint32_t last_change_ms;
};
extern control_throttle_state_t g_throttle;

// Driver override input state
extern control_driver_inputs_t g_driver_inputs;

// Timing state
extern uint32_t g_enable_start_ms;
extern bool g_enable_work_done;
extern uint32_t g_boot_start_ms;

// Heartbeat
extern volatile node_seq_t g_heartbeat_seq;
extern portMUX_TYPE g_hb_seq_lock;

// Recovery State
struct recovery_state_t
{
	uint8_t can_tx_fail_count;
};
extern recovery_state_t g_recovery;
extern volatile bool g_can_recovery_in_progress;
extern uint8_t g_can_tx_in_flight;

// Component health tracking
extern volatile bool g_twai_ready;
extern volatile bool g_dac_ready;
extern volatile bool g_dpdt_relay_ready;
extern volatile bool g_motor_uim2852_steering_ready;
extern volatile bool g_motor_uim2852_braking_ready;
extern volatile bool g_orin_link_ready;

// CAN RX task handle
#ifndef CONFIG_CONTROL_TEST_MODE
extern TaskHandle_t g_can_rx_task_handle;
extern control_orin_link_rx_context_t g_orin_rx_ctx;
#endif

// CAN TX lock
extern portMUX_TYPE g_can_tx_lock;

// Retry pacing
extern uint32_t g_last_retry_ms;

// Heartbeat monitor
extern heartbeat_monitor_t g_hb_monitor;
extern int g_node_safety;
extern int g_node_planner;

// Motor state (per-motor, updated from CAN RX)
struct motor_uim2852_state_t
{
	int32_t absolute_position;
	int32_t current_speed;
	int32_t target_position;
	bool driver_enabled;
	bool motion_in_progress;
	bool in_position;
	bool stopped;
	bool stall_detected;
	bool error_detected;
	bool pt_mode_active;
	bool pt_motion_started;
	bool pt_fifo_empty;
	bool pt_fifo_low;
	uint16_t pt_write_index;
	uint8_t pt_prefill_count;
	int32_t pt_last_position;
	uint16_t pt_frame_time_ms;
	TickType_t last_response_tick;
	portMUX_TYPE lock;
};

extern motor_uim2852_state_t g_steering_state;
extern motor_uim2852_state_t g_braking_state;

// Test mode context
#ifdef CONFIG_CONTROL_TEST_MODE
extern control_test_mode_context_t g_control_test_mode;
#endif
