/**
 * @file safety_globals.cpp
 * @brief Definitions for all Safety ESP32 global state.
 */

#include "safety_globals.h"

// ============================================================================
// Log Tags
// ============================================================================

const char *TAG = "SAFETY";
const char *TAG_INIT = "SAFETY_INIT";
const char *TAG_TX __attribute__((unused)) = "SAFETY_TX";
const char *TAG_RX __attribute__((unused)) = "SAFETY_RX";

// ============================================================================
// Heartbeat Monitor
// ============================================================================

heartbeat_monitor_t g_hb_monitor;
int g_node_planner = -1;
int g_node_control = -1;

// ============================================================================
// HB Mirror State
// ============================================================================

volatile hb_mirror_snapshot_t g_hb_mirror = {};

portMUX_TYPE g_hb_mirror_lock = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// System Target State
// ============================================================================

volatile node_state_t g_target_state = NODE_STATE_INIT;

// ============================================================================
// Heartbeat Sequence
// ============================================================================

volatile node_seq_t g_safety_hb_seq = 0;
[[maybe_unused]] volatile bool g_hb_tx_failing = false;

portMUX_TYPE g_safety_hb_seq_lock = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
// Cause Channels
// ============================================================================

volatile node_stop_t g_stop_flags = NODE_STOP_NONE;
volatile node_fault_t g_fault_flags = NODE_FAULT_NONE;
volatile uint8_t g_battery_soc = 0;

// ============================================================================
// Component Health Tracking
// ============================================================================

volatile bool g_push_button_init_ok = false;
volatile bool g_rf_remote_init_ok = false;
volatile bool g_ultrasonic_init_ok = false;
#ifndef CONFIG_BYPASS_INPUT_ULTRASONIC
TickType_t g_ultrasonic_init_tick = 0;
#endif
volatile bool g_relay_init_ok = false;
volatile bool g_battery_monitor_init_ok = false;
volatile bool g_twai_ready = false;
volatile bool g_orin_link_ready = false;

// ============================================================================
// Task Handle
// ============================================================================

TaskHandle_t g_can_rx_task_handle = nullptr;
safety_orin_link_rx_context_t g_orin_rx_ctx = {};

// ============================================================================
// Local Stop-Input State
// ============================================================================

safety_local_inputs_config_t g_local_inputs_cfg = {
    .estop_disengage_count = ESTOP_DISENGAGE_COUNT,
    .ultrasonic_health_hysteresis = ULTRASONIC_HEALTH_HYSTERESIS,
    .ultrasonic_engage_hold_ms = ULTRASONIC_ENGAGE_HOLD_MS,
    .ultrasonic_disengage_count = ULTRASONIC_DISENGAGE_COUNT,
    .ultrasonic_stop_distance_mm = ULTRASONIC_STOP_DISTANCE_MM,
    .ultrasonic_clear_distance_mm = ULTRASONIC_CLEAR_DISTANCE_MM,
};
safety_local_inputs_state_t g_local_inputs = {};

// ============================================================================
// Component Configurations
// ============================================================================

estop_input_config_t g_push_button_cfg;
estop_input_config_t g_rf_remote_cfg;
relay_srd05vdc_config_t g_relay_cfg;
ultrasonic_a02yyuw_config_t g_ultrasonic_cfg;
battery_monitor_config_t g_battery_cfg;

// ============================================================================
// Retry/Recovery State
// ============================================================================

uint32_t g_last_retry_ms = 0;
uint32_t g_boot_start_ms = 0;

uint8_t g_can_tx_fail_count = 0;
volatile bool g_can_recovery_in_progress = false;
uint8_t g_can_tx_in_flight = 0;

portMUX_TYPE g_can_tx_lock = portMUX_INITIALIZER_UNLOCKED;
