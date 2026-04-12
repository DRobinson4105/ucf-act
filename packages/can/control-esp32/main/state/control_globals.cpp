/**
 * @file control_globals.cpp
 * @brief Definitions for Control ESP32 shared state.
 */

#include "control_globals.h"
#include "optocoupler_pc817.h"

const char *TAG = "CONTROL";
const char *TAG_INIT = "CONTROL_INIT";
const char *TAG_TX __attribute__((unused)) = "CONTROL_TX";
const char *TAG_RX __attribute__((unused)) = "CONTROL_RX";

// Component Configurations
dac_mcp4728_config_t g_dac_cfg = {
	.i2c_port = I2C_NUM_0,
	.sda = DAC_SDA_GPIO,
	.scl = DAC_SCL_GPIO,
	.device_addr = 0x60,
	.clk_speed_hz = 100000,
};

relay_dpdt_my5nj_config_t g_dpdt_relay_cfg = {
	.gpio = DPDT_RELAY_GPIO,
};

adc_12bitsar_config_t g_pedal_adc_cfg = {
	.adc_unit = ADC_UNIT_1,
	.adc_channel = PEDAL_ADC_CHANNEL,
};

optocoupler_pc817_config_t g_fr_pc817_cfg = {
	.forward_gpio = FR_FORWARD_GPIO,
	.reverse_gpio = FR_REVERSE_GPIO,
};

control_driver_inputs_config_t g_driver_inputs_cfg = {
	.pedal_adc_config = &g_pedal_adc_cfg,
	.fr_pc817_config = &g_fr_pc817_cfg,
	.pedal_threshold_mv = PEDAL_ADC_THRESHOLD_MV,
	.pedal_rearm_ms = PEDAL_REARM_MS,
	.pedal_oversample_count = PEDAL_ADC_OVERSAMPLE_COUNT,
	.pedal_trigger_count = PEDAL_ADC_TRIGGER_COUNT,
};

// Thread-Safe Shared State
portMUX_TYPE g_cmd_lock = portMUX_INITIALIZER_UNLOCKED;
volatile command_snapshot_t g_cmd = {};

// State machine
volatile node_state_t g_control_state = NODE_STATE_INIT;
volatile node_fault_t g_fault_flags = NODE_FAULT_NONE;
volatile node_stop_t g_stop_flags = NODE_STOP_NONE;
volatile node_status_flags_t g_heartbeat_status_flags = 0;

// Throttle
control_throttle_state_t g_throttle = {};

// Driver override input state
control_driver_inputs_t g_driver_inputs = {};

// Timing state
uint32_t g_enable_start_ms = 0;
bool g_enable_work_done = false;
uint32_t g_boot_start_ms = 0;

// Heartbeat
volatile node_seq_t g_heartbeat_seq = 0;
portMUX_TYPE g_hb_seq_lock = portMUX_INITIALIZER_UNLOCKED;

// Recovery State
recovery_state_t g_recovery = {};
volatile bool g_can_recovery_in_progress = false;
uint8_t g_can_tx_in_flight = 0;

// Component health tracking
volatile bool g_twai_ready = false;
volatile bool g_dac_ready = false;
volatile bool g_dpdt_relay_ready = false;
volatile bool g_motor_uim2852_steering_ready = false;
volatile bool g_motor_uim2852_braking_ready = false;
volatile bool g_orin_link_ready = false;

// CAN RX task handle
#ifndef CONFIG_CONTROL_TEST_MODE
TaskHandle_t g_can_rx_task_handle = nullptr;
control_orin_link_rx_context_t g_orin_rx_ctx = {};
#endif

// CAN TX lock
portMUX_TYPE g_can_tx_lock = portMUX_INITIALIZER_UNLOCKED;

// Retry pacing
uint32_t g_last_retry_ms = 0;

// Heartbeat monitor
heartbeat_monitor_t g_hb_monitor;
int g_node_safety = -1;
int g_node_planner = -1;

// Motor state (per-motor, updated from CAN RX)
motor_uim2852_state_t g_steering_state = {};
motor_uim2852_state_t g_braking_state = {};

// Test mode context
#ifdef CONFIG_CONTROL_TEST_MODE
control_test_mode_context_t g_control_test_mode = {
#if defined(CONFIG_CONTROL_TEST_ACTUATOR_THROTTLE)
	.actuator = CONTROL_TEST_ACTUATOR_THROTTLE,
#elif defined(CONFIG_CONTROL_TEST_ACTUATOR_STEERING)
	.actuator = CONTROL_TEST_ACTUATOR_STEERING,
#else
	.actuator = CONTROL_TEST_ACTUATOR_BRAKING,
#endif
	.dac_cfg = &g_dac_cfg,
	.relay_cfg = &g_dpdt_relay_cfg,
	.driver_inputs = &g_driver_inputs,
	.driver_inputs_cfg = &g_driver_inputs_cfg,
	.pedal_threshold_mv = PEDAL_ADC_THRESHOLD_MV,
	.fr_debounce_ms = FR_PC817_DEBOUNCE_MS,
#ifdef CONFIG_BYPASS_INPUT_FR_SENSOR
	.bypass_input_fr_sensor = true,
#else
	.bypass_input_fr_sensor = false,
#endif
#ifdef CONFIG_BYPASS_INPUT_PEDAL_ADC
	.bypass_input_pedal_adc = true,
#else
	.bypass_input_pedal_adc = false,
#endif
#if defined(CONFIG_CONTROL_TEST_ACTUATOR_STEERING)
	.motor_node_id = MOTOR_NODE_STEERING,
	.motor_label = "STEERING",
	.twai_tx_gpio = TWAI_TX_GPIO,
	.twai_rx_gpio = TWAI_RX_GPIO,
	.motor_min_position = STEERING_POSITION_MIN,
	.motor_max_position = STEERING_POSITION_MAX,
#elif defined(CONFIG_CONTROL_TEST_ACTUATOR_BRAKING)
	.motor_node_id = MOTOR_NODE_BRAKING,
	.motor_label = "BRAKING",
	.twai_tx_gpio = TWAI_TX_GPIO,
	.twai_rx_gpio = TWAI_RX_GPIO,
	.motor_min_position = BRAKE_RELEASE_POSITION,
	.motor_max_position = BRAKING_POSITION_MAX,
#else
	.motor_node_id = 0,
	.motor_label = nullptr,
	.twai_tx_gpio = GPIO_NUM_NC,
	.twai_rx_gpio = GPIO_NUM_NC,
	.motor_min_position = 0,
	.motor_max_position = 0,
#endif
};
#endif
