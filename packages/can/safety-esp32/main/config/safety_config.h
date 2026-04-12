#pragma once

/**
 * @file safety_config.h
 * @brief Compile-time constants for the Safety ESP32 node.
 */

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "can_protocol.h"

// ============================================================================
// Task Configuration
// ============================================================================

constexpr int CAN_RX_TASK_STACK = 8192;
constexpr int SAFETY_TASK_STACK = 8192;
constexpr int HEARTBEAT_TASK_STACK = 8192;
constexpr UBaseType_t CAN_RX_TASK_PRIO = 7;
constexpr UBaseType_t SAFETY_TASK_PRIO = 6;
constexpr UBaseType_t HEARTBEAT_TASK_PRIO = 4;

// ============================================================================
// Timing Constants
// ============================================================================

constexpr TickType_t CAN_RX_TIMEOUT = pdMS_TO_TICKS(10);
constexpr TickType_t SAFETY_LOOP_INTERVAL = pdMS_TO_TICKS(50);
constexpr TickType_t HEARTBEAT_SEND_INTERVAL = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS);
constexpr uint32_t INIT_DWELL_MS = 500;         // minimum dwell in INIT before READY
constexpr uint32_t ACTIVE_ENTRY_GRACE_MS = 250; // allow one-way handoff from ENABLE to ACTIVE
constexpr uint32_t ENABLE_TIMEOUT_MS = 5000;    // max time in ENABLE before retreat to NOT_READY

// ============================================================================
// Recovery Constants
// ============================================================================

constexpr uint8_t CAN_TX_FAIL_THRESHOLD = 5;
constexpr uint32_t RETRY_INTERVAL_MS = 500; // component retry cadence (infinite, no cap)

// Debounce: require N consecutive "clear" reads before declaring push-button
// or RF remote disengaged. Engage is always immediate (safety-critical).
// At 50ms loop cadence, 3 samples = 150ms disengage hold.
constexpr uint8_t ESTOP_DISENGAGE_COUNT = 3;

// Ultrasonic obstacle debounce: require a sustained close condition before
// latching, then a shorter clear window to release. This filters one-off
// ghost echoes without making recovery feel sticky.
constexpr uint32_t ULTRASONIC_ENGAGE_HOLD_MS = 1000;
constexpr uint8_t ULTRASONIC_DISENGAGE_COUNT = 6;

// Ultrasonic health hysteresis: require N consecutive agreeing health samples
// before changing health state. Prevents flap around timeout boundary.
// At 50ms loop cadence, 3 samples = 150ms hysteresis window.
constexpr uint8_t ULTRASONIC_HEALTH_HYSTERESIS = 3;

// Startup grace period for ultrasonic edge logging. Suppresses LOST/REGAINED
// transitions while the sensor's UART RX task establishes its first valid
// measurement. Matches the driver's SENSOR_HEALTH_TIMEOUT (500ms).
constexpr TickType_t ULTRASONIC_LOG_GRACE_TICKS = pdMS_TO_TICKS(500);

// Rate-limit retry failure logs: log on 1st attempt, then every RETRY_LOG_EVERY_N
// attempts. At RETRY_INTERVAL_MS = 500ms, N=20 gives ~10 second intervals.
const uint16_t RETRY_LOG_EVERY_N = 20;

// ============================================================================
// GPIO Pin Assignments
// ============================================================================

// E-stop inputs
constexpr gpio_num_t PUSH_BUTTON_HB2ES544_GPIO = GPIO_NUM_6;
constexpr int PUSH_BUTTON_HB2ES544_ACTIVE_LEVEL = 1;

constexpr gpio_num_t RF_REMOTE_EV1527_GPIO = GPIO_NUM_7;
constexpr int RF_REMOTE_EV1527_ACTIVE_LEVEL = 1;

// Power relay output (cuts power to actuators when stop/fault active)
constexpr gpio_num_t POWER_RELAY_GPIO = GPIO_NUM_2;

// CAN bus (WAVESHARE SN65HVD230 transceiver)
constexpr gpio_num_t TWAI_TX_GPIO = GPIO_NUM_4;
constexpr gpio_num_t TWAI_RX_GPIO = GPIO_NUM_5;

// Battery monitor ADC inputs
constexpr int BATTERY_VOLTAGE_GPIO = 0;        // ADC1_CH0: pack voltage via 220k/10k divider
constexpr int BATTERY_CURRENT_GPIO = 1;        // ADC1_CH1: HTFS-200-P via 6.8k/10k divider
constexpr uint32_t BATTERY_CAPACITY_MAH = 170000; // 170Ah — typical 8V deep-cycle golf cart battery

// Ultrasonic sensor (A02YYUW)
constexpr uart_port_t ULTRASONIC_A02YYUW_UART = UART_NUM_1;
constexpr int ULTRASONIC_A02YYUW_TX_GPIO = GPIO_NUM_10; // Sensor RX (mode select)
constexpr int ULTRASONIC_A02YYUW_RX_GPIO = GPIO_NUM_11; // Sensor TX (data output)
constexpr int ULTRASONIC_A02YYUW_BAUD_RATE = 9600;
constexpr uint16_t ULTRASONIC_STOP_DISTANCE_MM = 2000;  // 2m — stop if obstacle within this range
constexpr uint16_t ULTRASONIC_CLEAR_DISTANCE_MM = 2500; // 2.5m — resume only after obstacle clears to this range
