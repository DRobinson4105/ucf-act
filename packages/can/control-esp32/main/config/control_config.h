/**
 * @file control_config.h
 * @brief Control ESP32 compile-time constants: task sizing, timing, GPIO pins, motor limits, retry tuning.
 */
#pragma once

#include "adc_12bitsar.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "can_protocol.h"

// Task Configuration
constexpr int CAN_RX_TASK_STACK = 8192;
constexpr int CONTROL_TASK_STACK = 8192;
constexpr int HEARTBEAT_TASK_STACK = 8192;
constexpr UBaseType_t CAN_RX_TASK_PRIO = 7;
constexpr UBaseType_t CONTROL_TASK_PRIO = 6;
constexpr UBaseType_t HEARTBEAT_TASK_PRIO = 4;

// Timing Constants
constexpr TickType_t CAN_RX_TIMEOUT = pdMS_TO_TICKS(10);
constexpr TickType_t CONTROL_LOOP_INTERVAL = pdMS_TO_TICKS(20);
constexpr TickType_t HEARTBEAT_SEND_INTERVAL = pdMS_TO_TICKS(HEARTBEAT_INTERVAL_MS);
constexpr int16_t THROTTLE_LEVEL_MIN = 0;
constexpr int16_t THROTTLE_LEVEL_MAX = 4095;
constexpr uint8_t PT_STREAM_STARTUP_FRAMES = 2;
constexpr uint32_t INIT_DWELL_MS = 500;
constexpr uint32_t ENABLE_SEQUENCE_MS = 200;
constexpr uint32_t THROTTLE_SLEW_INTERVAL_MS = 100;
constexpr int16_t THROTTLE_SLEW_STEP = 200;
constexpr TickType_t PLANNER_CMD_TIMEOUT = pdMS_TO_TICKS(500);
constexpr uint8_t PLANNER_CMD_STALE_COUNT = 10;
constexpr uint16_t PEDAL_ADC_THRESHOLD_MV = 500;
constexpr uint32_t PEDAL_REARM_MS = 500;
constexpr uint8_t PEDAL_ADC_OVERSAMPLE_COUNT = 8;
constexpr uint8_t PEDAL_ADC_TRIGGER_COUNT = 3;
constexpr int32_t MOTOR_UIM2852_POSITION_ERROR_THRESHOLD = 200;
constexpr TickType_t MOTOR_UIM2852_LIVENESS_TIMEOUT = pdMS_TO_TICKS(500);

// Motor Command Envelope Limits
constexpr int32_t STEERING_MAX_SPEED    = CONFIG_STEERING_LM0_MAX_SPEED;
constexpr int32_t STEERING_POSITION_MIN = CONFIG_STEERING_LM1_LOWER_LIMIT;
constexpr int32_t STEERING_POSITION_MAX = CONFIG_STEERING_LM2_UPPER_LIMIT;
constexpr int32_t STEERING_MAX_ACCEL    = CONFIG_STEERING_LM7_MAX_ACCEL;
constexpr int32_t BRAKING_MAX_SPEED     = CONFIG_BRAKING_LM0_MAX_SPEED;
constexpr int32_t BRAKING_POSITION_MIN  = CONFIG_BRAKING_LM1_LOWER_LIMIT;
constexpr int32_t BRAKING_POSITION_MAX  = CONFIG_BRAKING_LM2_UPPER_LIMIT;
constexpr int32_t BRAKING_MAX_ACCEL     = CONFIG_BRAKING_LM7_MAX_ACCEL;
constexpr int32_t BRAKE_RELEASE_POSITION = CONFIG_BRAKE_RELEASE_POSITION;
constexpr int32_t BRAKING_PLANNER_ZERO_POSITION = BRAKE_RELEASE_POSITION;
constexpr int32_t BRAKING_PT_STEP_LIMIT_PER_500MS = CONFIG_BRAKING_PT_STEP_LIMIT_PULSES_PER_500MS;

// Retry & Fault Constants
constexpr uint8_t CAN_TX_CONSEC_FAIL_THRESHOLD = 5;
constexpr uint32_t RETRY_INTERVAL_MS = 500;
const uint16_t RETRY_LOG_EVERY_N = 20;

// Motor CAN Node IDs (UIM2852CA)
constexpr uint8_t MOTOR_NODE_STEERING = 7;
constexpr uint8_t MOTOR_NODE_BRAKING  = 6;

// GPIO Pin Assignments
constexpr gpio_num_t TWAI_TX_GPIO = GPIO_NUM_4;
constexpr gpio_num_t TWAI_RX_GPIO = GPIO_NUM_5;
constexpr gpio_num_t DAC_SDA_GPIO = GPIO_NUM_6;
constexpr gpio_num_t DAC_SCL_GPIO = GPIO_NUM_7;
constexpr gpio_num_t DPDT_RELAY_GPIO = GPIO_NUM_10;
constexpr adc_channel_t PEDAL_ADC_CHANNEL = ADC_CHANNEL_0;
constexpr gpio_num_t FR_FORWARD_GPIO = GPIO_NUM_22;
constexpr gpio_num_t FR_REVERSE_GPIO = GPIO_NUM_23;
