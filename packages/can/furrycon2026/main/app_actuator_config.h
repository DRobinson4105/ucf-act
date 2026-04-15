#pragma once

#include "driver/gpio.h"

#define APP_THROTTLE_LEVEL_MIN         0U
#define APP_THROTTLE_LEVEL_MAX         4095U
#define APP_THROTTLE_DEFAULT_LEVEL     0U
#define APP_DAC_I2C_PORT               I2C_NUM_0
#define APP_DAC_SDA_GPIO               GPIO_NUM_6
#define APP_DAC_SCL_GPIO               GPIO_NUM_7
#define APP_DAC_I2C_ADDR               0x60U
#define APP_DAC_I2C_CLK_SPEED_HZ       100000U
#define APP_DPDT_RELAY_GPIO            GPIO_NUM_10
#define APP_DPDT_RELAY_SETTLE_DELAY_MS 50U
#define APP_STEERING_PWM_GPIO          GPIO_NUM_21
#define APP_STEERING_PWM_FREQ_HZ       50U
#define APP_STEERING_MIN_VALUE         0U
#define APP_STEERING_DEFAULT_VALUE     360U
#define APP_STEERING_MAX_VALUE         720U
#define APP_STEERING_PWM_MIN_US        500U
#define APP_STEERING_PWM_NEUTRAL_US    1500U
#define APP_STEERING_PWM_MAX_US        2500U
