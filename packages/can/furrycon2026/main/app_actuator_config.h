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
