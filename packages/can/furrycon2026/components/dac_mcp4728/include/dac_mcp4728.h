/**
 * @file dac_mcp4728.h
 * @brief MCP4728 4-channel 12-bit I2C DAC driver for throttle control.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
    i2c_port_num_t i2c_port;
    gpio_num_t sda;
    gpio_num_t scl;
    uint8_t device_addr;
    uint32_t clk_speed_hz;
} dac_mcp4728_config_t;

#define DAC_MCP4728_CONFIG_DEFAULT() \
    { \
        .i2c_port = I2C_NUM_0, \
        .sda = GPIO_NUM_6, \
        .scl = GPIO_NUM_7, \
        .device_addr = 0x60, \
        .clk_speed_hz = 100000, \
    }

#define DAC_MCP4728_LEVEL_MIN 0
#define DAC_MCP4728_LEVEL_MAX 4095

esp_err_t dac_mcp4728_init(const dac_mcp4728_config_t *config);
esp_err_t dac_mcp4728_set_level(uint16_t level);
esp_err_t dac_mcp4728_disable(void);
esp_err_t dac_mcp4728_enable_autonomous(void);
esp_err_t dac_mcp4728_emergency_stop(void);
uint16_t dac_mcp4728_get_level(void);
bool dac_mcp4728_is_autonomous(void);

#ifdef __cplusplus
}
#endif
