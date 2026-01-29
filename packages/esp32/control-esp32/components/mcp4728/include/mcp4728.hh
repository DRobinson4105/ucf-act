#pragma once

#include <stdint.h>

#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint16_t channel_values[4];
} mcp4728_handle_t;

// initialize MCP4728 on an existing I2C bus
esp_err_t mcp4728_init(i2c_master_bus_handle_t bus,
                       uint32_t scl_speed_hz,
                       uint8_t i2c_addr,
                       mcp4728_handle_t *out_handle);

// set a single DAC channel output (0-4095), writes all channels
esp_err_t mcp4728_set_channel(mcp4728_handle_t *handle, uint8_t channel, uint16_t value);

#ifdef __cplusplus
}
#endif
