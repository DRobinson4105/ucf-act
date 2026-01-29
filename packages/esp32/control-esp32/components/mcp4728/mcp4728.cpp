#include "mcp4728.hh"

#include <string.h>

// MCP4728 fast write command (updates all channels)
static const uint8_t kFastWriteCmd = 0x00;

// clamp to 12-bit DAC range
static void clamp_12bit(uint16_t *value) {
    if (*value > 0x0FFFU) *value = 0x0FFFU;
}

// write all channel values using the fast write command
static esp_err_t write_all_channels(mcp4728_handle_t *handle) {
    uint8_t payload[9] = {0};
    payload[0] = kFastWriteCmd;

    for (int i = 0; i < 4; ++i) {
        uint16_t v = handle->channel_values[i];
        clamp_12bit(&v);
        // upper nibble contains D11..D8; power-down bits kept at 00
        payload[1 + i * 2] = (uint8_t)((v >> 8) & 0x0F);
        payload[2 + i * 2] = (uint8_t)(v & 0xFF);
    }

    return i2c_master_transmit(handle->dev, payload, sizeof(payload), -1);
}

esp_err_t mcp4728_init(i2c_master_bus_handle_t bus,
                       uint32_t scl_speed_hz,
                       uint8_t i2c_addr,
                       mcp4728_handle_t *out_handle) {
    if (!out_handle) return ESP_ERR_INVALID_ARG;

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = i2c_addr,
        .scl_speed_hz = scl_speed_hz,
        .scl_wait_us = 0,
        .flags = {
            .disable_ack_check = 0,
        },
    };

    i2c_master_dev_handle_t dev = NULL;
    esp_err_t err = i2c_master_bus_add_device(bus, &dev_cfg, &dev);
    if (err != ESP_OK) return err;

    memset(out_handle, 0, sizeof(*out_handle));
    out_handle->bus = bus;
    out_handle->dev = dev;
    return write_all_channels(out_handle);
}

esp_err_t mcp4728_set_channel(mcp4728_handle_t *handle, uint8_t channel, uint16_t value) {
    if (!handle || channel > 3) return ESP_ERR_INVALID_ARG;
	
    handle->channel_values[channel] = value;
    return write_all_channels(handle);
}
