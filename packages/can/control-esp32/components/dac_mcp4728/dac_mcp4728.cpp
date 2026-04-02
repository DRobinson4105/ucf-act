/**
 * @file dac_mcp4728.cpp
 * @brief MCP4728 4-channel 12-bit I2C DAC driver implementation.
 */

#include "dac_mcp4728.h"

#include "freertos/FreeRTOS.h"
#include "esp_log.h"

namespace
{

const char *TAG = "DAC_MCP4728";

// ============================================================================
// MCP4728 I2C Protocol Constants
// ============================================================================

// Multi-write command: write to a single channel's DAC input register.
// Format: [0100 0 DAC1 DAC0 UDAC] [VREF PD1 PD0 Gx D11..D8] [D7..D0]
//   DAC1:DAC0 = channel (00=A, 01=B, 10=C, 11=D)
//   UDAC = 0 (update immediately)
//   VREF = 1 (internal reference)
//   PD1:PD0 = 00 (normal mode, not powered down)
//   Gx = 1 (2x gain)
constexpr uint8_t CMD_MULTI_WRITE_CH_A = 0x40; // 0100 0 00 0 = channel A, UDAC=0

// Channel A config byte: VREF=0 (VDD reference), PD=00 (normal), Gx=0 (1x gain)
// Output = VDD × (level / 4096).  External op-amp scales to 0-8.5V.
constexpr uint8_t CH_A_CONFIG = 0x00; // 0 00 0 0000 (upper 4 bits, lower 4 are D11:D8)

// ============================================================================
// Module State
// ============================================================================

dac_mcp4728_config_t s_config = {};
bool s_initialized = false;
uint16_t s_current_level = 0;
bool s_autonomous = false;
i2c_master_bus_handle_t s_bus_handle = nullptr;
i2c_master_dev_handle_t s_dev_handle = nullptr;

/**
 * @brief Release I2C resources and reset module state.
 */
void release_resources(void)
{
	if (s_dev_handle)
	{
		i2c_master_bus_rm_device(s_dev_handle);
		s_dev_handle = nullptr;
	}
	if (s_bus_handle)
	{
		i2c_del_master_bus(s_bus_handle);
		s_bus_handle = nullptr;
	}
	s_initialized = false;
	s_autonomous = false;
	s_current_level = 0;
}

/**
 * @brief Write a 12-bit value to channel A.
 *
 * Uses the multi-write command to set channel A with VDD reference,
 * 1x gain, normal power mode, and immediate update.
 *
 * @param level  12-bit DAC value (0-4095)
 * @return ESP_OK on success
 */
esp_err_t write_channel_a(uint16_t level)
{
	if (!s_dev_handle)
		return ESP_ERR_INVALID_STATE;

	if (level > DAC_MCP4728_LEVEL_MAX)
		level = DAC_MCP4728_LEVEL_MAX;

	// 3-byte multi-write command for channel A
	uint8_t tx[3] = {
		CMD_MULTI_WRITE_CH_A,
		(uint8_t)(CH_A_CONFIG | ((level >> 8) & 0x0F)), // config + upper 4 data bits
		(uint8_t)(level & 0xFF),                         // lower 8 data bits
	};

	return i2c_master_transmit(s_dev_handle, tx, sizeof(tx), pdMS_TO_TICKS(100));
}

/**
 * @brief Read channel A's DAC input register value.
 *
 * The MCP4728 responds to a read with 24 bytes (3 bytes per channel,
 * 8 channels: input + EEPROM for each).  We only need bytes 1-2
 * (channel A DAC input register).
 *
 * @param out_level  [out] 12-bit DAC value read from channel A
 * @return ESP_OK on success
 */
esp_err_t read_channel_a(uint16_t *out_level)
{
	if (!s_dev_handle || !out_level)
		return ESP_ERR_INVALID_STATE;

	// MCP4728 read: returns 3 bytes per channel × 4 channels × 2 (input + EEPROM) = 24 bytes.
	// Channel A DAC input register is in bytes 0-2:
	//   Byte 0: [RDY/BSY] [POR] [DAC1] [DAC0] [x] [x] [x] [x]
	//   Byte 1: [VREF] [PD1] [PD0] [Gx] [D11] [D10] [D9] [D8]
	//   Byte 2: [D7] [D6] [D5] [D4] [D3] [D2] [D1] [D0]
	uint8_t rx[3] = {};
	esp_err_t err = i2c_master_receive(s_dev_handle, rx, sizeof(rx), pdMS_TO_TICKS(100));
	if (err != ESP_OK)
		return err;

	*out_level = (uint16_t)(((rx[1] & 0x0F) << 8) | rx[2]);
	return ESP_OK;
}

} // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t dac_mcp4728_init(const dac_mcp4728_config_t *config)
{
	if (!config)
		return ESP_ERR_INVALID_ARG;

	if (s_initialized || s_bus_handle || s_dev_handle)
		release_resources();

	s_config = *config;

	// Initialize I2C master bus
	i2c_master_bus_config_t bus_cfg = {};
	bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
	bus_cfg.i2c_port = config->i2c_port;
	bus_cfg.scl_io_num = config->scl;
	bus_cfg.sda_io_num = config->sda;
	bus_cfg.glitch_ignore_cnt = 7;
	bus_cfg.flags.enable_internal_pullup = true;

	esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_bus_handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "I2C bus init failed: %s", esp_err_to_name(err));
		return err;
	}

	// Add MCP4728 as I2C device
	i2c_device_config_t dev_cfg = {};
	dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
	dev_cfg.device_address = config->device_addr;
	dev_cfg.scl_speed_hz = config->clk_speed_hz;

	err = i2c_master_bus_add_device(s_bus_handle, &dev_cfg, &s_dev_handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "I2C add device failed: %s", esp_err_to_name(err));
		release_resources();
		return err;
	}

	// Safe state: channel A output at 0
	err = write_channel_a(0);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Initial write failed: %s", esp_err_to_name(err));
		release_resources();
		return err;
	}

	s_current_level = 0;
	s_autonomous = false;
	s_initialized = true;

	ESP_LOGI(TAG, "Initialized: sda=%d scl=%d addr=0x%02X", config->sda, config->scl, config->device_addr);
	return ESP_OK;
}

// ============================================================================
// Output Control
// ============================================================================

esp_err_t dac_mcp4728_set_level(uint16_t level)
{
	if (!s_initialized)
	{
		ESP_LOGE(TAG, "Not initialized");
		return ESP_ERR_INVALID_STATE;
	}

	if (level > DAC_MCP4728_LEVEL_MAX)
		level = DAC_MCP4728_LEVEL_MAX;

	esp_err_t err = write_channel_a(level);
	if (err != ESP_OK)
		return err;

	// Read back and verify
	uint16_t readback = 0xFFFF;
	err = read_channel_a(&readback);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Readback failed: %s", esp_err_to_name(err));
		return err;
	}

	// Log periodically or on mismatch
	static uint16_t s_verify_count = 0;
	if (readback != level || (++s_verify_count % 50) == 0)
	{
		ESP_LOGI(TAG, "Level: wrote=%u read=%u %s", level, readback,
		         (readback == level) ? "OK" : "MISMATCH");
	}

	if (readback != level)
		return ESP_ERR_INVALID_RESPONSE;

	s_current_level = level;
	return ESP_OK;
}

esp_err_t dac_mcp4728_disable(void)
{
	if (!s_initialized)
		return ESP_ERR_INVALID_STATE;

	esp_err_t err = write_channel_a(0);
	if (err != ESP_OK)
		return err;

	s_current_level = 0;
	return ESP_OK;
}

esp_err_t dac_mcp4728_enable_autonomous(void)
{
	if (!s_initialized)
	{
		ESP_LOGE(TAG, "Not initialized");
		return ESP_ERR_INVALID_STATE;
	}

	esp_err_t err = write_channel_a(0);
	if (err != ESP_OK)
		return err;

	s_current_level = 0;
	s_autonomous = true;

	ESP_LOGI(TAG, "Autonomous mode ENABLED");
	return ESP_OK;
}

esp_err_t dac_mcp4728_emergency_stop(void)
{
	if (!s_initialized)
		return ESP_ERR_INVALID_STATE;

	(void)write_channel_a(0);
	s_current_level = 0;
	s_autonomous = false;

	ESP_LOGI(TAG, "EMERGENCY STOP");
	return ESP_OK;
}

uint16_t dac_mcp4728_get_level(void)
{
	return s_current_level;
}

bool dac_mcp4728_is_autonomous(void)
{
	return s_autonomous;
}
