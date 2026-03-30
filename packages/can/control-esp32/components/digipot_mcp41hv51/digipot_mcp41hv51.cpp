/**
 * @file digipot_mcp41hv51.cpp
 * @brief MCP41HV51 digital potentiometer throttle control implementation.
 */
#include "digipot_mcp41hv51.h"

#include "esp_log.h"

namespace
{

const char *TAG = "DIGIPOT";

// ============================================================================
// SPI Constants
// ============================================================================

constexpr uint8_t CMD_WRITE_VOLATILE_WIPER = 0x00; // MCP41HV51 write command (addr=0000, cmd=00)
constexpr uint8_t CMD_READ_VOLATILE_WIPER = 0x0C;  // MCP41HV51 read command (addr=0000, cmd=11)
constexpr int SPI_CLOCK_HZ = 100000;               // 100 kHz SPI clock (reduced for long cable runs)

// ============================================================================
// Module State
// ============================================================================

digipot_mcp41hv51_config_t s_config = {};
bool s_initialized = false;
uint8_t s_current_wiper = 0; // 0 = disabled/idle
bool s_autonomous = false;   // true = autonomous mode active (digipot output to Curtis)
spi_device_handle_t s_spi_handle = nullptr;
bool s_bus_initialized = false;

/**
 * @brief Release SPI device/bus resources and reset module state.
 */
void release_resources()
{
	if (s_spi_handle)
	{
		(void)spi_bus_remove_device(s_spi_handle);
		s_spi_handle = nullptr;
	}
	if (s_bus_initialized)
	{
		(void)spi_bus_free(s_config.spi_host);
		s_bus_initialized = false;
	}
	s_initialized = false;
	s_autonomous = false;
	s_current_wiper = 0;
}

/**
 * @brief Write wiper position to the MCP41HV51 via SPI.
 *
 * Sends a 2-byte SPI transaction: command byte (0x00 = write volatile wiper)
 * followed by the 8-bit wiper position.
 *
 * @param position  Wiper position 0-255
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t write_wiper(uint8_t position)
{
	if (!s_spi_handle)
		return ESP_ERR_INVALID_STATE;

	uint8_t tx_data[2] = {CMD_WRITE_VOLATILE_WIPER, position};

	spi_transaction_t txn = {};
	txn.length = 16;
	txn.tx_buffer = tx_data;

	return spi_device_transmit(s_spi_handle, &txn);
}

/**
 * @brief Read the volatile wiper register from the MCP41HV51 via SPI.
 *
 * Sends a read command and returns the wiper value from SDO.
 *
 * @param out_position  [out] Wiper position read from the chip
 * @return ESP_OK on success, or an error code on failure
 */
esp_err_t read_wiper(uint8_t *out_position)
{
	if (!s_spi_handle || !out_position)
		return ESP_ERR_INVALID_STATE;

	uint8_t tx_data[2] = {CMD_READ_VOLATILE_WIPER, 0x00};
	uint8_t rx_data[2] = {};

	spi_transaction_t txn = {};
	txn.length = 16;
	txn.tx_buffer = tx_data;
	txn.rx_buffer = rx_data;

	esp_err_t err = spi_device_transmit(s_spi_handle, &txn);
	if (err != ESP_OK)
		return err;

	*out_position = rx_data[1]; // Wiper value is in the second byte
	return ESP_OK;
}

} // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t digipot_mcp41hv51_init(const digipot_mcp41hv51_config_t *config)
{
	if (!config)
		return ESP_ERR_INVALID_ARG;

	// Re-init path: release any previous SPI resources first.
	if (s_initialized || s_spi_handle || s_bus_initialized)
	{
		release_resources();
	}

	s_config = *config;

	// Initialize SPI bus
	spi_bus_config_t bus_cfg = {};
	bus_cfg.mosi_io_num = config->sdi; // SDI on MCP41HV51 datasheet
	bus_cfg.miso_io_num = config->sdo; // SDO on MCP41HV51 (read-back verification)
	bus_cfg.sclk_io_num = config->sck;
	bus_cfg.quadwp_io_num = -1;
	bus_cfg.quadhd_io_num = -1;
	bus_cfg.max_transfer_sz = 2;

	esp_err_t err = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_DISABLED);
	if (err == ESP_ERR_INVALID_STATE)
	{
		// Recover from stale host state and retry once.
#ifdef CONFIG_LOG_RETRY_DIGIPOT
		ESP_LOGW(TAG, "SPI bus stale, freeing and retrying");
#endif
		(void)spi_bus_free(config->spi_host);
		err = spi_bus_initialize(config->spi_host, &bus_cfg, SPI_DMA_DISABLED);
	}
	if (err != ESP_OK)
		return err;
	s_bus_initialized = true;

	// Add MCP41HV51 as SPI device
	spi_device_interface_config_t dev_cfg = {};
	dev_cfg.clock_speed_hz = SPI_CLOCK_HZ;
	dev_cfg.mode = 0; // SPI mode 0 (CPOL=0, CPHA=0)
	dev_cfg.spics_io_num = config->cs;
	dev_cfg.queue_size = 1;

	err = spi_bus_add_device(config->spi_host, &dev_cfg, &s_spi_handle);
	if (err != ESP_OK)
	{
		release_resources();
		return err;
	}

	// Safe state: wiper at position 0
	err = write_wiper(0);
	if (err != ESP_OK)
	{
		release_resources();
		return err;
	}

	s_current_wiper = 0;
	s_autonomous = false;
	s_initialized = true;

	return ESP_OK;
}

// ============================================================================
// Wiper Control
// ============================================================================

esp_err_t digipot_mcp41hv51_set_wiper(uint8_t position)
{
	if (!s_initialized)
	{
		ESP_LOGE(TAG, "Not initialized");
		return ESP_ERR_INVALID_STATE;
	}

	esp_err_t err = write_wiper(position);
	if (err != ESP_OK)
		return err;

	// Read back and verify the wiper register matches what we wrote.
	uint8_t readback = 0xFF;
	err = read_wiper(&readback);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "Wiper readback failed: %s", esp_err_to_name(err));
		return err;
	}

	// Log readback periodically or on mismatch
	static uint16_t s_verify_count = 0;
	if (readback != position || (++s_verify_count % 50) == 0)
	{
		ESP_LOGI(TAG, "Wiper: wrote=%d read=%d %s", position, readback,
		         (readback == position) ? "OK" : "MISMATCH");
	}

	if (readback != position)
		return ESP_ERR_INVALID_RESPONSE;

	s_current_wiper = position;
	return ESP_OK;
}

esp_err_t digipot_mcp41hv51_disable(void)
{
	if (!s_initialized)
		return ESP_ERR_INVALID_STATE;

	esp_err_t err = write_wiper(0);
	if (err != ESP_OK)
		return err;

	s_current_wiper = 0;

#ifdef CONFIG_LOG_ACTUATOR_DIGIPOT_WIPER
	ESP_LOGI(TAG, "Digipot disabled (wiper=0)");
#endif
	return ESP_OK;
}

uint8_t digipot_mcp41hv51_get_wiper(void)
{
	return s_current_wiper;
}

// ============================================================================
// Autonomous Mode Control
// ============================================================================

esp_err_t digipot_mcp41hv51_enable_autonomous(void)
{
	if (!s_initialized)
	{
		ESP_LOGE(TAG, "Not initialized");
		return ESP_ERR_INVALID_STATE;
	}

#ifdef CONFIG_LOG_ACTUATOR_DIGIPOT_WIPER
	ESP_LOGI(TAG, "Enabling autonomous mode");
#endif

	// Start at idle throttle
	esp_err_t err = write_wiper(0);
	if (err != ESP_OK)
		return err;

	s_current_wiper = 0;
	s_autonomous = true;

#ifdef CONFIG_LOG_ACTUATOR_DIGIPOT_WIPER
	ESP_LOGI(TAG, "Autonomous mode ENABLED");
#endif
	return ESP_OK;
}

esp_err_t digipot_mcp41hv51_emergency_stop(void)
{
	if (!s_initialized)
		return ESP_ERR_INVALID_STATE;

#ifdef CONFIG_LOG_ACTUATOR_DIGIPOT_WIPER
	ESP_LOGI(TAG, "EMERGENCY STOP");
#endif

	// Immediately zero the wiper
	esp_err_t err = write_wiper(0);
	if (err != ESP_OK)
		return err;

	s_current_wiper = 0;
	s_autonomous = false;
	return ESP_OK;
}

bool digipot_mcp41hv51_is_autonomous(void)
{
	return s_autonomous;
}
