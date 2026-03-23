/**
 * @file relay_srd05vdc.cpp
 * @brief GPIO-driven AEDIKO SRD-05VDC-SL-C relay module driver.
 *
 * Supports multiple independent instances. Each relay is identified by its
 * config (GPIO pin). State is determined by reading the GPIO level
 * directly — no static shadow variable.
 */
#include "relay_srd05vdc.h"

// ============================================================================
// Initialization
// ============================================================================

esp_err_t relay_srd05vdc_init(const relay_srd05vdc_config_t *config)
{
	if (!config)
		return ESP_ERR_INVALID_ARG;

	// Preload output latch to safe (de-energized) level before enabling output.
	// Active-high: LOW = de-energized = safe.
	esp_err_t err = gpio_set_level(config->gpio, 0);
	if (err != ESP_OK)
		return err;

	gpio_config_t io_conf = {
		.pin_bit_mask = 1ULL << config->gpio,
		.mode = GPIO_MODE_INPUT_OUTPUT,
		.pull_up_en = GPIO_PULLUP_DISABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE,
		.intr_type = GPIO_INTR_DISABLE,
	};

	err = gpio_config(&io_conf);
	if (err != ESP_OK)
		return err;

	// Start in safe state (de-energized)
	err = relay_srd05vdc_disable(config);
	if (err != ESP_OK)
		return err;

	// Verify readback matches safe state.
	if (relay_srd05vdc_is_enabled(config))
		return ESP_FAIL;

	return ESP_OK;
}

// ============================================================================
// Relay Control
// ============================================================================

esp_err_t relay_srd05vdc_enable(const relay_srd05vdc_config_t *config)
{
	if (!config)
		return ESP_ERR_INVALID_ARG;
	return gpio_set_level(config->gpio, 1);
}

esp_err_t relay_srd05vdc_disable(const relay_srd05vdc_config_t *config)
{
	if (!config)
		return ESP_ERR_INVALID_ARG;
	return gpio_set_level(config->gpio, 0);
}

bool relay_srd05vdc_is_enabled(const relay_srd05vdc_config_t *config)
{
	if (!config)
		return false;
	return gpio_get_level(config->gpio) == 1;
}
