/**
 * @file relay_srd05vdc.cpp
 * @brief GPIO-driven AEDIKO SRD-05VDC-SL-C relay module driver.
 *
 * Supports multiple independent instances. Each relay is identified by its
 * config (GPIO pin + polarity). State is determined by reading the GPIO
 * level directly — no static shadow variable.
 */
#include "relay_srd05vdc.hh"

// ============================================================================
// Initialization
// ============================================================================

esp_err_t relay_srd05vdc_init(const relay_srd05vdc_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    // Preload output latch to safe (de-energized) level before enabling output.
    int safe_level = config->active_high ? 0 : 1;
    esp_err_t err = gpio_set_level(config->gpio, safe_level);
    if (err != ESP_OK) return err;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << config->gpio,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = config->enable_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = config->enable_pulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;

    // Start in safe state (de-energized)
    err = relay_srd05vdc_disable(config);
    if (err != ESP_OK) return err;

    // Verify readback matches safe state.
    if (relay_srd05vdc_is_enabled(config)) return ESP_FAIL;

    return ESP_OK;
}

// ============================================================================
// Relay Control
// ============================================================================

// Enable relay - energizes coil, closes NO contact
esp_err_t relay_srd05vdc_enable(const relay_srd05vdc_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    int level = config->active_high ? 1 : 0;
    return gpio_set_level(config->gpio, level);
}

// Disable relay - de-energizes coil, opens NO contact (safe state)
esp_err_t relay_srd05vdc_disable(const relay_srd05vdc_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    int level = config->active_high ? 0 : 1;
    return gpio_set_level(config->gpio, level);
}

// Read GPIO state directly to determine if relay is enabled
bool relay_srd05vdc_is_enabled(const relay_srd05vdc_config_t *config) {
    if (!config) return false;
    int level = gpio_get_level(config->gpio);
    return config->active_high ? (level == 1) : (level == 0);
}
