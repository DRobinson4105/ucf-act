/**
 * @file relay_srd05vdc.cpp
 * @brief GPIO-driven SRD-05VDC-SL-C relay implementation.
 */
#include "relay_srd05vdc.hh"

#include "esp_log.h"

namespace {

[[maybe_unused]] static const char *TAG = "RELAY";

// ============================================================================
// Module State
// ============================================================================

static bool s_enabled = false;

}  // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t relay_srd05vdc_init(const relay_srd05vdc_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << config->gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = config->enable_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = config->enable_pulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;

    // Start in safe state (de-energized)
    err = relay_srd05vdc_disable(config);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

// ============================================================================
// Relay Control
// ============================================================================

// Enable relay - energizes coil, closes NO contact
esp_err_t relay_srd05vdc_enable(const relay_srd05vdc_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    int level = config->active_high ? 1 : 0;
    esp_err_t err = gpio_set_level(config->gpio, level);
    if (err == ESP_OK) {
#ifdef CONFIG_LOG_ACTUATOR_POWER_RELAY_STATE
        if (!s_enabled) ESP_LOGI(TAG, "Power relay ENABLED");
#endif
        s_enabled = true;
    }
    return err;
}

// Disable relay - de-energizes coil, opens NO contact (safe state)
esp_err_t relay_srd05vdc_disable(const relay_srd05vdc_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    int level = config->active_high ? 0 : 1;
    esp_err_t err = gpio_set_level(config->gpio, level);
    if (err == ESP_OK) {
#ifdef CONFIG_LOG_ACTUATOR_POWER_RELAY_STATE
        if (s_enabled) ESP_LOGI(TAG, "Power relay DISABLED");
#endif
        s_enabled = false;
    }
    return err;
}

bool relay_srd05vdc_is_enabled(const relay_srd05vdc_config_t *config) {
    (void)config;
    return s_enabled;
}
