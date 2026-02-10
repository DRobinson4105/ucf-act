/**
 * @file power_relay.cpp
 * @brief GPIO-driven power relay implementation.
 */
#include "power_relay.hh"

#include "esp_log.h"

namespace {

static const char *TAG = "POWER_RELAY";

// ============================================================================
// Module State
// ============================================================================

static bool s_enabled = false;

}  // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t power_relay_init(const power_relay_config_t *config) {
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

    // Start in safe state (disabled)
    err = power_relay_disable(config);
    if (err != ESP_OK) return err;

    ESP_LOGI(TAG, "Power relay init on GPIO %d (active %s)",
             config->gpio, config->active_high ? "HIGH" : "LOW");
    return ESP_OK;
}

// ============================================================================
// Relay Control
// ============================================================================

// Enable power relay - allows power to flow to motor controller
esp_err_t power_relay_enable(const power_relay_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    int level = config->active_high ? 1 : 0;
    esp_err_t err = gpio_set_level(config->gpio, level);
    if (err == ESP_OK) {
        if (!s_enabled) ESP_LOGI(TAG, "Power relay ENABLED");
        s_enabled = true;
    }
    return err;
}

// Disable power relay - cuts power to motor controller (safe state)
esp_err_t power_relay_disable(const power_relay_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    int level = config->active_high ? 0 : 1;
    esp_err_t err = gpio_set_level(config->gpio, level);
    if (err == ESP_OK) {
        if (s_enabled) ESP_LOGI(TAG, "Power relay DISABLED (safe state)");
        s_enabled = false;
    }
    return err;
}

bool power_relay_is_enabled(const power_relay_config_t *config) {
    (void)config;
    return s_enabled;
}
