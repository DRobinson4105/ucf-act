/**
 * @file gpio_input.cpp
 * @brief Generic GPIO digital input driver implementation.
 */
#include "gpio_input.hh"

#include "esp_log.h"

// ============================================================================
// Initialization
// ============================================================================

esp_err_t gpio_input_init(const gpio_input_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << config->gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = config->enable_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = config->enable_pulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;

    return ESP_OK;
}

// ============================================================================
// State Reading
// ============================================================================

bool gpio_input_is_active(const gpio_input_config_t *config) {
    if (!config) return true;  // fail-safe: treat as active
    return gpio_get_level(config->gpio) == config->active_level;
}
