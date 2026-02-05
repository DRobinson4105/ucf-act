#include "push_button_hb2es544.hh"

// =============================================================================
// Initialization
// =============================================================================

esp_err_t push_button_hb2es544_init(const push_button_hb2es544_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << config->gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = config->enable_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = config->enable_pulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    return gpio_config(&io_conf);
}

// =============================================================================
// State Reading
// =============================================================================

// Returns true when e-stop is active (mushroom button pressed)
bool push_button_hb2es544_read_active(const push_button_hb2es544_config_t *config) {
    if (!config) return false;
    return gpio_get_level(config->gpio) == config->active_level;
}
