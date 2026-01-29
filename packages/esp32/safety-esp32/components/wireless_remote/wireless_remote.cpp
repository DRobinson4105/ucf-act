#include "wireless_remote.hh"

// configure the GPIO for the wireless remote input
esp_err_t wireless_remote_init(const wireless_remote_config_t *config) {
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

// read the GPIO level and return true when active
bool wireless_remote_is_active(const wireless_remote_config_t *config) {
    if (!config) return false;

    int level = gpio_get_level(config->gpio);
    return level == config->active_level;
}
