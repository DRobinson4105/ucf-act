/**
 * @file push_button_hb2es544.cpp
 * @brief HB2-ES544 push-button e-stop input implementation.
 */
#include "push_button_hb2es544.hh"

#include "esp_log.h"

namespace {

[[maybe_unused]] static const char *TAG = "PUSH_BUTTON";

}  // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t push_button_hb2es544_init(const push_button_hb2es544_config_t *config) {
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

// Returns true when e-stop is active (push button pressed)
bool push_button_hb2es544_read_active(const push_button_hb2es544_config_t *config) {
    if (!config) return true;  // fail-safe: treat as active (e-stop triggered)
    bool active = gpio_get_level(config->gpio) == config->active_level;

#ifdef CONFIG_LOG_INPUT_PUSH_BUTTON
    static bool s_prev = false;
    static bool s_first = true;
    if (active != s_prev || s_first) {
        ESP_LOGI(TAG, "Push button %s", active ? "PRESSED" : "released");
        s_prev = active;
        s_first = false;
    }
#endif

    return active;
}
