/**
 * @file rf_remote_ev1527.cpp
 * @brief EV1527 RF remote e-stop input implementation.
 */
#include "rf_remote_ev1527.hh"

#include "esp_log.h"

namespace {

static const char *TAG = "RF_REMOTE";

}  // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t rf_remote_ev1527_init(const rf_remote_ev1527_config_t *config) {
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

    ESP_LOGI(TAG, "Initialized on GPIO %d (active %s)",
             config->gpio, config->active_level ? "HIGH" : "LOW");
    return ESP_OK;
}

// ============================================================================
// State Reading
// ============================================================================

// Returns true when remote e-stop button is pressed
bool rf_remote_ev1527_is_active(const rf_remote_ev1527_config_t *config) {
    if (!config) return true;  // fail-safe: treat as active (e-stop triggered)
    bool active = gpio_get_level(config->gpio) == config->active_level;

#ifdef CONFIG_LOG_RF_REMOTE
    static bool s_prev = false;
    static bool s_first = true;
    if (active != s_prev || s_first) {
        ESP_LOGI(TAG, "RF remote %s", active ? "ENGAGED" : "disengaged");
        s_prev = active;
        s_first = false;
    }
#endif

    return active;
}
