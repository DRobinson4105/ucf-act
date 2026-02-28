/**
 * @file relay_jd2912.cpp
 * @brief JD-2912 automotive relay driver implementation (via S8050 NPN transistor).
 */
#include "relay_jd2912.hh"

#include "esp_log.h"

namespace {

static const char *TAG = "RELAY";

// ============================================================================
// Module State
// ============================================================================

static relay_jd2912_config_t s_config = {};
static bool s_initialized = false;
static bool s_energized = false;

static esp_err_t set_and_verify_level(int level) {
    esp_err_t err = gpio_set_level(s_config.gpio, level);
    if (err != ESP_OK) return err;
    if (gpio_get_level(s_config.gpio) != level) return ESP_ERR_INVALID_STATE;
    return ESP_OK;
}

}  // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t relay_jd2912_init(const relay_jd2912_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    s_config = *config;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << config->gpio,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;

    // Start de-energized - pedal microswitch operates normally
    int level = config->active_high ? 0 : 1;
    gpio_set_level(config->gpio, level);
    if (gpio_get_level(config->gpio) != level) {
        return ESP_ERR_INVALID_STATE;
    }
    s_energized = false;
    s_initialized = true;

    return ESP_OK;
}

// ============================================================================
// Relay Control
// ============================================================================

// Energize relay to bypass pedal microswitch (for autonomous mode)
esp_err_t relay_jd2912_energize(void) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    int level = s_config.active_high ? 1 : 0;
    esp_err_t err = set_and_verify_level(level);
    if (err != ESP_OK) return err;
    s_energized = true;

#ifdef CONFIG_LOG_ACTUATOR_PEDAL_RELAY
    ESP_LOGI(TAG, "ENERGIZED (pedal bypass active)");
#endif
    return ESP_OK;
}

// De-energize relay to restore normal pedal operation
esp_err_t relay_jd2912_deenergize(void) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    int level = s_config.active_high ? 0 : 1;
    esp_err_t err = set_and_verify_level(level);
    if (err != ESP_OK) return err;
    s_energized = false;

#ifdef CONFIG_LOG_ACTUATOR_PEDAL_RELAY
    ESP_LOGI(TAG, "DE-ENERGIZED (normal pedal operation)");
#endif
    return ESP_OK;
}

bool relay_jd2912_is_energized(void) {
    return s_energized;
}
