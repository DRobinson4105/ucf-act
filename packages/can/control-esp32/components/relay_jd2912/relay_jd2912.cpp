/**
 * @file relay_jd2912.cpp
 * @brief JD-2912 automotive relay driver implementation (via IRLZ44N MOSFET).
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

}  // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t relay_jd2912_init(const relay_jd2912_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    s_config = *config;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << config->gpio,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

    // Start de-energized - pedal microswitch operates normally
    int level = config->active_high ? 0 : 1;
    gpio_set_level(config->gpio, level);
    s_energized = false;
    s_initialized = true;

    ESP_LOGI(TAG, "Initialized on GPIO %d (active %s), starting DE-ENERGIZED",
             config->gpio, config->active_high ? "HIGH" : "LOW");

    return ESP_OK;
}

// ============================================================================
// Relay Control
// ============================================================================

// Energize relay to bypass pedal microswitch (for autonomous mode)
void relay_jd2912_energize(void) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return;
    }

    int level = s_config.active_high ? 1 : 0;
    gpio_set_level(s_config.gpio, level);
    s_energized = true;

#ifdef CONFIG_LOG_PEDAL_RELAY
    ESP_LOGI(TAG, "ENERGIZED (pedal bypass active)");
#endif
}

// De-energize relay to restore normal pedal operation
void relay_jd2912_deenergize(void) {
    if (!s_initialized) return;

    int level = s_config.active_high ? 0 : 1;
    gpio_set_level(s_config.gpio, level);
    s_energized = false;

#ifdef CONFIG_LOG_PEDAL_RELAY
    ESP_LOGI(TAG, "DE-ENERGIZED (normal pedal operation)");
#endif
}

bool relay_jd2912_is_energized(void) {
    return s_energized;
}
