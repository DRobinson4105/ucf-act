/**
 * @file relay_dpdt_my5nj.cpp
 * @brief MY5NJ DPDT relay driver implementation.
 */

#include "relay_dpdt_my5nj.h"

#include "esp_log.h"

namespace
{

const char *TAG = "DPDT_RELAY";

relay_dpdt_my5nj_config_t s_config = {};
bool s_initialized = false;

esp_err_t set_and_verify_level(int level)
{
    esp_err_t err = gpio_set_level(s_config.gpio, level);
    if (err != ESP_OK) {
        return err;
    }
    if (gpio_get_level(s_config.gpio) != level) {
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_OK;
}

} // namespace

esp_err_t relay_dpdt_my5nj_init(const relay_dpdt_my5nj_config_t *config)
{
    if (config == nullptr) {
        return ESP_ERR_INVALID_ARG;
    }

    s_config = *config;

    esp_err_t err = gpio_set_level(config->gpio, 0);
    if (err != ESP_OK) {
        return err;
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << config->gpio,
        .mode = GPIO_MODE_INPUT_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        return err;
    }

    err = set_and_verify_level(0);
    if (err != ESP_OK) {
        return err;
    }

    s_initialized = true;
    return ESP_OK;
}

esp_err_t relay_dpdt_my5nj_energize(void)
{
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = set_and_verify_level(1);
    if (err != ESP_OK) {
        return err;
    }

#ifdef CONFIG_LOG_ACTUATOR_DPDT_RELAY_CHANGES
    ESP_LOGI(TAG, "ENERGIZED");
#endif
    return ESP_OK;
}

esp_err_t relay_dpdt_my5nj_deenergize(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = set_and_verify_level(0);
    if (err != ESP_OK) {
        return err;
    }

#ifdef CONFIG_LOG_ACTUATOR_DPDT_RELAY_CHANGES
    ESP_LOGI(TAG, "DE-ENERGIZED");
#endif
    return ESP_OK;
}

bool relay_dpdt_my5nj_is_energized(void)
{
    if (!s_initialized) {
        return false;
    }
    return gpio_get_level(s_config.gpio) == 1;
}
