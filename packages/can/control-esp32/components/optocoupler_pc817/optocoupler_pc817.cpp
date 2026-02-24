/**
 * @file optocoupler_pc817.cpp
 * @brief F/R switch decode + debounce for dual PC817 inputs.
 */

#include "optocoupler_pc817.hh"

#include "esp_log.h"

namespace {

[[maybe_unused]] static const char *TAG = "OPTOCOUPLER_PC817";

static bool s_initialized = false;
static optocoupler_pc817_config_t s_config = {};
static fr_state_t s_state_debounced = FR_STATE_NEUTRAL;
static fr_state_t s_state_pending = FR_STATE_NEUTRAL;
static uint32_t s_change_time_ms = 0;
static bool s_debouncing = false;

/**
 * Decode F/R state from dual PC817 optocoupler inputs.
 *
 * 1999 Club Car DS 48V wiring:
 *   forward_gpio (GPIO 22) = anti-arcing microswitch (ON in Forward AND Reverse)
 *   reverse_gpio (GPIO 23) = reverse buzzer microswitch (ON in Reverse only)
 *
 * Both active  = Reverse (anti-arc + buzzer both ON)
 * Forward only = Forward (anti-arc ON, buzzer OFF)
 * Neither      = Neutral (handle in center detent)
 * Reverse only = Invalid (buzzer without anti-arc is a wiring fault)
 */
static fr_state_t read_state_raw_internal(void) {
    bool forward_active = (gpio_get_level(s_config.forward_gpio) == 0);
    bool reverse_active = (gpio_get_level(s_config.reverse_gpio) == 0);

    if (forward_active && !reverse_active) return FR_STATE_FORWARD;
    if (forward_active && reverse_active)  return FR_STATE_REVERSE;
    if (!forward_active && !reverse_active) return FR_STATE_NEUTRAL;
    return FR_STATE_INVALID;  // reverse_active without forward = wiring fault
}

}  // namespace

esp_err_t optocoupler_pc817_init(const optocoupler_pc817_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    s_config = *config;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->forward_gpio) | (1ULL << config->reverse_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;

    s_state_debounced = read_state_raw_internal();
    s_state_pending = s_state_debounced;
    s_change_time_ms = 0;
    s_debouncing = false;
    s_initialized = true;

    return ESP_OK;
}

fr_state_t optocoupler_pc817_get_state(void) {
    if (!s_initialized) return FR_STATE_NEUTRAL;
    return s_state_debounced;
}

fr_state_t optocoupler_pc817_get_state_raw(void) {
    if (!s_initialized) return FR_STATE_NEUTRAL;
    return read_state_raw_internal();
}

void optocoupler_pc817_update(uint32_t now_ms) {
    if (!s_initialized) return;

    fr_state_t current_raw = read_state_raw_internal();

    if (current_raw != s_state_debounced) {
        if (!s_debouncing) {
            s_state_pending = current_raw;
            s_change_time_ms = now_ms;
            s_debouncing = true;
#ifdef CONFIG_LOG_INPUT_FR_DEBOUNCE
            ESP_LOGI(TAG, "F/R change detected, starting debounce: %d -> %d",
                     s_state_debounced,
                     current_raw);
#endif
        } else if (current_raw != s_state_pending) {
            s_state_pending = current_raw;
            s_change_time_ms = now_ms;
#ifdef CONFIG_LOG_INPUT_FR_DEBOUNCE
            ESP_LOGI(TAG, "F/R changed during debounce, resetting: -> %d", current_raw);
#endif
        } else if ((now_ms - s_change_time_ms) >= FR_PC817_DEBOUNCE_MS) {
#ifdef CONFIG_LOG_INPUT_FR_STATE
            ESP_LOGI(TAG, "F/R state changed: %d -> %d", s_state_debounced, s_state_pending);
#endif
            s_state_debounced = s_state_pending;
            s_debouncing = false;
        }
    } else {
#ifdef CONFIG_LOG_INPUT_FR_DEBOUNCE
        if (s_debouncing) {
            ESP_LOGI(TAG, "F/R returned to stable state, canceling debounce");
        }
#endif
        s_debouncing = false;
    }
}
