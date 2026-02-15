/**
 * @file multiplexer_dg408djz.cpp
 * @brief DG408DJZ 8-channel analog multiplexer throttle control implementation.
 */
#include "multiplexer_dg408djz.hh"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

namespace {

static const char *TAG = "MULTIPLEXER";

// ============================================================================
// Timing Constants
// ============================================================================

constexpr uint32_t ADDRESS_SETTLE_US = 10;      // Wait for address lines before enabling mux
constexpr uint32_t RELAY_SETTLE_MS = 50;        // Relay contact bounce/settle time

// ============================================================================
// Module State
// ============================================================================

static multiplexer_dg408djz_config_t s_config = {};
static bool s_initialized = false;
static int8_t s_current_level = -1;  // -1 = mux disabled, 0-7 = active channel
static bool s_autonomous = false;    // true = relay energized (mux output to Curtis)

static esp_err_t set_and_verify(gpio_num_t gpio, int level) {
    esp_err_t err = gpio_set_level(gpio, level);
    if (err != ESP_OK) return err;
    if (gpio_get_level(gpio) != level) return ESP_ERR_INVALID_STATE;
    return ESP_OK;
}

}  // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t multiplexer_dg408djz_init(const multiplexer_dg408djz_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    s_config = *config;

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->a0) | (1ULL << config->a1) |
                        (1ULL << config->a2) | (1ULL << config->en) |
                        (1ULL << config->relay),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    esp_err_t err = gpio_config(&io_conf);
    if (err != ESP_OK) return err;

    // Safe state: mux disabled, relay de-energized (manual pedal control)
    gpio_set_level(config->a0, 0);
    gpio_set_level(config->a1, 0);
    gpio_set_level(config->a2, 0);
    gpio_set_level(config->en, 0);
    gpio_set_level(config->relay, 0);

    // Verify MCU-side output levels reached expected safe defaults.
    if (gpio_get_level(config->a0) != 0 ||
        gpio_get_level(config->a1) != 0 ||
        gpio_get_level(config->a2) != 0 ||
        gpio_get_level(config->en) != 0 ||
        gpio_get_level(config->relay) != 0) {
        return ESP_ERR_INVALID_STATE;
    }

    s_current_level = -1;
    s_autonomous = false;
    s_initialized = true;

    return ESP_OK;
}

// ============================================================================
// Level Control
// ============================================================================

// Set throttle level 0-7, or <0 to disable mux output
esp_err_t multiplexer_dg408djz_set_level(int8_t level) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (level < 0 || level > 7) {
        esp_err_t err = set_and_verify(s_config.en, 0);
        if (err != ESP_OK) return err;
        s_current_level = -1;
#ifdef CONFIG_LOG_ACTUATOR_MUX_LEVEL
        ESP_LOGI(TAG, "Disabled (level=%d)", level);
#endif
        return ESP_OK;
    }

    // Set address lines BEFORE enabling - prevents glitching through wrong channel
    esp_err_t err = set_and_verify(s_config.a0, (level >> 0) & 1);
    if (err != ESP_OK) return err;
    err = set_and_verify(s_config.a1, (level >> 1) & 1);
    if (err != ESP_OK) return err;
    err = set_and_verify(s_config.a2, (level >> 2) & 1);
    if (err != ESP_OK) return err;

    ets_delay_us(ADDRESS_SETTLE_US);

    err = set_and_verify(s_config.en, 1);
    if (err != ESP_OK) return err;
    s_current_level = level;

#ifdef CONFIG_LOG_ACTUATOR_MUX_LEVEL
    ESP_LOGI(TAG, "Level set to %d (A2=%d A1=%d A0=%d)",
             level, (level >> 2) & 1, (level >> 1) & 1, (level >> 0) & 1);
#endif
    return ESP_OK;
}

esp_err_t multiplexer_dg408djz_disable(void) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

    esp_err_t err = set_and_verify(s_config.en, 0);
    if (err != ESP_OK) return err;
    s_current_level = -1;
#ifdef CONFIG_LOG_ACTUATOR_MUX_LEVEL
    ESP_LOGI(TAG, "Mux disabled");
#endif
    return ESP_OK;
}

int8_t multiplexer_dg408djz_get_level(void) {
    return s_current_level;
}

// ============================================================================
// Autonomous Mode Control
// ============================================================================

// Enable autonomous throttle control
// Sequence: set level 0 -> energize relay -> wait for settle
// After this, Curtis motor controller reads mux output instead of pedal pot
esp_err_t multiplexer_dg408djz_enable_autonomous(void) {
    if (!s_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }

#ifdef CONFIG_LOG_ACTUATOR_MUX_LEVEL
    ESP_LOGI(TAG, "Enabling autonomous mode");
#endif

    // Start at idle throttle to prevent surge when relay switches
    esp_err_t err = multiplexer_dg408djz_set_level(0);
    if (err != ESP_OK) return err;
    vTaskDelay(pdMS_TO_TICKS(10));

    // Energize DPDT relay - switches Curtis input from pedal to mux
    err = set_and_verify(s_config.relay, 1);
    if (err != ESP_OK) return err;
    s_autonomous = true;

    // Wait for relay contacts to settle before allowing throttle changes
    vTaskDelay(pdMS_TO_TICKS(RELAY_SETTLE_MS));

#ifdef CONFIG_LOG_ACTUATOR_MUX_LEVEL
    ESP_LOGI(TAG, "Autonomous mode ENABLED");
#endif
    return ESP_OK;
}

// Immediate cutoff - no ramp down, used for override/fault conditions
esp_err_t multiplexer_dg408djz_emergency_stop(void) {
    if (!s_initialized) return ESP_ERR_INVALID_STATE;

#ifdef CONFIG_LOG_ACTUATOR_MUX_LEVEL
    ESP_LOGI(TAG, "EMERGENCY STOP");
#endif

    // Immediately disable mux output
    esp_err_t err = set_and_verify(s_config.en, 0);
    if (err != ESP_OK) return err;
    s_current_level = -1;

    // De-energize relay - NC contacts restore manual pedal control
    err = set_and_verify(s_config.relay, 0);
    if (err != ESP_OK) return err;
    s_autonomous = false;
    return ESP_OK;
}

bool multiplexer_dg408djz_is_autonomous(void) {
    return s_autonomous;
}
