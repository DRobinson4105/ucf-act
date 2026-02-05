#include "throttle_mux.hh"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "rom/ets_sys.h"

namespace {

static const char *TAG = "THROTTLE_MUX";

// =============================================================================
// Timing Constants
// =============================================================================

constexpr uint32_t ADDRESS_SETTLE_US = 10;      // Wait for address lines before enabling mux
constexpr uint32_t RELAY_SETTLE_MS = 50;        // Relay contact bounce/settle time
constexpr uint32_t THROTTLE_REGISTER_MS = 100;  // Time for Curtis controller to see low throttle

// =============================================================================
// Module State
// =============================================================================

static throttle_mux_config_t s_config = {};
static bool s_initialized = false;
static int8_t s_current_level = -1;  // -1 = mux disabled, 0-7 = active channel
static bool s_autonomous = false;    // true = relay energized (mux output to Curtis)

}  // namespace

// =============================================================================
// Initialization
// =============================================================================

esp_err_t throttle_mux_init(const throttle_mux_config_t *config) {
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
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        return err;
    }

    // Safe state: mux disabled, relay de-energized (manual pedal control)
    gpio_set_level(config->a0, 0);
    gpio_set_level(config->a1, 0);
    gpio_set_level(config->a2, 0);
    gpio_set_level(config->en, 0);
    gpio_set_level(config->relay, 0);

    s_current_level = -1;
    s_autonomous = false;
    s_initialized = true;

    ESP_LOGI(TAG, "Initialized: A0=%d A1=%d A2=%d EN=%d RELAY=%d",
             config->a0, config->a1, config->a2, config->en, config->relay);

    return ESP_OK;
}

// =============================================================================
// Level Control
// =============================================================================

// Set throttle level 0-7, or <0 to disable mux output
void throttle_mux_set_level(int8_t level) {
    if (!s_initialized) {
        ESP_LOGW(TAG, "Not initialized");
        return;
    }

    if (level < 0 || level > 7) {
        gpio_set_level(s_config.en, 0);
        s_current_level = -1;
        ESP_LOGD(TAG, "Disabled (level=%d)", level);
        return;
    }

    // Set address lines BEFORE enabling - prevents glitching through wrong channel
    gpio_set_level(s_config.a0, (level >> 0) & 1);
    gpio_set_level(s_config.a1, (level >> 1) & 1);
    gpio_set_level(s_config.a2, (level >> 2) & 1);

    ets_delay_us(ADDRESS_SETTLE_US);

    gpio_set_level(s_config.en, 1);
    s_current_level = level;

    ESP_LOGD(TAG, "Level set to %d (A2=%d A1=%d A0=%d)",
             level, (level >> 2) & 1, (level >> 1) & 1, (level >> 0) & 1);
}

void throttle_mux_disable(void) {
    if (!s_initialized) return;

    gpio_set_level(s_config.en, 0);
    s_current_level = -1;
    ESP_LOGD(TAG, "Mux disabled");
}

int8_t throttle_mux_get_level(void) {
    return s_current_level;
}

// =============================================================================
// Autonomous Mode Control
// =============================================================================

// Enable autonomous throttle control
// Sequence: set level 0 -> energize relay -> wait for settle
// After this, Curtis motor controller reads mux output instead of pedal pot
void throttle_mux_enable_autonomous(void) {
    if (!s_initialized) {
        ESP_LOGW(TAG, "Not initialized");
        return;
    }

    ESP_LOGI(TAG, "Enabling autonomous mode");

    // Start at idle throttle to prevent surge when relay switches
    throttle_mux_set_level(0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Energize DPDT relay - switches Curtis input from pedal to mux
    gpio_set_level(s_config.relay, 1);
    s_autonomous = true;

    // Wait for relay contacts to settle before allowing throttle changes
    vTaskDelay(pdMS_TO_TICKS(RELAY_SETTLE_MS));

    ESP_LOGI(TAG, "Autonomous mode ENABLED");
}

// Gracefully disable autonomous mode
// Sequence: ramp to 0 -> wait for Curtis -> de-energize relay -> disable mux
void throttle_mux_disable_autonomous(void) {
    if (!s_initialized) return;

    ESP_LOGI(TAG, "Disabling autonomous mode");

    // Ramp to idle before switching to prevent throttle discontinuity
    throttle_mux_set_level(0);
    vTaskDelay(pdMS_TO_TICKS(THROTTLE_REGISTER_MS));

    // Switch relay back to manual pedal
    gpio_set_level(s_config.relay, 0);
    s_autonomous = false;

    vTaskDelay(pdMS_TO_TICKS(20));
    throttle_mux_disable();

    ESP_LOGI(TAG, "Autonomous mode DISABLED (manual control active)");
}

// Immediate cutoff - no ramp down, used for override/fault conditions
void throttle_mux_emergency_stop(void) {
    if (!s_initialized) return;

    ESP_LOGW(TAG, "EMERGENCY STOP");

    // Immediately disable mux output
    gpio_set_level(s_config.en, 0);
    s_current_level = -1;

    // De-energize relay - NC contacts restore manual pedal control
    gpio_set_level(s_config.relay, 0);
    s_autonomous = false;
}

bool throttle_mux_is_autonomous(void) {
    return s_autonomous;
}
