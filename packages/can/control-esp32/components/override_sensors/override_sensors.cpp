/**
 * @file override_sensors.cpp
 * @brief Human override sensor implementation with debouncing and re-arm logic.
 */
#include "override_sensors.hh"

#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

namespace {

static const char *TAG = "OVERRIDE_SENSORS";

// ============================================================================
// Module State
// ============================================================================

static override_sensors_config_t s_config = {};
static bool s_initialized = false;

// ADC handles for pedal voltage reading
static adc_oneshot_unit_handle_t s_adc_handle = nullptr;
static adc_cali_handle_t s_cali_handle = nullptr;
static bool s_cali_enabled = false;

// F/R switch debounce state
static fr_state_t s_fr_state_debounced = FR_STATE_NEUTRAL;
static fr_state_t s_fr_state_pending = FR_STATE_NEUTRAL;
static uint32_t s_fr_change_time = 0;
static bool s_fr_debouncing = false;

// Pedal re-arm tracking - prevents immediate re-enable after override
static uint32_t s_pedal_below_threshold_since = 0;
static bool s_pedal_was_above = false;

// ============================================================================
// Internal Helpers - F/R Switch Reading
// ============================================================================

// Read raw F/R state from optocouplers (no debounce)
// Optocoupler logic: switch closed -> LED on -> phototransistor conducts -> GPIO LOW
//
// Truth table:
//   dir=LOW  rev=HIGH -> Forward (direction enabled, not reverse)
//   dir=LOW  rev=LOW  -> Reverse (direction enabled, reverse active)
//   dir=HIGH rev=HIGH -> Neutral (direction not enabled)
//   dir=HIGH rev=LOW  -> Invalid (reverse without direction - wiring fault)
static fr_state_t read_fr_raw() {
    bool dir_active = (gpio_get_level(s_config.fr_dir_gpio) == 0);
    bool rev_active = (gpio_get_level(s_config.fr_rev_gpio) == 0);

    if (dir_active && !rev_active) return FR_STATE_FORWARD;
    else if (dir_active && rev_active) return FR_STATE_REVERSE;
    else if (!dir_active && !rev_active) return FR_STATE_NEUTRAL;
    else return FR_STATE_INVALID;
}

// ============================================================================
// Internal Helpers - Pedal ADC
// ============================================================================

// Read pedal voltage via ADC, return millivolts
static uint16_t read_pedal_adc_mv() {
    if (!s_initialized || s_adc_handle == nullptr) return 0;

    int raw = 0;
    esp_err_t err = adc_oneshot_read(s_adc_handle, s_config.pedal_adc_channel, &raw);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed: %s", esp_err_to_name(err));
        return 0;
    }

    int voltage_mv = 0;
    if (s_cali_enabled && s_cali_handle != nullptr)
		adc_cali_raw_to_voltage(s_cali_handle, raw, &voltage_mv);
    else
        // Fallback: 12-bit ADC, 3.3V reference
        voltage_mv = (raw * 3300) / 4095;

    return (uint16_t)voltage_mv;
}

}  // namespace

// ============================================================================
// Initialization
// ============================================================================

esp_err_t override_sensors_init(const override_sensors_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    s_config = *config;

    // Initialize ADC for pedal voltage sensing
    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = config->adc_unit,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t err = adc_oneshot_new_unit(&adc_config, &s_adc_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC unit init failed: %s", esp_err_to_name(err));
        return err;
    }

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,     // Full 0-3.3V range
        .bitwidth = ADC_BITWIDTH_12,
    };

    err = adc_oneshot_config_channel(s_adc_handle, config->pedal_adc_channel, &chan_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ADC channel config failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = nullptr;
        return err;
    }

    // Try ADC calibration - improves accuracy but may not be available
    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = config->adc_unit,
        .chan = config->pedal_adc_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    err = adc_cali_create_scheme_curve_fitting(&cali_config, &s_cali_handle);
    if (err == ESP_OK) {
        s_cali_enabled = true;
        ESP_LOGI(TAG, "ADC calibration enabled (curve fitting)");
    } else {
        s_cali_enabled = false;
        ESP_LOGW(TAG, "ADC calibration not available, using raw conversion");
    }

    // Configure F/R optocoupler inputs with pull-ups
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << config->fr_dir_gpio) |
            			(1ULL << config->fr_rev_gpio),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };

    err = gpio_config(&io_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "GPIO config failed: %s", esp_err_to_name(err));
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = nullptr;
        return err;
    }

    // Initialize debounce state to current switch position
    s_fr_state_debounced = read_fr_raw();
    s_fr_state_pending = s_fr_state_debounced;
    s_fr_debouncing = false;

    // Initialize pedal re-arm tracking
    uint16_t initial_mv = read_pedal_adc_mv();
    s_pedal_was_above = (initial_mv >= PEDAL_ADC_THRESHOLD_MV);
    s_pedal_below_threshold_since = 0;

    s_initialized = true;

    ESP_LOGI(TAG, "Initialized: pedal_adc=ADC%d_CH%d, fr_dir=%d, fr_rev=%d",
             config->adc_unit + 1, config->pedal_adc_channel,
             config->fr_dir_gpio, config->fr_rev_gpio);
    ESP_LOGI(TAG, "Initial pedal: %u mV, F/R state: %d", initial_mv, s_fr_state_debounced);

    return ESP_OK;
}

// ============================================================================
// Pedal Detection
// ============================================================================

// Immediate pedal check - true if voltage above threshold (pressed)
bool override_sensors_pedal_pressed(void) {
    if (!s_initialized) return false;
    return (read_pedal_adc_mv() >= PEDAL_ADC_THRESHOLD_MV);
}

// Check if pedal has been released long enough to re-arm (500ms)
// Used to prevent accidental re-enable when driver just lifted foot
bool override_sensors_pedal_rearmed(void) {
    if (!s_initialized) return true;

    if (read_pedal_adc_mv() >= PEDAL_ADC_THRESHOLD_MV) return false;  // Currently pressed

    // Pedal below threshold - check if held long enough
    return !s_pedal_was_above;  // Re-armed when was_above cleared by update()
}

uint16_t override_sensors_get_pedal_mv(void) {
    if (!s_initialized) return 0;
    return read_pedal_adc_mv();
}

// ============================================================================
// F/R Switch Detection
// ============================================================================

fr_state_t override_sensors_get_fr_state(void) {
    if (!s_initialized) return FR_STATE_NEUTRAL;
    return s_fr_state_debounced;
}

fr_state_t override_sensors_get_fr_state_raw(void) {
    if (!s_initialized) return FR_STATE_NEUTRAL;
    return read_fr_raw();
}

// ============================================================================
// Runtime Update
// ============================================================================

// Call every 10-20ms to update debounce and re-arm state
void override_sensors_update(uint32_t now_ms) {
    if (!s_initialized) return;

    // --- Pedal re-arm tracking ---
    // Once pedal pressed, must be released for PEDAL_REARM_MS before autonomous can resume
    uint16_t pedal_mv = read_pedal_adc_mv();
    if (pedal_mv >= PEDAL_ADC_THRESHOLD_MV) {
        s_pedal_was_above = true;
        s_pedal_below_threshold_since = 0;
    } else {
        if (s_pedal_was_above)
            // Just transitioned from pressed to released - start timer
            s_pedal_below_threshold_since = now_ms;

        // Check if held below threshold long enough
        if (s_pedal_below_threshold_since > 0 &&
            (now_ms - s_pedal_below_threshold_since) >= PEDAL_REARM_MS) {
            s_pedal_was_above = false;
            ESP_LOGD(TAG, "Pedal re-armed");
        }
    }

    // --- F/R switch debounce ---
    // Mechanical switch bounces during transitions - wait for stable state
    fr_state_t current_raw = read_fr_raw();

    if (current_raw != s_fr_state_debounced) {
        if (!s_fr_debouncing) {
            // Start debounce timer on first change detection
            s_fr_state_pending = current_raw;
            s_fr_change_time = now_ms;
            s_fr_debouncing = true;
            ESP_LOGD(TAG, "F/R change detected, starting debounce: %d -> %d",
                     s_fr_state_debounced, current_raw);
        } else if (current_raw != s_fr_state_pending) {
            // State changed again during debounce - reset timer with new target
            s_fr_state_pending = current_raw;
            s_fr_change_time = now_ms;
            ESP_LOGD(TAG, "F/R changed during debounce, resetting: -> %d", current_raw);
        } else if ((now_ms - s_fr_change_time) >= FR_DEBOUNCE_MS) {
            // Held stable for debounce period - accept new state
            ESP_LOGI(TAG, "F/R state changed: %d -> %d",
                     s_fr_state_debounced, s_fr_state_pending);
            s_fr_state_debounced = s_fr_state_pending;
            s_fr_debouncing = false;
        }
    } else {
        // Raw matches debounced - cancel pending debounce
        if (s_fr_debouncing)
			ESP_LOGD(TAG, "F/R returned to stable state, canceling debounce");
        s_fr_debouncing = false;
    }
}

// ============================================================================
// Status Flags
// ============================================================================

// Get raw sensor flags for CAN status message
uint8_t override_sensors_get_flags(void) {
    if (!s_initialized) return 0;

    uint8_t flags = 0;

    if (override_sensors_pedal_pressed()) flags |= SENSOR_FLAG_PEDAL_PRESSED;
    if (gpio_get_level(s_config.fr_dir_gpio) == 0) flags |= SENSOR_FLAG_FR_FORWARD;
    if (gpio_get_level(s_config.fr_rev_gpio) == 0) flags |= SENSOR_FLAG_FR_REVERSE;

    return flags;
}
