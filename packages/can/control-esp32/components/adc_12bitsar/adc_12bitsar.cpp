/**
 * @file adc_12bitsar.cpp
 * @brief Calibrated ESP32-C6 SAR ADC reader implementation.
 */

#include "adc_12bitsar.hh"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
namespace {

[[maybe_unused]] static const char *TAG = "ADC_12BITSAR";

static bool s_initialized = false;
static adc_12bitsar_config_t s_config = {};
static adc_oneshot_unit_handle_t s_adc_handle = nullptr;
static adc_cali_handle_t s_cali_handle = nullptr;
static bool s_cali_enabled = false;

}  // namespace

esp_err_t adc_12bitsar_init(const adc_12bitsar_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;

    adc_12bitsar_deinit();

    s_config = *config;

    adc_oneshot_unit_init_cfg_t adc_config = {
        .unit_id = config->adc_unit,
        .clk_src = ADC_DIGI_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    esp_err_t err = adc_oneshot_new_unit(&adc_config, &s_adc_handle);
    if (err != ESP_OK) return err;

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    err = adc_oneshot_config_channel(s_adc_handle, config->adc_channel, &chan_config);
    if (err != ESP_OK) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = nullptr;
        return err;
    }

    adc_cali_curve_fitting_config_t cali_config = {
        .unit_id = config->adc_unit,
        .chan = config->adc_channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

    err = adc_cali_create_scheme_curve_fitting(&cali_config, &s_cali_handle);
    s_cali_enabled = (err == ESP_OK);
    if (!s_cali_enabled) s_cali_handle = nullptr;

    s_initialized = true;
    return ESP_OK;
}

void adc_12bitsar_deinit(void) {
    if (s_cali_handle) {
        adc_cali_delete_scheme_curve_fitting(s_cali_handle);
        s_cali_handle = nullptr;
    }

    if (s_adc_handle) {
        adc_oneshot_del_unit(s_adc_handle);
        s_adc_handle = nullptr;
    }

    s_cali_enabled = false;
    s_initialized = false;
}

uint16_t adc_12bitsar_read_mv(void) {
    uint16_t mv = 0;
    if (adc_12bitsar_read_mv_checked(&mv) != ESP_OK) {
        return 0;
    }
    return mv;
}

esp_err_t adc_12bitsar_read_mv_checked(uint16_t *out_mv) {
    if (!out_mv) return ESP_ERR_INVALID_ARG;
    if (!s_initialized || !s_adc_handle) return ESP_ERR_INVALID_STATE;

    int raw = 0;
    esp_err_t err = adc_oneshot_read(s_adc_handle, s_config.adc_channel, &raw);
    if (err != ESP_OK) {
#ifdef CONFIG_LOG_INPUT_PEDAL_ADC
        ESP_LOGI(TAG, "ADC read failed: %s", esp_err_to_name(err));
#endif
        return err;
    }

    int voltage_mv = 0;
    if (s_cali_enabled && s_cali_handle) {
        adc_cali_raw_to_voltage(s_cali_handle, raw, &voltage_mv);
    } else {
        voltage_mv = (raw * 3300) / 4095;
    }

    *out_mv = (uint16_t)voltage_mv;
    return ESP_OK;
}

bool adc_12bitsar_is_calibrated(void) {
    return s_cali_enabled;
}
