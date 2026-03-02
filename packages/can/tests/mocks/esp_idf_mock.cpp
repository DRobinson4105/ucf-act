/**
 * @file esp_idf_mock.c
 * @brief Global mock state variables for ESP-IDF/FreeRTOS mocks.
 */
#include "esp_idf_mock.h"

uint32_t     mock_tick_count       = 0;
int          mock_sem_take_result  = pdTRUE;
esp_err_t    mock_twai_send_result = ESP_OK;
mock_frame_t mock_sent_frames[MOCK_MAX_FRAMES];
int          mock_sent_count       = 0;
int          mock_sem_give_count   = 0;
int          mock_sem_take_count   = 0;
int          mock_sem_create_fail  = 0;
esp_err_t    mock_gpio_config_result = ESP_OK;
int          mock_gpio_levels[40] = {0};
esp_err_t    mock_adc_new_unit_result = ESP_OK;
esp_err_t    mock_adc_config_channel_result = ESP_OK;
esp_err_t    mock_adc_read_result = ESP_OK;
int          mock_adc_read_raw_value = 0;
esp_err_t    mock_adc_cali_create_result = ESP_FAIL;
esp_err_t    mock_adc_cali_raw_to_voltage_result = ESP_OK;
int          mock_adc_cali_voltage_mv = 0;
int          g_mock_send_ext_fail_after = 0;
mock_sem_take_callback_t g_mock_sem_take_callback = NULL;

void mock_reset_all(void) {
    mock_tick_count      = 0;
    mock_sem_take_result = pdTRUE;
    mock_twai_send_result = ESP_OK;
    mock_sent_count      = 0;
    mock_sem_give_count  = 0;
    mock_sem_take_count  = 0;
    mock_sem_create_fail = 0;
    mock_gpio_config_result = ESP_OK;
    memset(mock_gpio_levels, 0, sizeof(mock_gpio_levels));
    mock_adc_new_unit_result = ESP_OK;
    mock_adc_config_channel_result = ESP_OK;
    mock_adc_read_result = ESP_OK;
    mock_adc_read_raw_value = 0;
    mock_adc_cali_create_result = ESP_FAIL;
    mock_adc_cali_raw_to_voltage_result = ESP_OK;
    mock_adc_cali_voltage_mv = 0;
    g_mock_send_ext_fail_after = 0;
    g_mock_sem_take_callback = NULL;
    memset(mock_sent_frames, 0, sizeof(mock_sent_frames));
}

esp_err_t gpio_config(const gpio_config_t *cfg) {
    (void)cfg;
    return mock_gpio_config_result;
}

int gpio_get_level(gpio_num_t gpio) {
    if (gpio < 0 || gpio >= (int)(sizeof(mock_gpio_levels) / sizeof(mock_gpio_levels[0])))
        return 0;
    return mock_gpio_levels[gpio];
}

esp_err_t gpio_set_level(gpio_num_t gpio, int level) {
    if (gpio >= 0 && gpio < (int)(sizeof(mock_gpio_levels) / sizeof(mock_gpio_levels[0])))
        mock_gpio_levels[gpio] = level;
    return ESP_OK;
}

esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t *cfg,
                               adc_oneshot_unit_handle_t *ret_handle) {
    (void)cfg;
    if (!ret_handle) return ESP_ERR_INVALID_ARG;
    if (mock_adc_new_unit_result != ESP_OK) return mock_adc_new_unit_result;
    *ret_handle = (adc_oneshot_unit_handle_t)1;
    return ESP_OK;
}

esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle,
                                     adc_channel_t chan,
                                     const adc_oneshot_chan_cfg_t *cfg) {
    (void)handle;
    (void)chan;
    (void)cfg;
    return mock_adc_config_channel_result;
}

esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle,
                           adc_channel_t chan,
                           int *out_raw) {
    (void)handle;
    (void)chan;
    if (!out_raw) return ESP_ERR_INVALID_ARG;
    if (mock_adc_read_result != ESP_OK) return mock_adc_read_result;
    *out_raw = mock_adc_read_raw_value;
    return ESP_OK;
}

esp_err_t adc_oneshot_del_unit(adc_oneshot_unit_handle_t handle) {
    (void)handle;
    return ESP_OK;
}

esp_err_t adc_cali_create_scheme_curve_fitting(const adc_cali_curve_fitting_config_t *cfg,
                                               adc_cali_handle_t *ret_handle) {
    (void)cfg;
    if (!ret_handle) return ESP_ERR_INVALID_ARG;
    if (mock_adc_cali_create_result != ESP_OK) return mock_adc_cali_create_result;
    *ret_handle = (adc_cali_handle_t)1;
    return ESP_OK;
}

esp_err_t adc_cali_delete_scheme_curve_fitting(adc_cali_handle_t handle) {
    (void)handle;
    return ESP_OK;
}

esp_err_t adc_cali_raw_to_voltage(adc_cali_handle_t handle,
                                  int raw,
                                  int *voltage_mv) {
    (void)handle;
    (void)raw;
    if (!voltage_mv) return ESP_ERR_INVALID_ARG;
    if (mock_adc_cali_raw_to_voltage_result != ESP_OK)
        return mock_adc_cali_raw_to_voltage_result;
    *voltage_mv = mock_adc_cali_voltage_mv;
    return ESP_OK;
}
