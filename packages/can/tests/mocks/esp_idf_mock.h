/**
 * @file esp_idf_mock.h
 * @brief Mock definitions for ESP-IDF and FreeRTOS types used in host-native tests.
 *
 * This header replaces all ESP-IDF and FreeRTOS dependencies so that
 * ESP32 component code can be compiled and tested with host gcc.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// ESP-IDF basic types
// ============================================================================

typedef int esp_err_t;
typedef int gpio_num_t;
typedef int adc_unit_t;
typedef int adc_channel_t;

#define ESP_OK                 0
#define ESP_FAIL              -1
#define ESP_ERR_NO_MEM         0x101
#define ESP_ERR_INVALID_ARG    0x102
#define ESP_ERR_INVALID_STATE  0x103
#define ESP_ERR_TIMEOUT        0x107

#define ADC_UNIT_1 0
#define ADC_CHANNEL_0 0
#define ADC_DIGI_CLK_SRC_DEFAULT 0
#define ADC_ULP_MODE_DISABLE 0
#define ADC_ATTEN_DB_12 0
#define ADC_BITWIDTH_12 12

#define GPIO_MODE_INPUT 0
#define GPIO_MODE_OUTPUT 1
#define GPIO_PULLUP_DISABLE 0
#define GPIO_PULLUP_ENABLE 1
#define GPIO_PULLDOWN_DISABLE 0
#define GPIO_PULLDOWN_ENABLE 1
#define GPIO_INTR_DISABLE 0

static inline const char *esp_err_to_name(esp_err_t err) {
    switch (err) {
        case ESP_OK:                return "ESP_OK";
        case ESP_ERR_TIMEOUT:       return "ESP_ERR_TIMEOUT";
        case ESP_ERR_INVALID_ARG:   return "ESP_ERR_INVALID_ARG";
        case ESP_ERR_INVALID_STATE: return "ESP_ERR_INVALID_STATE";
        case ESP_ERR_NO_MEM:        return "ESP_ERR_NO_MEM";
        default:                    return "UNKNOWN";
    }
}

// ============================================================================
// FreeRTOS types
// ============================================================================

typedef uint32_t TickType_t;
typedef void *SemaphoreHandle_t;
typedef int BaseType_t;
typedef void *adc_oneshot_unit_handle_t;
typedef void *adc_cali_handle_t;

#define pdTRUE   1
#define pdFALSE  0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))
#define portTICK_PERIOD_MS 1

typedef struct { int _dummy; } portMUX_TYPE;
#define portMUX_INITIALIZER_UNLOCKED { 0 }

// ============================================================================
// Mock control state (defined in esp_idf_mock.c)
// ============================================================================

#define MOCK_MAX_FRAMES 64

typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t len;
} mock_frame_t;

// Global mock state — test code controls these
extern uint32_t      mock_tick_count;
extern int           mock_sem_take_result;   // pdTRUE or pdFALSE
extern esp_err_t     mock_twai_send_result;
extern mock_frame_t  mock_sent_frames[MOCK_MAX_FRAMES];
extern int           mock_sent_count;
extern int           mock_sem_give_count;
extern int           mock_sem_take_count;
extern int           mock_sem_create_fail;   // if nonzero, xSemaphoreCreateBinary returns NULL
extern esp_err_t     mock_gpio_config_result;
extern int           mock_gpio_levels[40];
extern esp_err_t     mock_adc_new_unit_result;
extern esp_err_t     mock_adc_config_channel_result;
extern esp_err_t     mock_adc_read_result;
extern int           mock_adc_read_raw_value;
extern esp_err_t     mock_adc_cali_create_result;
extern esp_err_t     mock_adc_cali_raw_to_voltage_result;
extern int           mock_adc_cali_voltage_mv;

// Mock control: fail the Nth call to can_twai_send_extended (0 = don't fail)
extern int g_mock_send_ext_fail_after;

// Optional callback invoked on each xSemaphoreTake (useful for setting query_result at the right time)
typedef void (*mock_sem_take_callback_t)(int take_count);
extern mock_sem_take_callback_t g_mock_sem_take_callback;

void mock_reset_all(void);

// ============================================================================
// GPIO mock types/functions
// ============================================================================

typedef struct {
    uint64_t pin_bit_mask;
    int mode;
    int pull_up_en;
    int pull_down_en;
    int intr_type;
} gpio_config_t;

esp_err_t gpio_config(const gpio_config_t *cfg);
int gpio_get_level(gpio_num_t gpio);
esp_err_t gpio_set_level(gpio_num_t gpio, int level);

// ============================================================================
// ADC oneshot mock types/functions
// ============================================================================

typedef struct {
    adc_unit_t unit_id;
    int clk_src;
    int ulp_mode;
} adc_oneshot_unit_init_cfg_t;

typedef struct {
    int atten;
    int bitwidth;
} adc_oneshot_chan_cfg_t;

esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t *cfg,
                               adc_oneshot_unit_handle_t *ret_handle);
esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle,
                                     adc_channel_t chan,
                                     const adc_oneshot_chan_cfg_t *cfg);
esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle,
                           adc_channel_t chan,
                           int *out_raw);
esp_err_t adc_oneshot_del_unit(adc_oneshot_unit_handle_t handle);

// ============================================================================
// ADC calibration mock types/functions
// ============================================================================

typedef struct {
    adc_unit_t unit_id;
    adc_channel_t chan;
    int atten;
    int bitwidth;
} adc_cali_curve_fitting_config_t;

esp_err_t adc_cali_create_scheme_curve_fitting(const adc_cali_curve_fitting_config_t *cfg,
                                               adc_cali_handle_t *ret_handle);
esp_err_t adc_cali_delete_scheme_curve_fitting(adc_cali_handle_t handle);
esp_err_t adc_cali_raw_to_voltage(adc_cali_handle_t handle,
                                  int raw,
                                  int *voltage_mv);

// ============================================================================
// FreeRTOS function mocks
// ============================================================================

static inline TickType_t xTaskGetTickCount(void) {
    return mock_tick_count;
}

static inline SemaphoreHandle_t xSemaphoreCreateBinary(void) {
    if (mock_sem_create_fail) return NULL;
    return (SemaphoreHandle_t)1;  // non-NULL dummy
}

static inline BaseType_t xSemaphoreTake(SemaphoreHandle_t sem, TickType_t timeout) {
    (void)sem; (void)timeout;
    mock_sem_take_count++;
    if (g_mock_sem_take_callback) g_mock_sem_take_callback(mock_sem_take_count);
    return mock_sem_take_result;
}

static inline void xSemaphoreGive(SemaphoreHandle_t sem) {
    (void)sem;
    mock_sem_give_count++;
}

static inline void vTaskDelay(TickType_t ticks) {
    (void)ticks;
}

// ============================================================================
// Logging — no-op
// ============================================================================

#define ESP_LOGI(tag, fmt, ...) (void)0
#define ESP_LOGW(tag, fmt, ...) (void)0
#define ESP_LOGE(tag, fmt, ...) (void)0
#define ESP_LOGD(tag, fmt, ...) (void)0

#ifdef __cplusplus
}
#endif
