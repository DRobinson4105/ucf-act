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

#define ESP_OK                 0
#define ESP_FAIL              -1
#define ESP_ERR_NO_MEM         0x101
#define ESP_ERR_INVALID_ARG    0x102
#define ESP_ERR_INVALID_STATE  0x103
#define ESP_ERR_TIMEOUT        0x107

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

#define pdTRUE   1
#define pdFALSE  0
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms))

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

void mock_reset_all(void);

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
