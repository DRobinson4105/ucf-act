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
    g_mock_send_ext_fail_after = 0;
    g_mock_sem_take_callback = NULL;
    memset(mock_sent_frames, 0, sizeof(mock_sent_frames));
}
