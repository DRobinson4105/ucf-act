/**
 * @file can_twai.hh (mock)
 * @brief Mock CAN TWAI driver that captures sent frames instead of transmitting.
 */
#pragma once

#include "esp_idf_mock.h"
#include "driver/twai.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline esp_err_t can_twai_init_default(gpio_num_t tx_gpio, gpio_num_t rx_gpio) {
    (void)tx_gpio; (void)rx_gpio;
    return ESP_OK;
}

static inline esp_err_t can_twai_send(uint32_t identifier, const uint8_t data[8], TickType_t timeout) {
    (void)timeout;
    if (mock_twai_send_result != ESP_OK) return mock_twai_send_result;
    if (mock_sent_count < MOCK_MAX_FRAMES) {
        mock_sent_frames[mock_sent_count].id = identifier;
        memcpy(mock_sent_frames[mock_sent_count].data, data, 8);
        mock_sent_frames[mock_sent_count].len = 8;
        mock_sent_count++;
    } else {
        fprintf(stderr, "MOCK WARNING: mock_sent_count >= MOCK_MAX_FRAMES, frame dropped!\n");
    }
    return ESP_OK;
}

static inline esp_err_t can_twai_send_extended(uint32_t identifier, const uint8_t *data,
                                                uint8_t dlc, TickType_t timeout) {
    (void)timeout;
    if (mock_twai_send_result != ESP_OK) return mock_twai_send_result;
    if (g_mock_send_ext_fail_after > 0) {
        g_mock_send_ext_fail_after--;
        if (g_mock_send_ext_fail_after == 0) return ESP_FAIL;
    }
    if (mock_sent_count < MOCK_MAX_FRAMES) {
        mock_sent_frames[mock_sent_count].id = identifier;
        memcpy(mock_sent_frames[mock_sent_count].data, data, dlc > 8 ? 8 : dlc);
        mock_sent_frames[mock_sent_count].len = dlc;
        mock_sent_count++;
    } else {
        fprintf(stderr, "MOCK WARNING: mock_sent_count >= MOCK_MAX_FRAMES, frame dropped!\n");
    }
    return ESP_OK;
}

static inline esp_err_t can_twai_receive(twai_message_t *msg, TickType_t timeout) {
    (void)msg; (void)timeout;
    return ESP_ERR_TIMEOUT;
}

static inline bool can_twai_bus_ok(void) {
    return true;
}

static inline esp_err_t can_twai_recover_bus_off(void) {
    return ESP_OK;
}

#ifdef __cplusplus
}
#endif
