/**
 * @file twai.h
 * @brief Mock TWAI driver types for host testing.
 */
#pragma once
#include "../esp_idf_mock.h"

typedef struct {
    uint32_t extd : 1;       // 1 = extended 29-bit, 0 = standard 11-bit
    uint32_t rtr  : 1;       // 1 = RTR frame
    uint32_t ss   : 1;       // 1 = single-shot
    uint32_t self : 1;       // 1 = self-reception
    uint32_t dlc_non_comp : 1;
    uint32_t reserved : 27;
    uint32_t identifier;
    uint8_t  data_length_code;
    uint8_t  data[8];
} twai_message_t;

// Status info mock (for can_twai_recover_bus_off polling)
#define TWAI_STATE_RUNNING  1
#define TWAI_STATE_STOPPED  0

typedef struct {
    uint32_t state;
} twai_status_info_t;

static inline esp_err_t twai_get_status_info(twai_status_info_t *status) {
    if (status) status->state = TWAI_STATE_RUNNING;
    return ESP_OK;
}
static inline esp_err_t twai_driver_install(void *g, void *t, void *f) { (void)g; (void)t; (void)f; return ESP_OK; }
static inline esp_err_t twai_driver_uninstall(void) { return ESP_OK; }
static inline esp_err_t twai_start(void) { return ESP_OK; }
static inline esp_err_t twai_stop(void) { return ESP_OK; }
static inline esp_err_t twai_transmit(void *msg, uint32_t timeout) { (void)msg; (void)timeout; return ESP_OK; }
static inline esp_err_t twai_receive(void *msg, uint32_t timeout) { (void)msg; (void)timeout; return ESP_ERR_TIMEOUT; }
static inline esp_err_t twai_initiate_recovery(void) { return ESP_OK; }
