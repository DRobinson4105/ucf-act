#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/twai.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    TWAI_PORT_STATE_UNINITIALIZED = 0,
    TWAI_PORT_STATE_STOPPED,
    TWAI_PORT_STATE_RUNNING,
    TWAI_PORT_STATE_QUIESCED,
    TWAI_PORT_STATE_RECOVERING,
} twai_port_state_t;

typedef enum {
    TWAI_PORT_MODE_NORMAL = 0,
    TWAI_PORT_MODE_LISTEN_ONLY,
    TWAI_PORT_MODE_NO_ACK,
} twai_port_mode_t;

typedef enum {
    TWAI_PORT_FILTER_ACCEPT_ALL = 0,
    TWAI_PORT_FILTER_STD_ONLY,
    TWAI_PORT_FILTER_EXT_ONLY,
} twai_port_filter_mode_t;

typedef struct {
    int tx_gpio;
    int rx_gpio;

    uint32_t bitrate;
    uint32_t alerts_enabled;

    uint16_t tx_queue_len;
    uint16_t rx_queue_len;

    twai_port_mode_t mode;

    bool log_tx;
    bool log_rx;
    bool log_alerts;
    bool log_ext_only;

    TickType_t default_timeout_ticks;
    twai_port_filter_mode_t filter_mode;
} twai_port_cfg_t;

typedef struct {
    twai_port_state_t state;
    bool initialized;
    bool running;
    bool quiesced;
    bool recovery_in_progress;
} twai_port_runtime_info_t;

esp_err_t twai_port_init(const twai_port_cfg_t *cfg);
esp_err_t twai_port_start(void);
esp_err_t twai_port_stop(void);
esp_err_t twai_port_deinit(void);

twai_port_state_t twai_port_get_state(void);
bool twai_port_is_initialized(void);
bool twai_port_is_running(void);
bool twai_port_is_quiesced(void);
esp_err_t twai_port_get_runtime_info(twai_port_runtime_info_t *out_info);
esp_err_t twai_port_get_cfg(twai_port_cfg_t *out_cfg);

esp_err_t twai_port_send(const twai_message_t *msg, TickType_t timeout_ticks);
esp_err_t twai_port_receive(twai_message_t *out_msg, TickType_t timeout_ticks);

esp_err_t twai_port_send_std(
        uint16_t std_id,
        const uint8_t *data,
        uint8_t dlc,
        TickType_t timeout_ticks
    );

esp_err_t twai_port_send_ext(
        uint32_t ext_id,
        const uint8_t *data,
        uint8_t dlc,
        TickType_t timeout_ticks
    );

esp_err_t twai_port_read_alerts(uint32_t *out_alerts, TickType_t timeout_ticks);
esp_err_t twai_port_get_status(twai_status_info_t *out_status);
bool twai_port_is_bus_off(void);

esp_err_t twai_port_quiesce(TickType_t timeout_ticks);
esp_err_t twai_port_resume(void);

esp_err_t twai_port_reconfigure_blocking(const twai_port_cfg_t *new_cfg, TickType_t timeout_ticks);

esp_err_t twai_port_set_bitrate_blocking(uint32_t bitrate, TickType_t timeout_ticks);

/*
 * Internal test hook surface for Unity-based component tests.
 * Production code should not depend on these APIs.
 */
typedef struct {
    esp_err_t (*driver_install)(const twai_general_config_t *g_config,
                                const twai_timing_config_t *t_config,
                                const twai_filter_config_t *f_config);
    esp_err_t (*driver_uninstall)(void);
    esp_err_t (*start)(void);
    esp_err_t (*stop)(void);
    esp_err_t (*transmit)(twai_message_t *message, TickType_t ticks_to_wait);
    esp_err_t (*receive)(twai_message_t *message, TickType_t ticks_to_wait);
    esp_err_t (*read_alerts)(uint32_t *alerts, TickType_t ticks_to_wait);
    esp_err_t (*get_status_info)(twai_status_info_t *status_info);
} twai_port_driver_ops_t;

void twai_port_test_set_driver_ops(const twai_port_driver_ops_t *ops);
void twai_port_test_reset_state(void);
#ifdef __cplusplus
}
#endif
