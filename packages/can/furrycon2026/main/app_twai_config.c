#include "app_twai_config.h"
#include "twai_port_config.h"
#include "freertos/FreeRTOS.h"

twai_port_cfg_t app_make_twai_port_cfg(void)
{
    twai_port_cfg_t cfg = {
        .tx_gpio              = TWAI_PORT_TX_GPIO,
        .rx_gpio              = TWAI_PORT_RX_GPIO,
        .bitrate              = TWAI_PORT_BITRATE,
        .alerts_enabled       = TWAI_PORT_ALERTS_DEFAULT,
        .tx_queue_len         = TWAI_PORT_TX_QUEUE_LEN,
        .rx_queue_len         = TWAI_PORT_RX_QUEUE_LEN,
        .mode                 = TWAI_PORT_MODE_DEFAULT,
        .log_tx               = TWAI_PORT_LOG_TX,
        .log_rx               = TWAI_PORT_LOG_RX,
        .log_alerts           = TWAI_PORT_LOG_ALERTS,
        .log_ext_only         = TWAI_PORT_LOG_EXT_ONLY,
        .default_timeout_ticks = pdMS_TO_TICKS(TWAI_PORT_TIMEOUT_MS),
        .filter_mode          = TWAI_PORT_FILTER_ACCEPT_ALL,
    };
    return cfg;
}
