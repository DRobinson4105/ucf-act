/**
 * @file can_twai.cpp
 * @brief CAN bus TWAI driver wrapper implementation.
 */
#include "can_twai.hh"

// ============================================================================
// Initialization
// ============================================================================

// Configure TWAI peripheral: 1 Mbps, normal mode, accept all frames
esp_err_t can_twai_init_default(gpio_num_t tx_gpio, gpio_num_t rx_gpio) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_gpio, rx_gpio, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK) return err;
    
    return twai_start();
}

// ============================================================================
// Transmit
// ============================================================================

// Send standard 11-bit CAN frame with fixed 8-byte payload
esp_err_t can_twai_send(uint32_t identifier, const uint8_t data[8], TickType_t timeout) {
    twai_message_t msg = {};
    msg.identifier = identifier;
    msg.extd = 0;
    msg.rtr = 0;
    msg.ss = 0;
    msg.self = 0;
    msg.dlc_non_comp = 0;
    msg.data_length_code = 8;
    for (int i = 0; i < 8; ++i) msg.data[i] = data[i];
    
    return twai_transmit(&msg, timeout);
}

// Send extended 29-bit CAN frame with variable payload (for UIM2852CA motors)
esp_err_t can_twai_send_extended(uint32_t identifier, const uint8_t *data, uint8_t dlc, TickType_t timeout) {
    twai_message_t msg = {};
    msg.identifier = identifier;
    msg.extd = 1;
    msg.rtr = 0;
    msg.ss = 0;
    msg.self = 0;
    msg.dlc_non_comp = 0;
    msg.data_length_code = (dlc > 8) ? 8 : dlc;
    for (int i = 0; i < msg.data_length_code; ++i) {
        msg.data[i] = data ? data[i] : 0;
    }
    
    return twai_transmit(&msg, timeout);
}

// ============================================================================
// Receive
// ============================================================================

// Receive next CAN frame (blocking with timeout)
esp_err_t can_twai_receive(twai_message_t *msg, TickType_t timeout) {
    return twai_receive(msg, timeout);
}
