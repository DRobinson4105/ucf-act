#include "safety_can.hh"

#include "can_twai.hh"

// =============================================================================
// Initialization
// =============================================================================

esp_err_t safety_can_init(const safety_can_config_t *config) {
    if (!config) return ESP_ERR_INVALID_ARG;
    return can_twai_init_default(config->tx_gpio, config->rx_gpio);
}

// =============================================================================
// Frame Processing
// =============================================================================

esp_err_t safety_can_receive(twai_message_t *msg, TickType_t timeout) {
    return can_twai_receive(msg, timeout);
}

// Parse incoming frames for app e-stop or control fault signals
bool safety_can_parse_frame(const safety_can_config_t *config,
                            const twai_message_t *msg,
                            bool *app_estop,
                            bool *control_fault) {
    if (!config || !msg || msg->extd || msg->rtr || msg->data_length_code < 1)
        return false;

    bool updated = false;
    if (msg->identifier == config->app_estop_id && app_estop) {
        *app_estop = (msg->data[0] != 0);
        updated = true;
    } else if (msg->identifier == config->control_fault_id && control_fault) {
        *control_fault = (msg->data[0] != 0);
        updated = true;
    }

    return updated;
}

// =============================================================================
// Publishing
// =============================================================================

// Broadcast e-stop state on CAN
esp_err_t safety_can_publish_estop(const safety_can_config_t *config, bool active, TickType_t timeout) {
    if (!config) return ESP_ERR_INVALID_ARG;

    uint8_t data[8] = {0};
    data[0] = active ? 1 : 0;
    return can_twai_send(config->safety_estop_id, data, timeout);
}
