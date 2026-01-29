#pragma once

#include "driver/twai.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gpio_num_t tx_gpio;
    gpio_num_t rx_gpio;
    uint32_t app_estop_id;
    uint32_t control_fault_id;
    uint32_t safety_estop_id;
} safety_can_config_t;

// initialize the TWAI driver for safety CAN
esp_err_t safety_can_init(const safety_can_config_t *config);

// receive a CAN frame with timeout
esp_err_t safety_can_receive(twai_message_t *msg, TickType_t timeout);

// updates app_estop/control_fault if a matching frame is seen
// returns true if a field was updated
bool safety_can_parse_frame(const safety_can_config_t *config,
                            const twai_message_t *msg,
                            bool *app_estop,
                            bool *control_fault);

// publish the safety E-stop state frame
esp_err_t safety_can_publish_estop(const safety_can_config_t *config, bool active, TickType_t timeout);

#ifdef __cplusplus
}
#endif
