/**
 * @file control_startup_log.h
 * @brief Startup device status logging helpers for Control ESP32.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "adc_12bitsar.h"
#include "dac_mcp4728.h"
#include "driver/gpio.h"
#include "relay_dpdt_my5nj.h"

void control_log_startup_device_status(const char *tag, bool twai_ready, gpio_num_t twai_tx_gpio, gpio_num_t twai_rx_gpio,
                                       const dac_mcp4728_config_t *dac_cfg, bool dac_ready,
                                       const relay_dpdt_my5nj_config_t *relay_cfg, bool relay_ready,
                                       const adc_12bitsar_config_t *pedal_cfg, unsigned pedal_threshold_mv,
                                       unsigned initial_pedal_mv, bool pedal_ready, bool pedal_calibrated,
                                       gpio_num_t fr_forward_gpio, gpio_num_t fr_reverse_gpio, bool fr_ready,
                                       unsigned steering_node_id, bool steering_ready, unsigned braking_node_id,
                                       bool braking_ready, bool heartbeat_ready, bool orin_link_ready);
