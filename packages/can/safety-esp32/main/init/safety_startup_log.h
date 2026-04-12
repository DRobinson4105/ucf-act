/**
 * @file safety_startup_log.h
 * @brief Startup device status logging helpers for Safety ESP32.
 */
#pragma once

#include <stdbool.h>

#include "driver/gpio.h"
#include "relay_srd05vdc.h"
#include "ultrasonic_a02yyuw.h"

void safety_log_startup_device_status(const char *tag, bool twai_ready, gpio_num_t twai_tx_gpio, gpio_num_t twai_rx_gpio,
                                      int push_button_gpio, int push_button_active_level, bool push_button_ready,
                                      int rf_remote_gpio, int rf_remote_active_level, bool rf_remote_ready,
                                      const ultrasonic_a02yyuw_config_t *ultrasonic_cfg, bool ultrasonic_ready,
                                      const relay_srd05vdc_config_t *relay_cfg, bool relay_ready, int battery_voltage_gpio,
                                      int battery_current_gpio, bool battery_monitor_ready, bool heartbeat_ready,
                                      bool orin_link_ready);
