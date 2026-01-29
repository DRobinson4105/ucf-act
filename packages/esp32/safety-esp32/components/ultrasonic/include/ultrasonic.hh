// ultrasonic UART sensor interface
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uart_port_t uart_num;
	int tx_gpio;
	int rx_gpio;
	int baud_rate;
} ultrasonic_config_t;

// initialize ultrasonic UART reader
esp_err_t ultrasonic_init(const ultrasonic_config_t *config);

// return the latest distance in mm if a fresh sample is available
bool ultrasonic_get_distance_mm(uint16_t *out_distance_mm);

// return true when the latest distance is <= threshold
bool ultrasonic_is_too_close(uint16_t threshold_mm, uint16_t *out_distance_mm);

#ifdef __cplusplus
}
#endif