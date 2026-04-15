#pragma once

#include <stdint.h>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"

/* Wire format: [0xAA, seq, thr_hi, thr_lo, steer_hi, steer_lo, brake] */
#define SERIAL_INPUT_SYNC      0xAAU
#define SERIAL_INPUT_FRAME_LEN 7U

typedef struct {
    uint8_t  seq;
    uint16_t throttle;
    uint16_t steering;
    uint8_t  braking;
} serial_input_frame_t;

/**
 * Configure the dedicated planner UART receiver. Call once before
 * serial_input_read().
 */
esp_err_t serial_input_init(void);

/**
 * Block until a valid frame is received or the timeout expires.
 *
 * Scans the byte stream for a SYNC byte (0xAA), then reads the remaining
 * six payload bytes and fills @p out.
 *
 * @param out            Destination for the parsed frame.
 * @param timeout_ticks  Maximum time to wait expressed in FreeRTOS ticks.
 *
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if no complete frame arrived
 *         within @p timeout_ticks, or ESP_ERR_INVALID_ARG if @p out is NULL.
 */
esp_err_t serial_input_read(serial_input_frame_t *out, TickType_t timeout_ticks);
