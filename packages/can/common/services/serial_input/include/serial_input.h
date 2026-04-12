/**
 * @file serial_input.h
 * @brief Non-blocking serial input via USB Serial JTAG.
 *
 * Thin wrapper around the ESP-IDF USB Serial JTAG driver for single-
 * character non-blocking reads. Intended for diagnostic/test modes
 * (throttle test, motor jog, runtime bypass toggling, etc.) where
 * keystrokes typed in `idf.py monitor` drive commands.
 *
 * The ESP-IDF VFS/console layer already uses USB Serial JTAG for log
 * output (ESP_LOGx). This component installs the driver-level API on
 * top for reliable non-blocking input and is safe to call even when
 * the VFS console is active.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Install the USB Serial JTAG driver for non-blocking reads.
 *
 * Safe to call multiple times. Uses default TX/RX buffer sizes.
 *
 * @return ESP_OK on success (or already installed), error code on failure
 */
esp_err_t serial_input_init(void);

/**
 * @brief Non-blocking single-character read.
 *
 * Polls the USB Serial JTAG RX buffer for one byte with zero timeout.
 * Call this once per loop iteration (e.g. every 20 ms).
 *
 * @return The character read (0-255), or -1 if nothing available
 */
int serial_input_read_char(void);

typedef struct
{
	int32_t value; // -1 = no digits pending, >= 0 = accumulated value
	int32_t max;   // clamp ceiling
} serial_input_accum_t;

/**
 * @brief Static initializer for a serial input accumulator.
 * @param max_val  Maximum allowed value
 */
#define SERIAL_INPUT_ACCUM_INIT(max_val) {.value = -1, .max = (max_val)}

/**
 * @brief Feed a character into the accumulator.
 *
 * Digit characters are accumulated into the value, clamped to max.
 * Space, carriage return, or newline commits the accumulated value.
 * Any other character while digits are pending discards them.
 *
 * @param accum  Accumulator state
 * @param ch     Character from serial_input_read_char() (-1 = no input, ignored)
 * @return true if a value was committed (read accum->value, then call _reset)
 */
bool serial_input_accum_feed(serial_input_accum_t *accum, int ch);

/**
 * @brief Reset the accumulator, discarding any pending digits.
 * @param accum  Accumulator state
 */
void serial_input_accum_reset(serial_input_accum_t *accum);

#ifdef __cplusplus
}
#endif
