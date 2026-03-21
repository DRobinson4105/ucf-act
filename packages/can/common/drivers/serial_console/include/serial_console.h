/**
 * @file serial_console.h
 * @brief Non-blocking serial input via USB Serial JTAG.
 *
 * Thin wrapper around the ESP-IDF USB Serial JTAG driver for single-
 * character non-blocking reads.  Intended for diagnostic/test modes
 * (throttle test, stepper jog, runtime bypass toggling, etc.) where
 * keystrokes typed in `idf.py monitor` drive commands.
 *
 * The ESP-IDF VFS/console layer already uses USB Serial JTAG for log
 * output (ESP_LOGx).  This component installs the driver-level API on
 * top for reliable non-blocking input — it is safe to call even when
 * the VFS console is active.
 */
#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * @brief Install the USB Serial JTAG driver for non-blocking reads.
 *
 * Safe to call multiple times — skips installation if the driver is
 * already installed.  Uses default TX/RX buffer sizes.
 *
 * @return ESP_OK on success (or already installed), error code on failure
 */
esp_err_t serial_console_init(void);

/**
 * @brief Non-blocking single-character read.
 *
 * Polls the USB Serial JTAG RX buffer for one byte with zero timeout.
 * Call this once per loop iteration (e.g. every 20 ms).
 *
 * @return The character read (0-255), or -1 if nothing available
 */
int serial_console_read_char(void);

#ifdef __cplusplus
}
#endif
