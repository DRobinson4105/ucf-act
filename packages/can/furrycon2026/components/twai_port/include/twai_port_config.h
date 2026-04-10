#pragma once

/*
 * Single config entry point for the twai_port component.
 *
 * Workflow:
 *   - Use `idf.py menuconfig` to change settings.
 *   - Project-wide defaults live in sdkconfig.defaults — do not hand-edit sdkconfig.
 *   - All consumer code should use TWAI_PORT_* names, not raw CONFIG_APP_TWAI_* names.
 *   - No-ACK mode is for single-node bring-up only. Normal mode requires another
 *     node on the bus to acknowledge frames.
 */

#include "sdkconfig.h"
#include "twai_port.h"

/* GPIO */
#define TWAI_PORT_TX_GPIO           CONFIG_APP_TWAI_TX_GPIO
#define TWAI_PORT_RX_GPIO           CONFIG_APP_TWAI_RX_GPIO

/* Bitrate (Hz) */
#define TWAI_PORT_BITRATE           CONFIG_APP_TWAI_BITRATE

/* Queue lengths */
#define TWAI_PORT_TX_QUEUE_LEN      CONFIG_APP_TWAI_TX_QUEUE_LEN
#define TWAI_PORT_RX_QUEUE_LEN      CONFIG_APP_TWAI_RX_QUEUE_LEN

/* Timeout */
#define TWAI_PORT_TIMEOUT_MS        CONFIG_APP_TWAI_DEFAULT_TIMEOUT_MS

/* Operating mode — derived from the Kconfig choice */
#if defined(CONFIG_APP_TWAI_MODE_LISTEN_ONLY)
#  define TWAI_PORT_MODE_DEFAULT    TWAI_PORT_MODE_LISTEN_ONLY
#elif defined(CONFIG_APP_TWAI_MODE_NO_ACK)
#  define TWAI_PORT_MODE_DEFAULT    TWAI_PORT_MODE_NO_ACK
#else
#  define TWAI_PORT_MODE_DEFAULT    TWAI_PORT_MODE_NORMAL
#endif

/* Logging — guarded because ESP-IDF omits disabled bool symbols entirely. */
#ifdef CONFIG_APP_TWAI_LOG_TX
#  define TWAI_PORT_LOG_TX      1
#else
#  define TWAI_PORT_LOG_TX      0
#endif

#ifdef CONFIG_APP_TWAI_LOG_RX
#  define TWAI_PORT_LOG_RX      1
#else
#  define TWAI_PORT_LOG_RX      0
#endif

#ifdef CONFIG_APP_TWAI_LOG_ALERTS
#  define TWAI_PORT_LOG_ALERTS  1
#else
#  define TWAI_PORT_LOG_ALERTS  0
#endif

#ifdef CONFIG_APP_TWAI_LOG_EXT_ONLY
#  define TWAI_PORT_LOG_EXT_ONLY 1
#else
#  define TWAI_PORT_LOG_EXT_ONLY 0
#endif

/*
 * Default alert mask.
 * Covers the conditions worth waking up for in normal operation:
 * bus-off, recovery, error-passive, TX failure, and RX queue full.
 */
#define TWAI_PORT_ALERTS_DEFAULT    (TWAI_ALERT_BUS_OFF        | \
                                     TWAI_ALERT_BUS_RECOVERED  | \
                                     TWAI_ALERT_ERR_PASS       | \
                                     TWAI_ALERT_TX_FAILED      | \
                                     TWAI_ALERT_RX_QUEUE_FULL)
