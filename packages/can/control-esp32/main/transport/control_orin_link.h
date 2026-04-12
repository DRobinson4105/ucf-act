/**
 * @file control_orin_link.h
 * @brief Control-side Orin UART link helpers.
 */
#pragma once

#include "control_can_rx.h"
#include "freertos/FreeRTOS.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	const char *tag;
	portMUX_TYPE *cmd_lock;
	volatile command_snapshot_t *snapshot;
} control_orin_link_rx_context_t;

void control_orin_link_rx_task(void *param);

#ifdef __cplusplus
}
#endif
