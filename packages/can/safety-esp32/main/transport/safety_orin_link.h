/**
 * @file safety_orin_link.h
 * @brief Safety-side Orin UART link helpers.
 */
#pragma once

#include "freertos/FreeRTOS.h"
#include "heartbeat_monitor.h"
#include "safety_can_rx.h"

#ifdef __cplusplus
extern "C"
{
#endif

typedef struct
{
	const char *tag;
	heartbeat_monitor_t *monitor;
	int planner_node_handle;
	portMUX_TYPE *mirror_lock;
	volatile hb_mirror_snapshot_t *mirror;
} safety_orin_link_rx_context_t;

void safety_orin_link_rx_task(void *param);

#ifdef __cplusplus
}
#endif
