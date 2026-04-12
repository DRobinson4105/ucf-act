/**
 * @file control_loop.h
 * @brief FreeRTOS task entry points for the Control ESP32 runtime.
 */
#pragma once

void can_rx_task(void *param);
void control_task(void *param);
void heartbeat_task(void *param);
