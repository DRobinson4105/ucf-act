/**
 * @file safety_loop.h
 * @brief FreeRTOS task entry points for the Safety ESP32 runtime.
 */
#pragma once

void can_rx_task(void *param);
void safety_task(void *param);
void heartbeat_task(void *param);
