/**
 * @file FreeRTOS.h
 * @brief Mock FreeRTOS types, macros, and semaphore stubs for host testing.
 */
#pragma once
#include "../esp_idf_mock.h"

// Critical-section no-ops for host testing
#define portMUX_INITIALIZE(x) (void)(x)
#define taskENTER_CRITICAL(x) (void)(x)
#define taskEXIT_CRITICAL(x)  (void)(x)
