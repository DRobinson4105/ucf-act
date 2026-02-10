/**
 * @file twai.h
 * @brief Mock TWAI driver types for host testing.
 */
#pragma once
#include "../esp_idf_mock.h"

typedef struct {
    uint32_t identifier;
    uint8_t  data_length_code;
    uint8_t  data[8];
    uint32_t extd : 1;       // 1 = extended 29-bit, 0 = standard 11-bit
    uint32_t rtr  : 1;       // 1 = RTR frame
    uint32_t ss   : 1;       // 1 = single-shot
    uint32_t self : 1;       // 1 = self-reception
    uint32_t dlc_non_comp : 1;
    uint32_t reserved : 27;
} twai_message_t;
