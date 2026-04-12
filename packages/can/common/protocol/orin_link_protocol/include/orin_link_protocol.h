/**
 * @file orin_link_protocol.h
 * @brief Message typing helpers for the Orin <-> ESP UART links (transported via usb_serial_link).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "can_protocol.h"

#ifdef __cplusplus
extern "C"
{
#endif

#define ORIN_LINK_SYNC_BYTE 0xAA
#define ORIN_LINK_MAX_PAYLOAD_LEN 8

#define ORIN_LINK_MSG_PLANNER_COMMAND   0x01
#define ORIN_LINK_MSG_PLANNER_HEARTBEAT 0x02
#define ORIN_LINK_MSG_CONTROL_HEARTBEAT 0x03
#define ORIN_LINK_MSG_SAFETY_HEARTBEAT  0x04

const char *orin_link_message_type_to_string(uint8_t type);
uint8_t orin_link_expected_payload_len(uint8_t type);

bool orin_link_encode_planner_command_message(uint8_t *type_out, uint8_t *payload_out, uint8_t *payload_len_out,
                                              const planner_command_t *cmd);
bool orin_link_decode_planner_command_message(uint8_t type, const uint8_t *payload, uint8_t payload_len,
                                              planner_command_t *cmd);

bool orin_link_encode_planner_heartbeat_message(uint8_t *type_out, uint8_t *payload_out, uint8_t *payload_len_out,
                                                const node_heartbeat_t *hb);
bool orin_link_decode_planner_heartbeat_message(uint8_t type, const uint8_t *payload, uint8_t payload_len,
                                                node_heartbeat_t *hb);

bool orin_link_encode_control_heartbeat_message(uint8_t *type_out, uint8_t *payload_out, uint8_t *payload_len_out,
                                                const node_heartbeat_t *hb);
bool orin_link_encode_safety_heartbeat_message(uint8_t *type_out, uint8_t *payload_out, uint8_t *payload_len_out,
                                               const node_heartbeat_t *hb);

#ifdef __cplusplus
}
#endif
