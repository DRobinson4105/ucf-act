#include "uim342.hh"

#include <string.h>

#include "can_twai.hh"

// base CAN ID for UIM342 protocol
static const uint32_t kBaseCanId = 0x140;

// pack little-endian 16-bit value
static void fill_le16(uint8_t *dst, uint16_t value) {
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
}

// pack little-endian 32-bit value
static void fill_le32(uint8_t *dst, uint32_t value) {
    dst[0] = (uint8_t)(value & 0xFFU);
    dst[1] = (uint8_t)((value >> 8) & 0xFFU);
    dst[2] = (uint8_t)((value >> 16) & 0xFFU);
    dst[3] = (uint8_t)((value >> 24) & 0xFFU);
}

uint32_t motor_make_can_id(uint8_t node_id) {
    return kBaseCanId + node_id;
}

bool motor_can_id_to_node(uint32_t can_id, uint8_t *node_id) {
    if (can_id < kBaseCanId || can_id > (kBaseCanId + 127)) return false;
    
    if (node_id) *node_id = (uint8_t)(can_id - kBaseCanId);
    
    return true;
}

// send command frame to node
static esp_err_t send_command(uint8_t node_id, const uint8_t data[8]) {
    return can_twai_send(motor_make_can_id(node_id), data, pdMS_TO_TICKS(20));
}

esp_err_t motor_enable(uint8_t node_id) {
    uint8_t data[8] = {0xA0, 0, 0, 0, 0, 0, 0, 0};
    return send_command(node_id, data);
}

esp_err_t motor_disable(uint8_t node_id) {
    uint8_t data[8] = {0xA1, 0, 0, 0, 0, 0, 0, 0};
    return send_command(node_id, data);
}

esp_err_t motor_clear_fault(uint8_t node_id) {
    uint8_t data[8] = {0xA2, 0, 0, 0, 0, 0, 0, 0};
    return send_command(node_id, data);
}

esp_err_t motor_emergency_stop(uint8_t node_id) {
    uint8_t data[8] = {0xA3, 0, 0, 0, 0, 0, 0, 0};
    return send_command(node_id, data);
}

// velocity control: rpm is scaled by 100, accel by 10
esp_err_t motor_set_velocity(uint8_t node_id, int32_t rpm, uint16_t accel) {
    uint8_t data[8] = {0};
    data[0] = 0xA2;
    int32_t rpm_x100 = rpm * 100;
    uint16_t accel_x10 = (uint16_t)(accel * 10U);
    fill_le32(&data[1], (uint32_t)rpm_x100);
    fill_le16(&data[5], accel_x10);
    data[7] = 0x00;
    return send_command(node_id, data);
}

// position control: counts in encoder ticks, speed limit scaled by 10
esp_err_t motor_set_position(uint8_t node_id, int32_t counts, uint16_t speed_limit_rpm) {
    uint8_t data[8] = {0};
    data[0] = 0xA4;
    fill_le32(&data[1], (uint32_t)counts);
    uint16_t speed_x10 = (uint16_t)(speed_limit_rpm * 10U);
    fill_le16(&data[5], speed_x10);
    data[7] = 0x00;
    return send_command(node_id, data);
}

// decode status payload fields from the motor
void motor_parse_status_frame(const twai_message_t *msg, MotorStatus *out_status) {
    if (!msg || !out_status) return;
    
    out_status->temperature_c10 = (int16_t)(msg->data[0] | (msg->data[1] << 8));
    out_status->speed_rpm = (int16_t)(msg->data[2] | (msg->data[3] << 8));
    out_status->current_a100 = (int16_t)(msg->data[4] | (msg->data[5] << 8));
    out_status->fault_code = (uint16_t)(msg->data[6] | (msg->data[7] << 8));
}
