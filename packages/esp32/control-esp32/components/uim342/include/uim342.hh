#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "driver/twai.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int16_t temperature_c10;
    int16_t speed_rpm;
    int16_t current_a100;
    uint16_t fault_code;
} MotorStatus;

// compute motor CAN identifier (0x140 + node_id)
uint32_t motor_make_can_id(uint8_t node_id);

// convert CAN ID to node_id if it is in the valid range
bool motor_can_id_to_node(uint32_t can_id, uint8_t *node_id);

// enable motor (command 0xA0)
esp_err_t motor_enable(uint8_t node_id);

// disable motor (command 0xA1)
esp_err_t motor_disable(uint8_t node_id);

// clear motor fault (command 0xA2)
esp_err_t motor_clear_fault(uint8_t node_id);

// emergency stop (command 0xA3)
esp_err_t motor_emergency_stop(uint8_t node_id);

// closed-loop velocity control, rpm in RPM, accel in RPM/s
esp_err_t motor_set_velocity(uint8_t node_id, int32_t rpm, uint16_t accel);

// position control, counts in encoder ticks, speed_limit_rpm in RPM
esp_err_t motor_set_position(uint8_t node_id, int32_t counts, uint16_t speed_limit_rpm);

// parse an 8-byte status frame into a MotorStatus struct
void motor_parse_status_frame(const twai_message_t *msg, MotorStatus *out_status);

#ifdef __cplusplus
}
#endif
