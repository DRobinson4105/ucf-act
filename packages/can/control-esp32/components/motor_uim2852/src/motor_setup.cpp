/*
 * Responsibility:
 * Blocking multi-step setup runner layered above motor_exec.
 */

#include "motor_setup.h"
#include "motor_setup_binding.h"
#include "motor_setup_internal.h"

#include <inttypes.h>
#include <string.h>

#include "esp_log.h"
#include "freertos/semphr.h"

#define MOTOR_SETUP_CALLBACK_WAIT_SLACK_TICKS pdMS_TO_TICKS(25)

static const char *TAG = "motor_setup";

#define BRAKE_PT_PV_DEMO_NODE_ID    6U
#define STEERING_PT_PV_DEMO_NODE_ID 7U

static const motor_setup_step_t s_brake_pt_pv_demo_steps[] = {
    {
        .name = "PP get bitrate",
        .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_GET,
            .object = MOTOR_OBJECT_PP,
            .ack_requested = true,
            .has_index = true,
            .index = MOTOR_PP_INDEX_BITRATE,
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PP get motor id",
        .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_GET,
            .object = MOTOR_OBJECT_PP,
            .ack_requested = true,
            .has_index = true,
            .index = MOTOR_PP_INDEX_NODE_ID,
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "OG set origin",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_ACTION,
            .object = MOTOR_OBJECT_OG,
            .ack_requested = true,
            .has_index = false,
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "SD set 3200",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_SD,
            .ack_requested = true,
            .has_index = false,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U32, .as = {.u32 = 3200U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MP[4] set 500",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MP,
            .ack_requested = true,
            .has_index = true,
            .index = 4U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 500U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MP[3] set 0",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MP,
            .ack_requested = true,
            .has_index = true,
            .index = 3U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 0U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MP[5] set 1",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MP,
            .ack_requested = true,
            .has_index = true,
            .index = 5U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 1U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MP[2] set 13",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MP,
            .ack_requested = true,
            .has_index = true,
            .index = 2U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 13U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MP[1] set 2",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MP,
            .ack_requested = true,
            .has_index = true,
            .index = 1U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 2U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MP[0] set 0",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MP,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 0U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 1600 1",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 1600}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 1600 2",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 1600}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 1",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 2",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 3",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 4",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 5",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 6",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = BRAKE_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 7",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = 6U,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MO set 1",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = 6U,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MO,
            .ack_requested = true,
            .has_index = false,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_ENUM, .as = {.enum_value = 1}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PV set 0",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = 6U,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PV,
            .ack_requested = true,
            .has_index = false,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 0U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "BG begin motion",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = 6U,
            .operation = MOTOR_EXEC_OPERATION_ACTION,
            .object = MOTOR_OBJECT_BG,
            .ack_requested = true,
            .has_index = false,
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
};

static const motor_setup_plan_t s_brake_pt_pv_demo_plan = {
    .name = "brake motion sequence demo",
    .steps = s_brake_pt_pv_demo_steps,
    .step_count = sizeof(s_brake_pt_pv_demo_steps) / sizeof(s_brake_pt_pv_demo_steps[0]),
};

static const motor_setup_step_t s_steering_pt_pv_demo_steps[] = {
    {
        .name = "PP get bitrate",
        .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_GET,
            .object = MOTOR_OBJECT_PP,
            .ack_requested = true,
            .has_index = true,
            .index = MOTOR_PP_INDEX_BITRATE,
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PP get motor id",
        .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_GET,
            .object = MOTOR_OBJECT_PP,
            .ack_requested = true,
            .has_index = true,
            .index = MOTOR_PP_INDEX_NODE_ID,
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "OG set origin",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_ACTION,
            .object = MOTOR_OBJECT_OG,
            .ack_requested = true,
            .has_index = false,
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "SD set 3200",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_SD,
            .ack_requested = true,
            .has_index = false,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U32, .as = {.u32 = 3200U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MP[4] set 500",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MP,
            .ack_requested = true,
            .has_index = true,
            .index = 4U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 500U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MP[3] set 0",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MP,
            .ack_requested = true,
            .has_index = true,
            .index = 3U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 0U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MP[5] set 1",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MP,
            .ack_requested = true,
            .has_index = true,
            .index = 5U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 1U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MP[2] set 13",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MP,
            .ack_requested = true,
            .has_index = true,
            .index = 2U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 13U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MP[1] set 2",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MP,
            .ack_requested = true,
            .has_index = true,
            .index = 1U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 2U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MP[0] set 0",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MP,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 0U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 1600 1",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 1600}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 1600 2",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 1600}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 1",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 2",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 3",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 4",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 5",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 6",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PT[0] set 0 7",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PT,
            .ack_requested = true,
            .has_index = true,
            .index = 0U,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_I32, .as = {.i32 = 0}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "MO set 1",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_MO,
            .ack_requested = true,
            .has_index = false,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_ENUM, .as = {.enum_value = 1}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "PV set 0",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_SET,
            .object = MOTOR_OBJECT_PV,
            .ack_requested = true,
            .has_index = false,
            .has_value = true,
            .value = {.kind = MOTOR_SETUP_VALUE_KIND_U16, .as = {.u16 = 0U}},
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
    {
        .name = "BG begin motion",
        .kind = MOTOR_SETUP_STEP_KIND_WRITE_ONLY,
        .exec_action = {
            .role = MOTOR_EXEC_ROLE_BRAKE,
            .node_id = STEERING_PT_PV_DEMO_NODE_ID,
            .operation = MOTOR_EXEC_OPERATION_ACTION,
            .object = MOTOR_OBJECT_BG,
            .ack_requested = true,
            .has_index = false,
        },
        .compare_kind = MOTOR_SETUP_COMPARE_NONE,
    },
};

static const motor_setup_plan_t s_steering_pt_pv_demo_plan = {
    .name = "steering motion sequence demo",
    .steps = s_steering_pt_pv_demo_steps,
    .step_count = sizeof(s_steering_pt_pv_demo_steps) / sizeof(s_steering_pt_pv_demo_steps[0]),
};

static motor_setup_log_cfg_t s_log_cfg = {
    .enabled = true,
    .log_step_success = true,
};
static motor_setup_submit_fn_t s_submit_fn = motor_setup_submit_action;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

typedef struct {
    SemaphoreHandle_t done_sem;
    motor_exec_result_t *result_out;
    bool completed;
} blocking_wait_t;

static bool is_valid_value_kind(motor_setup_value_kind_t kind)
{
    return kind > MOTOR_SETUP_VALUE_KIND_NONE && kind <= MOTOR_SETUP_VALUE_KIND_ENUM;
}

const motor_setup_plan_t *motor_setup_brake_pt_pv_demo_plan(void)
{
    return &s_brake_pt_pv_demo_plan;
}

const motor_setup_plan_t *motor_setup_steering_pt_pv_demo_plan(void)
{
    return &s_steering_pt_pv_demo_plan;
}

static bool action_requires_index(motor_object_t object)
{
    switch (object) {
        case MOTOR_OBJECT_PP:
        case MOTOR_OBJECT_IC:
        case MOTOR_OBJECT_IE:
        case MOTOR_OBJECT_ER:
        case MOTOR_OBJECT_QE:
        case MOTOR_OBJECT_SY:
        case MOTOR_OBJECT_MT:
        case MOTOR_OBJECT_IL:
        case MOTOR_OBJECT_MP:
        case MOTOR_OBJECT_PT:
        case MOTOR_OBJECT_LM:
            return true;
        default:
            return false;
    }
}

static bool action_requires_value(motor_exec_operation_t operation)
{
    switch (operation) {
        case MOTOR_EXEC_OPERATION_SET:
            return true;
        default:
            return false;
    }
}

static bool validate_action_container(const motor_setup_action_t *action)
{
    if (action == NULL) {
        return false;
    }

    if (action->role == MOTOR_EXEC_ROLE_NONE) {
        return false;
    }

    if (action_requires_index(action->object) != action->has_index) {
        return false;
    }

    if (action_requires_value(action->operation) != action->has_value) {
        return false;
    }

    if (action->has_value && !is_valid_value_kind(action->value.kind)) {
        return false;
    }

    return true;
}

static bool validate_action_binding(const motor_setup_action_t *action)
{
    return motor_setup_action_binding_supported(action);
}

static bool validate_action(const motor_setup_action_t *action)
{
    return validate_action_container(action) && validate_action_binding(action);
}

static bool compare_kind_matches_expected(motor_setup_compare_kind_t compare_kind,
                                          const motor_setup_value_t *expected)
{
    if (compare_kind == MOTOR_SETUP_COMPARE_NONE || expected == NULL) {
        return compare_kind == MOTOR_SETUP_COMPARE_NONE;
    }

    switch (compare_kind) {
        case MOTOR_SETUP_COMPARE_INT_EQ:
            return expected->kind == MOTOR_SETUP_VALUE_KIND_U8 ||
                   expected->kind == MOTOR_SETUP_VALUE_KIND_U16 ||
                   expected->kind == MOTOR_SETUP_VALUE_KIND_U32 ||
                   expected->kind == MOTOR_SETUP_VALUE_KIND_I32;
        case MOTOR_SETUP_COMPARE_ENUM_EQ:
            return expected->kind == MOTOR_SETUP_VALUE_KIND_ENUM;
        case MOTOR_SETUP_COMPARE_BOOL_EQ:
            return expected->kind == MOTOR_SETUP_VALUE_KIND_BOOL;
        case MOTOR_SETUP_COMPARE_NONE:
        default:
            return false;
    }
}

static bool validate_plan_container(const motor_setup_plan_t *plan)
{
    if (plan == NULL || plan->name == NULL) {
        return false;
    }

    if (plan->step_count == 0U) {
        return true;
    }

    return plan->steps != NULL;
}

static bool validate_step_container(const motor_setup_step_t *step)
{
    return step != NULL &&
           step->name != NULL &&
           step->kind > MOTOR_SETUP_STEP_KIND_UNKNOWN &&
           step->kind <= MOTOR_SETUP_STEP_KIND_WRITE_THEN_VERIFY;
}

static bool validate_step_verification(const motor_setup_step_t *step)
{
    if (step == NULL) {
        return false;
    }

    switch (step->kind) {
        case MOTOR_SETUP_STEP_KIND_WRITE_ONLY:
            return step->compare_kind == MOTOR_SETUP_COMPARE_NONE;
        case MOTOR_SETUP_STEP_KIND_READ_ONLY:
            return step->exec_action.operation == MOTOR_EXEC_OPERATION_GET &&
                   step->verify_action.role == MOTOR_EXEC_ROLE_NONE &&
                   !step->verify_action.has_index &&
                   !step->verify_action.has_value &&
                   !step->has_verify_timing_override &&
                   compare_kind_matches_expected(step->compare_kind, &step->expected_value);
        case MOTOR_SETUP_STEP_KIND_WRITE_THEN_VERIFY:
            return validate_action(&step->verify_action) &&
                   step->verify_action.operation == MOTOR_EXEC_OPERATION_GET &&
                   compare_kind_matches_expected(step->compare_kind, &step->expected_value);
        case MOTOR_SETUP_STEP_KIND_UNKNOWN:
        default:
            return false;
    }
}

static bool validate_step(const motor_setup_step_t *step)
{
    if (!validate_step_container(step)) {
        return false;
    }

    if (!validate_action(&step->exec_action)) {
        return false;
    }

    return validate_step_verification(step);
}

static const motor_exec_timing_t *timing_or_default(bool use_override,
                                                    const motor_exec_timing_t *override)
{
    static motor_exec_timing_t default_timing;

    if (use_override && override != NULL) {
        return override;
    }

    default_timing = motor_exec_default_timing();
    return &default_timing;
}

static TickType_t blocking_wait_limit(const motor_exec_timing_t *timing)
{
    TickType_t wait_ticks = MOTOR_SETUP_CALLBACK_WAIT_SLACK_TICKS;

    if (timing != NULL) {
        wait_ticks += timing->timeout_ticks;
        wait_ticks += timing->no_ack_grace_ticks;
    }

    if (wait_ticks == 0) {
        wait_ticks = 1;
    }

    return wait_ticks;
}

static void completion_cb(const motor_exec_result_t *result, void *ctx)
{
    blocking_wait_t *wait = (blocking_wait_t *)ctx;

    if (wait == NULL || result == NULL) {
        return;
    }

    if (wait->result_out != NULL) {
        *wait->result_out = *result;
    }
    wait->completed = true;
    if (wait->done_sem != NULL) {
        xSemaphoreGive(wait->done_sem);
    }
}

static bool setup_log_enabled(void)
{
    bool enabled;

    portENTER_CRITICAL(&s_lock);
    enabled = s_log_cfg.enabled;
    portEXIT_CRITICAL(&s_lock);

    return enabled;
}

static bool setup_log_step_success_enabled(void)
{
    bool enabled;

    portENTER_CRITICAL(&s_lock);
    enabled = s_log_cfg.enabled && s_log_cfg.log_step_success;
    portEXIT_CRITICAL(&s_lock);

    return enabled;
}

static bool copy_exec_value_as_setup(const motor_exec_value_t *src, motor_setup_value_t *dst)
{
    int64_t number;

    if (src == NULL || dst == NULL || !src->has_value) {
        return false;
    }

    memset(dst, 0, sizeof(*dst));
    number = src->has_raw_number ? src->raw_number : src->number;

    switch (src->kind) {
        case MOTOR_EXEC_VALUE_KIND_U8:
            dst->kind = MOTOR_SETUP_VALUE_KIND_U8;
            dst->as.u8 = (uint8_t)number;
            return true;
        case MOTOR_EXEC_VALUE_KIND_U16:
            dst->kind = MOTOR_SETUP_VALUE_KIND_U16;
            dst->as.u16 = (uint16_t)number;
            return true;
        case MOTOR_EXEC_VALUE_KIND_U32:
            dst->kind = MOTOR_SETUP_VALUE_KIND_U32;
            dst->as.u32 = (uint32_t)number;
            return true;
        case MOTOR_EXEC_VALUE_KIND_I32:
            dst->kind = MOTOR_SETUP_VALUE_KIND_I32;
            dst->as.i32 = (int32_t)number;
            return true;
        case MOTOR_EXEC_VALUE_KIND_ENUM:
            dst->kind = MOTOR_SETUP_VALUE_KIND_ENUM;
            dst->as.enum_value = (int32_t)number;
            return true;
        case MOTOR_EXEC_VALUE_KIND_TEXT:
        case MOTOR_EXEC_VALUE_KIND_NONE:
        default:
            return false;
    }
}

static bool compare_expected(const motor_setup_value_t *expected,
                             motor_setup_compare_kind_t compare_kind,
                             const motor_exec_result_t *verify_result,
                             motor_setup_value_t *out_actual)
{
    int64_t actual_number;

    if (compare_kind == MOTOR_SETUP_COMPARE_NONE) {
        return true;
    }

    if (expected == NULL || verify_result == NULL || !verify_result->has_response_value) {
        return false;
    }

    if (out_actual != NULL) {
        (void)copy_exec_value_as_setup(&verify_result->response_value, out_actual);
    }

    actual_number = verify_result->response_value.has_raw_number ?
        verify_result->response_value.raw_number :
        verify_result->response_value.number;

    switch (compare_kind) {
        case MOTOR_SETUP_COMPARE_INT_EQ:
            switch (expected->kind) {
                case MOTOR_SETUP_VALUE_KIND_U8:
                    return actual_number == expected->as.u8;
                case MOTOR_SETUP_VALUE_KIND_U16:
                    return actual_number == expected->as.u16;
                case MOTOR_SETUP_VALUE_KIND_U32:
                    return actual_number == (int64_t)expected->as.u32;
                case MOTOR_SETUP_VALUE_KIND_I32:
                    return actual_number == expected->as.i32;
                default:
                    return false;
            }
        case MOTOR_SETUP_COMPARE_ENUM_EQ:
            return actual_number == expected->as.enum_value;
        case MOTOR_SETUP_COMPARE_BOOL_EQ:
            return (actual_number != 0) == expected->as.boolean;
        case MOTOR_SETUP_COMPARE_NONE:
        default:
            return true;
    }
}

static bool comparison_required(motor_setup_compare_kind_t compare_kind)
{
    return compare_kind != MOTOR_SETUP_COMPARE_NONE;
}

static void log_plan_start(const motor_setup_plan_t *plan)
{
    if (!setup_log_enabled() || plan == NULL) {
        return;
    }

    ESP_LOGI(TAG, "plan=%s start steps=%u", plan->name, (unsigned)plan->step_count);
}

static void log_step_start(const motor_setup_step_result_t *step_result)
{
    if (!setup_log_enabled() || step_result == NULL) {
        return;
    }

    ESP_LOGI(TAG, "plan step=%u name=%s start",
             (unsigned)step_result->step_index, step_result->step_name);
}

static void log_step_success(const motor_setup_step_result_t *step_result)
{
    if (!setup_log_step_success_enabled() || step_result == NULL) {
        return;
    }

    ESP_LOGI(TAG, "plan step=%u name=%s success",
             (unsigned)step_result->step_index, step_result->step_name);
}

static void log_step_failure(const motor_setup_step_result_t *step_result)
{
    if (!setup_log_enabled() || step_result == NULL) {
        return;
    }

    switch (step_result->failure_reason) {
        case MOTOR_SETUP_FAILURE_VERIFY_MISMATCH:
            ESP_LOGW(TAG, "plan step=%u name=%s verify mismatch",
                     (unsigned)step_result->step_index, step_result->step_name);
            break;
        case MOTOR_SETUP_FAILURE_EXEC_RESULT:
        case MOTOR_SETUP_FAILURE_VERIFY_RESULT:
            ESP_LOGW(TAG, "plan step=%u name=%s terminal failure status=%d",
                     (unsigned)step_result->step_index,
                     step_result->step_name,
                     (int)(step_result->verification_attempted ?
                           step_result->verification_result.status :
                           step_result->execution_result.status));
            break;
        default:
            ESP_LOGW(TAG, "plan step=%u name=%s failed reason=%d",
                     (unsigned)step_result->step_index,
                     step_result->step_name,
                     (int)step_result->failure_reason);
            break;
    }
}

static void log_plan_finish(const motor_setup_result_t *result)
{
    if (!setup_log_enabled() || result == NULL) {
        return;
    }

    if (result->status == MOTOR_SETUP_STATUS_SUCCESS) {
        ESP_LOGI(TAG, "plan=%s success completed=%u/%u",
                 result->plan_name,
                 (unsigned)result->completed_steps,
                 (unsigned)result->total_steps);
        return;
    }

    ESP_LOGW(TAG, "plan=%s failed completed=%u/%u step=%u name=%s reason=%d",
             result->plan_name != NULL ? result->plan_name : "<null>",
             (unsigned)result->completed_steps,
             (unsigned)result->total_steps,
             (unsigned)result->failing_step_index,
             result->failing_step_name != NULL ? result->failing_step_name : "<null>",
             (int)result->failure_reason);
}

static motor_exec_submit_result_t run_action_blocking(const motor_setup_action_t *action,
                                                      const motor_exec_timing_t *timing,
                                                      motor_exec_result_t *out_result)
{
    blocking_wait_t wait = {};
    motor_exec_submit_opts_t opts = {
        .on_complete = completion_cb,
        .ctx = &wait,
        .timing = *timing,
    };
    motor_exec_submit_result_t submit;

    wait.result_out = out_result;
    wait.done_sem = xSemaphoreCreateBinary();
    if (wait.done_sem == NULL) {
        return (motor_exec_submit_result_t) {
            .accepted = false,
            .code = MOTOR_EXEC_SUBMIT_SCHEDULING_FAILED,
            .err = ESP_ERR_NO_MEM,
        };
    }

    submit = s_submit_fn(action, &opts);
    if (submit.accepted) {
        if (!(xSemaphoreTake(wait.done_sem, blocking_wait_limit(timing)) == pdTRUE && wait.completed)) {
            submit.accepted = false;
            submit.code = MOTOR_EXEC_SUBMIT_SCHEDULING_FAILED;
            submit.err = ESP_ERR_TIMEOUT;
        }
    }

    vSemaphoreDelete(wait.done_sem);
    return submit;
}

static void fill_failed_result(motor_setup_result_t *out_result,
                               const motor_setup_step_result_t *step_result)
{
    if (out_result == NULL || step_result == NULL) {
        return;
    }

    out_result->status = step_result->failure_reason == MOTOR_SETUP_FAILURE_INVALID_PLAN ?
        MOTOR_SETUP_STATUS_INVALID_PLAN :
        MOTOR_SETUP_STATUS_FAILED;
    out_result->has_failure = true;
    out_result->failing_step_index = step_result->step_index;
    out_result->failing_step_name = step_result->step_name;
    out_result->failure_reason = step_result->failure_reason;
    out_result->failing_step = *step_result;
}

static void execute_step(const motor_setup_step_t *step,
                         size_t step_index,
                         motor_setup_result_t *out_result,
                         motor_setup_step_result_t *out_step_result)
{
    const motor_exec_timing_t *exec_timing;
    const motor_exec_timing_t *verify_timing;

    if (step == NULL || out_result == NULL || out_step_result == NULL) {
        return;
    }

    exec_timing = timing_or_default(step->has_exec_timing_override, &step->exec_timing);
    verify_timing = timing_or_default(step->has_verify_timing_override, &step->verify_timing);
    memset(out_step_result, 0, sizeof(*out_step_result));
    out_step_result->step_name = step->name;
    out_step_result->step_index = step_index;

    log_step_start(out_step_result);

    out_step_result->execution_attempted = true;
    out_step_result->execution_submit = run_action_blocking(&step->exec_action,
                                                            exec_timing,
                                                            &out_step_result->execution_result);
    if (!out_step_result->execution_submit.accepted) {
        out_step_result->failure_reason = MOTOR_SETUP_FAILURE_EXEC_SUBMIT;
        fill_failed_result(out_result, out_step_result);
        return;
    }

    if (out_step_result->execution_result.status != MOTOR_EXEC_STATUS_SUCCESS) {
        out_step_result->execution_success = false;
        out_step_result->failure_reason = MOTOR_SETUP_FAILURE_EXEC_RESULT;
        fill_failed_result(out_result, out_step_result);
        return;
    }

    out_step_result->execution_success = true;

    if (step->kind == MOTOR_SETUP_STEP_KIND_READ_ONLY) {
        out_step_result->verification_attempted = true;
        out_step_result->verification_success = true;
        out_step_result->verification_result = out_step_result->execution_result;
        out_step_result->has_actual_value = out_step_result->execution_result.has_response_value;
        if (comparison_required(step->compare_kind)) {
            out_step_result->comparison_attempted = true;
            out_step_result->comparison_success = compare_expected(&step->expected_value,
                                                                   step->compare_kind,
                                                                   &out_step_result->execution_result,
                                                                   &out_step_result->actual_value);
            if (!out_step_result->comparison_success) {
                out_step_result->failure_reason = MOTOR_SETUP_FAILURE_VERIFY_MISMATCH;
                fill_failed_result(out_result, out_step_result);
            }
        }
    } else if (step->kind == MOTOR_SETUP_STEP_KIND_WRITE_THEN_VERIFY) {
        out_step_result->verification_attempted = true;
        out_step_result->verification_submit = run_action_blocking(&step->verify_action,
                                                                   verify_timing,
                                                                   &out_step_result->verification_result);
        if (!out_step_result->verification_submit.accepted) {
            out_step_result->failure_reason = MOTOR_SETUP_FAILURE_VERIFY_SUBMIT;
            fill_failed_result(out_result, out_step_result);
            return;
        }

        if (out_step_result->verification_result.status != MOTOR_EXEC_STATUS_SUCCESS) {
            out_step_result->failure_reason = MOTOR_SETUP_FAILURE_VERIFY_RESULT;
            fill_failed_result(out_result, out_step_result);
            return;
        }

        out_step_result->verification_success = true;
        out_step_result->has_actual_value = out_step_result->verification_result.has_response_value;
        if (comparison_required(step->compare_kind)) {
            out_step_result->comparison_attempted = true;
            out_step_result->comparison_success = compare_expected(&step->expected_value,
                                                                   step->compare_kind,
                                                                   &out_step_result->verification_result,
                                                                   &out_step_result->actual_value);
            if (!out_step_result->comparison_success) {
                out_step_result->failure_reason = MOTOR_SETUP_FAILURE_VERIFY_MISMATCH;
                fill_failed_result(out_result, out_step_result);
            }
        }
    }
}

motor_setup_run_cfg_t motor_setup_default_run_cfg(void)
{
    return (motor_setup_run_cfg_t) {
        .step_results = NULL,
        .step_results_capacity = 0U,
    };
}

motor_setup_log_cfg_t motor_setup_default_log_cfg(void)
{
    return (motor_setup_log_cfg_t) {
        .enabled = true,
        .log_step_success = true,
    };
}

esp_err_t motor_setup_set_log_cfg(const motor_setup_log_cfg_t *cfg)
{
    if (cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    s_log_cfg = *cfg;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t motor_setup_get_log_cfg(motor_setup_log_cfg_t *out_cfg)
{
    if (out_cfg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    portENTER_CRITICAL(&s_lock);
    *out_cfg = s_log_cfg;
    portEXIT_CRITICAL(&s_lock);
    return ESP_OK;
}

esp_err_t motor_setup_validate_plan(const motor_setup_plan_t *plan)
{
    size_t i;

    if (!validate_plan_container(plan)) {
        return ESP_ERR_INVALID_ARG;
    }

    if (plan->step_count == 0U) {
        return ESP_OK;
    }

    for (i = 0; i < plan->step_count; ++i) {
        if (!validate_step(&plan->steps[i])) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    return ESP_OK;
}

esp_err_t motor_setup_run_plan(const motor_setup_plan_t *plan,
                               const motor_setup_run_cfg_t *cfg,
                               motor_setup_result_t *out_result)
{
    motor_setup_run_cfg_t effective_cfg;
    esp_err_t err;
    size_t i;

    if (out_result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(out_result, 0, sizeof(*out_result));
    out_result->status = MOTOR_SETUP_STATUS_UNKNOWN;

    if (plan == NULL) {
        out_result->status = MOTOR_SETUP_STATUS_INVALID_PLAN;
        out_result->failure_reason = MOTOR_SETUP_FAILURE_INVALID_PLAN;
        return ESP_ERR_INVALID_ARG;
    }

    out_result->plan_name = plan->name;
    out_result->total_steps = plan->step_count;
    effective_cfg = cfg != NULL ? *cfg : motor_setup_default_run_cfg();

    err = motor_setup_validate_plan(plan);
    if (err != ESP_OK) {
        out_result->status = MOTOR_SETUP_STATUS_INVALID_PLAN;
        out_result->has_failure = true;
        out_result->failure_reason = MOTOR_SETUP_FAILURE_INVALID_PLAN;
        log_plan_finish(out_result);
        return err;
    }

    log_plan_start(plan);

    for (i = 0; i < plan->step_count; ++i) {
        const motor_setup_step_t *step = &plan->steps[i];
        motor_setup_step_result_t *step_result = &out_result->failing_step;

        memset(step_result, 0, sizeof(*step_result));
        execute_step(step, i, out_result, step_result);

        if (effective_cfg.step_results != NULL && i < effective_cfg.step_results_capacity) {
            effective_cfg.step_results[i] = *step_result;
            out_result->step_results_written = i + 1U;
        }

        if (out_result->has_failure) {
            log_step_failure(step_result);
            log_plan_finish(out_result);
            return ESP_OK;
        }

        out_result->completed_steps++;
        log_step_success(step_result);
    }

    out_result->status = MOTOR_SETUP_STATUS_SUCCESS;
    memset(&out_result->failing_step, 0, sizeof(out_result->failing_step));
    log_plan_finish(out_result);
    return ESP_OK;
}

void motor_setup_test_set_submit_fn(motor_setup_submit_fn_t fn)
{
    portENTER_CRITICAL(&s_lock);
    s_submit_fn = fn != NULL ? fn : motor_setup_submit_action;
    portEXIT_CRITICAL(&s_lock);
}

void motor_setup_test_reset_state(void)
{
    portENTER_CRITICAL(&s_lock);
    s_log_cfg = motor_setup_default_log_cfg();
    s_submit_fn = motor_setup_submit_action;
    portEXIT_CRITICAL(&s_lock);
}
