/*
 * Responsibility:
 * Outgoing command catalog for building motor protocol TX frames.
 */

#include "motor_codec.h"

#include <assert.h>

#define MOTOR_CMD_BASE_PP 0x01U
#define MOTOR_CMD_BASE_IC 0x06U
#define MOTOR_CMD_BASE_IE 0x07U
#define MOTOR_CMD_BASE_ER 0x0FU
#define MOTOR_CMD_BASE_SD 0x1CU
#define MOTOR_CMD_BASE_SY 0x7EU
#define MOTOR_CMD_BASE_MT 0x10U
#define MOTOR_CMD_BASE_MO 0x15U
#define MOTOR_CMD_BASE_BG 0x16U
#define MOTOR_CMD_BASE_ST 0x17U
#define MOTOR_CMD_BASE_PA 0x20U
#define MOTOR_CMD_BASE_MP 0x22U
#define MOTOR_CMD_BASE_OG 0x21U
#define MOTOR_CMD_BASE_LM 0x2CU
#define MOTOR_CMD_BASE_BL 0x2DU
#define MOTOR_CMD_BASE_IL 0x34U
#define MOTOR_CMD_BASE_QE 0x3DU

/*
 * RT is RX-only; no TX builder intentionally.
 */

/*
 * Payload families currently covered in this file:
 * - no payload
 * - u8 payload
 * - i32 payload
 * - index only
 * - index + u8
 * - index + u16
 * - index + i32
 */

static bool validate_target_id(uint8_t target_id)
{
    return (target_id == 0U) || (target_id >= 5U && target_id <= 126U);
}

static esp_err_t validate_out_and_target(uint8_t target_id, const motor_cmd_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!validate_target_id(target_id)) {
        return ESP_ERR_INVALID_ARG;
    }

    return ESP_OK;
}

static bool validate_pp_index(motor_pp_index_t index)
{
    return index == MOTOR_PP_INDEX_BITRATE ||
           index == MOTOR_PP_INDEX_NODE_ID ||
           index == MOTOR_PP_INDEX_GROUP_ID;
}

static bool validate_ic_index(motor_ic_index_t index)
{
    return index == MOTOR_IC_INDEX_AUTO_ENABLE_AFTER_POWERUP ||
           index == MOTOR_IC_INDEX_POSITIVE_MOTOR_DIRECTION ||
           index == MOTOR_IC_INDEX_ENABLE_LOCKDOWN_SYSTEM ||
           index == MOTOR_IC_INDEX_AC_DC_UNITS ||
           index == MOTOR_IC_INDEX_USE_CLOSED_LOOP ||
           index == MOTOR_IC_INDEX_SOFTWARE_LIMIT_ENABLE ||
           index == MOTOR_IC_INDEX_INTERNAL_BRAKE_LOGIC_ENABLE ||
           index == MOTOR_IC_INDEX_USE_INTERNAL_BRAKE;
}

static bool validate_er_index(motor_er_index_t index)
{
    return index >= MOTOR_ER_INDEX_HISTORY_1 &&
           index <= MOTOR_ER_INDEX_HISTORY_9;
}

static bool validate_ie_index(motor_ie_index_t index)
{
    return index == MOTOR_IE_INDEX_IN1_CHANGE_NOTIFICATION ||
           index == MOTOR_IE_INDEX_IN2_CHANGE_NOTIFICATION ||
           index == MOTOR_IE_INDEX_IN3_CHANGE_NOTIFICATION ||
           index == MOTOR_IE_INDEX_PTP_POSITIONING_FINISH_NOTIFICATION ||
           index == MOTOR_IE_INDEX_FIFO_EMPTY_NOTIFICATION ||
           index == MOTOR_IE_INDEX_FIFO_LOW_WARNING_NOTIFICATION;
}

static bool validate_qe_index(motor_qe_index_t index)
{
    return index == MOTOR_QE_INDEX_LINES_PER_REVOLUTION ||
           index == MOTOR_QE_INDEX_MAX_POSITION_ERROR ||
           index == MOTOR_QE_INDEX_BATTERY_VOLTAGE ||
           index == MOTOR_QE_INDEX_COUNTS_PER_REVOLUTION;
}

static bool validate_mt_index(motor_mt_index_t index)
{
    return index == MOTOR_MT_INDEX_MICROSTEP ||
           index == MOTOR_MT_INDEX_WORKING_CURRENT ||
           index == MOTOR_MT_INDEX_IDLE_CURRENT_PERCENT ||
           index == MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS ||
           index == MOTOR_MT_INDEX_BRAKE_STATE;
}

static bool validate_il_index(motor_il_index_t index)
{
    return index == MOTOR_IL_INDEX_IN1_TRIGGER_ACTION ||
           index == MOTOR_IL_INDEX_IN2_TRIGGER_ACTION ||
           index == MOTOR_IL_INDEX_IN3_TRIGGER_ACTION ||
           index == MOTOR_IL_INDEX_STALL_BEHAVIOR;
}

static bool validate_lm_index(motor_lm_index_t index)
{
    return index == MOTOR_LM_INDEX_MAX_SPEED ||
           index == MOTOR_LM_INDEX_LOWER_WORKING_LIMIT ||
           index == MOTOR_LM_INDEX_UPPER_WORKING_LIMIT ||
           index == MOTOR_LM_INDEX_LOWER_BUMPING_LIMIT ||
           index == MOTOR_LM_INDEX_UPPER_BUMPING_LIMIT ||
           index == MOTOR_LM_INDEX_MAX_POSITION_ERROR ||
           index == MOTOR_LM_INDEX_MAX_ACCEL_DECEL ||
           index == MOTOR_LM_INDEX_RESET_LIMITS ||
           index == MOTOR_LM_INDEX_ENABLE_DISABLE;
}

static bool validate_mp_index(motor_mp_index_t index)
{
    return index == MOTOR_MP_INDEX_CURRENT_QUEUE_LEVEL_OR_RESET_TABLE ||
           index == MOTOR_MP_INDEX_FIRST_VALID_ROW ||
           index == MOTOR_MP_INDEX_LAST_VALID_ROW ||
           index == MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE ||
           index == MOTOR_MP_INDEX_PT_MOTION_TIME ||
           index == MOTOR_MP_INDEX_QUEUE_LOW_THRESHOLD ||
           index == MOTOR_MP_INDEX_NEXT_AVAILABLE_WRITING_ROW;
}

static bool validate_pp_value(motor_pp_index_t index, uint8_t value)
{
    switch (index) {
        case MOTOR_PP_INDEX_BITRATE:
            return value <= MOTOR_PP_BITRATE_125K;
        case MOTOR_PP_INDEX_NODE_ID:
        case MOTOR_PP_INDEX_GROUP_ID:
            return value >= 5U && value <= 126U;
        default:
            return false;
    }
}

static bool validate_ic_value(uint16_t value)
{
    return value == 0U || value == 1U;
}

static bool validate_ie_value(uint16_t value)
{
    return value == MOTOR_IE_STATE_DISABLE || value == MOTOR_IE_STATE_ENABLE;
}

static bool validate_mt_value(motor_mt_index_t index, uint16_t value)
{
    switch (index) {
        case MOTOR_MT_INDEX_MICROSTEP:
            return value == 1U || value == 2U || value == 4U || value == 8U ||
                   value == 16U || value == 32U || value == 64U;
        case MOTOR_MT_INDEX_WORKING_CURRENT:
            return value >= 5U && value <= 80U;
        case MOTOR_MT_INDEX_IDLE_CURRENT_PERCENT:
            return value <= 100U;
        case MOTOR_MT_INDEX_AUTO_ENABLE_DELAY_MS:
            return value <= 60000U;
        case MOTOR_MT_INDEX_BRAKE_STATE:
            return value == MOTOR_MT_BRAKE_STATE_RELEASED || value == MOTOR_MT_BRAKE_STATE_ENGAGED;
        default:
            return false;
    }
}

static bool validate_mo_state(motor_mo_state_t state)
{
    return state == MOTOR_MO_STATE_DISABLE || state == MOTOR_MO_STATE_ENABLE;
}

static bool validate_mp_value(motor_mp_index_t index, uint16_t value)
{
    switch (index) {
        case MOTOR_MP_INDEX_PVT_DATA_MANAGEMENT_MODE:
            return value == MOTOR_MP_MODE_FIFO ||
                   value == MOTOR_MP_MODE_SINGLE ||
                   value == MOTOR_MP_MODE_LOOP;
        default:
            return true;
    }
}

static bool validate_lm_value(motor_lm_index_t index, int32_t value)
{
    switch (index) {
        case MOTOR_LM_INDEX_MAX_SPEED:
            return value >= 0;
        case MOTOR_LM_INDEX_LOWER_WORKING_LIMIT:
        case MOTOR_LM_INDEX_UPPER_WORKING_LIMIT:
        case MOTOR_LM_INDEX_LOWER_BUMPING_LIMIT:
        case MOTOR_LM_INDEX_UPPER_BUMPING_LIMIT:
            return true;
        case MOTOR_LM_INDEX_MAX_POSITION_ERROR:
            return value >= 50 && value <= 65535;
        case MOTOR_LM_INDEX_MAX_ACCEL_DECEL:
            return value >= 1 && value <= 1000000;
        case MOTOR_LM_INDEX_RESET_LIMITS:
            return value == 1;
        case MOTOR_LM_INDEX_ENABLE_DISABLE:
            return value == 0 || value == 1;
        default:
            return false;
    }
}

static void set_index_metadata(motor_cmd_t *out, uint16_t index)
{
    assert(out != NULL);
    assert(index <= UINT8_MAX);

    out->has_index = true;
    out->index = index;
}

/*
 * Temporary internal payload-shape helpers.
 * These are intentionally private and used only to reduce repetition while the
 * family-specific command catalog is still being built out.
 */
static esp_err_t build_no_payload(uint8_t target_id,
                                  motor_section_t section,
                                  motor_object_t object,
                                  uint8_t base_code,
                                  bool ack_requested,
                                  const char *name,
                                  motor_cmd_t *out)
{
    esp_err_t err = validate_out_and_target(target_id, out);

    assert(name != NULL);

    if (err != ESP_OK) {
        return err;
    }

    err = motor_codec_init_cmd(target_id, section, object, base_code, ack_requested, 0U, name, out);
    assert(err != ESP_OK || out->msg.data_length_code <= 8U);
    return err;
}

static esp_err_t build_index_only(uint8_t target_id,
                                  motor_section_t section,
                                  motor_object_t object,
                                  uint8_t base_code,
                                  bool ack_requested,
                                  uint16_t index,
                                  const char *name,
                                  motor_cmd_t *out)
{
    esp_err_t err = validate_out_and_target(target_id, out);

    assert(name != NULL);
    assert(index <= UINT8_MAX);

    if (err != ESP_OK) {
        return err;
    }

    err = motor_codec_init_cmd(target_id, section, object, base_code, ack_requested, 1U, name, out);
    if (err != ESP_OK) {
        return err;
    }

    assert(out->msg.data_length_code <= 8U);
    out->msg.data[0] = (uint8_t)index;
    set_index_metadata(out, index);
    return ESP_OK;
}

static esp_err_t build_u8_payload(uint8_t target_id,
                                  motor_section_t section,
                                  motor_object_t object,
                                  uint8_t base_code,
                                  bool ack_requested,
                                  uint8_t value,
                                  const char *name,
                                  motor_cmd_t *out)
{
    esp_err_t err = validate_out_and_target(target_id, out);

    assert(name != NULL);

    if (err != ESP_OK) {
        return err;
    }

    err = motor_codec_init_cmd(target_id, section, object, base_code, ack_requested, 1U, name, out);
    if (err != ESP_OK) {
        return err;
    }

    assert(out->msg.data_length_code <= 8U);
    out->msg.data[0] = value;
    return ESP_OK;
}

static esp_err_t build_index_u8(uint8_t target_id,
                                motor_section_t section,
                                motor_object_t object,
                                uint8_t base_code,
                                bool ack_requested,
                                uint16_t index,
                                uint8_t value,
                                const char *name,
                                motor_cmd_t *out)
{
    esp_err_t err = validate_out_and_target(target_id, out);

    assert(name != NULL);
    assert(index <= UINT8_MAX);

    if (err != ESP_OK) {
        return err;
    }

    err = motor_codec_init_cmd(target_id, section, object, base_code, ack_requested, 2U, name, out);
    if (err != ESP_OK) {
        return err;
    }

    assert(out->msg.data_length_code <= 8U);
    out->msg.data[0] = (uint8_t)index;
    out->msg.data[1] = value;
    set_index_metadata(out, index);
    return ESP_OK;
}

static esp_err_t build_index_u16(uint8_t target_id,
                                 motor_section_t section,
                                 motor_object_t object,
                                 uint8_t base_code,
                                 bool ack_requested,
                                 uint16_t index,
                                 uint16_t value,
                                 const char *name,
                                 motor_cmd_t *out)
{
    esp_err_t err = validate_out_and_target(target_id, out);

    assert(name != NULL);
    assert(index <= UINT8_MAX);

    if (err != ESP_OK) {
        return err;
    }

    err = motor_codec_init_cmd(target_id, section, object, base_code, ack_requested, 3U, name, out);
    if (err != ESP_OK) {
        return err;
    }

    assert(out->msg.data_length_code <= 8U);
    out->msg.data[0] = (uint8_t)index;
    motor_codec_pack_u16_le(&out->msg.data[1], value);
    set_index_metadata(out, index);
    return ESP_OK;
}

static esp_err_t build_i32_payload(uint8_t target_id,
                                   motor_section_t section,
                                   motor_object_t object,
                                   uint8_t base_code,
                                   bool ack_requested,
                                   int32_t value,
                                   const char *name,
                                   motor_cmd_t *out)
{
    esp_err_t err = validate_out_and_target(target_id, out);

    assert(name != NULL);

    if (err != ESP_OK) {
        return err;
    }

    err = motor_codec_init_cmd(target_id, section, object, base_code, ack_requested, 4U, name, out);
    if (err != ESP_OK) {
        return err;
    }

    assert(out->msg.data_length_code <= 8U);
    motor_codec_pack_i32_le(&out->msg.data[0], value);
    return ESP_OK;
}

static esp_err_t build_u32_payload(uint8_t target_id,
                                   motor_section_t section,
                                   motor_object_t object,
                                   uint8_t base_code,
                                   bool ack_requested,
                                   uint32_t value,
                                   const char *name,
                                   motor_cmd_t *out)
{
    esp_err_t err = validate_out_and_target(target_id, out);

    assert(name != NULL);

    if (err != ESP_OK) {
        return err;
    }

    err = motor_codec_init_cmd(target_id, section, object, base_code, ack_requested, 4U, name, out);
    if (err != ESP_OK) {
        return err;
    }

    assert(out->msg.data_length_code <= 8U);
    motor_codec_pack_u32_le(&out->msg.data[0], value);
    return ESP_OK;
}

static esp_err_t build_index_i32(uint8_t target_id,
                                 motor_section_t section,
                                 motor_object_t object,
                                 uint8_t base_code,
                                 bool ack_requested,
                                 uint16_t index,
                                 int32_t value,
                                 const char *name,
                                 motor_cmd_t *out)
{
    esp_err_t err = validate_out_and_target(target_id, out);

    assert(name != NULL);
    assert(index <= UINT8_MAX);

    if (err != ESP_OK) {
        return err;
    }

    err = motor_codec_init_cmd(target_id, section, object, base_code, ack_requested, 5U, name, out);
    if (err != ESP_OK) {
        return err;
    }

    assert(out->msg.data_length_code <= 8U);
    out->msg.data[0] = (uint8_t)index;
    motor_codec_pack_i32_le(&out->msg.data[1], value);
    set_index_metadata(out, index);
    return ESP_OK;
}

esp_err_t motor_cmd_pp_get(uint8_t target_id, bool ack_requested, motor_pp_index_t index, motor_cmd_t *out)
{
    if (!validate_pp_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_only(target_id, MOTOR_SECTION_PROTOCOL, MOTOR_OBJECT_PP,
                            MOTOR_CMD_BASE_PP, ack_requested, (uint16_t)index, "PP_GET", out);
}

esp_err_t motor_cmd_pp_set_u8(uint8_t target_id, bool ack_requested, motor_pp_index_t index, uint8_t value, motor_cmd_t *out)
{
    if (!validate_pp_index(index) || !validate_pp_value(index, value)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_u8(target_id, MOTOR_SECTION_PROTOCOL, MOTOR_OBJECT_PP,
                          MOTOR_CMD_BASE_PP, ack_requested, (uint16_t)index, value, "PP_SET_U8", out);
}

esp_err_t motor_cmd_ic_get(uint8_t target_id, bool ack_requested, motor_ic_index_t index, motor_cmd_t *out)
{
    if (!validate_ic_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_only(target_id, MOTOR_SECTION_SYSTEM, MOTOR_OBJECT_IC,
                            MOTOR_CMD_BASE_IC, ack_requested, (uint16_t)index, "IC_GET", out);
}

esp_err_t motor_cmd_ic_set_u16(uint8_t target_id, bool ack_requested, motor_ic_index_t index, uint16_t value, motor_cmd_t *out)
{
    if (!validate_ic_index(index) || !validate_ic_value(value)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_u16(target_id, MOTOR_SECTION_SYSTEM, MOTOR_OBJECT_IC,
                           MOTOR_CMD_BASE_IC, ack_requested, (uint16_t)index, value, "IC_SET_U16", out);
}

esp_err_t motor_cmd_ie_get(uint8_t target_id, bool ack_requested, motor_ie_index_t index, motor_cmd_t *out)
{
    if (!validate_ie_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_only(target_id, MOTOR_SECTION_SYSTEM, MOTOR_OBJECT_IE,
                            MOTOR_CMD_BASE_IE, ack_requested, (uint16_t)index, "IE_GET", out);
}

esp_err_t motor_cmd_ie_set_u16(uint8_t target_id, bool ack_requested, motor_ie_index_t index, uint16_t value, motor_cmd_t *out)
{
    if (!validate_ie_index(index) || !validate_ie_value(value)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_u16(target_id, MOTOR_SECTION_SYSTEM, MOTOR_OBJECT_IE,
                           MOTOR_CMD_BASE_IE, ack_requested, (uint16_t)index, value, "IE_SET_U16", out);
}

esp_err_t motor_cmd_er_get(uint8_t target_id, bool ack_requested, motor_er_index_t index, motor_cmd_t *out)
{
    if (!validate_er_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_only(target_id, MOTOR_SECTION_SYSTEM, MOTOR_OBJECT_ER,
                            MOTOR_CMD_BASE_ER, ack_requested, (uint16_t)index, "ER_GET", out);
}

esp_err_t motor_cmd_qe_get(uint8_t target_id, bool ack_requested, motor_qe_index_t index, motor_cmd_t *out)
{
    if (!validate_qe_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_only(target_id, MOTOR_SECTION_SYSTEM, MOTOR_OBJECT_QE,
                            MOTOR_CMD_BASE_QE, ack_requested, (uint16_t)index, "QE_GET", out);
}

esp_err_t motor_cmd_qe_set_u16(uint8_t target_id, bool ack_requested, motor_qe_index_t index, uint16_t value, motor_cmd_t *out)
{
    if (!validate_qe_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_u16(target_id, MOTOR_SECTION_SYSTEM, MOTOR_OBJECT_QE,
                           MOTOR_CMD_BASE_QE, ack_requested, (uint16_t)index, value, "QE_SET_U16", out);
}

esp_err_t motor_cmd_er_clear_all(uint8_t target_id, bool ack_requested, motor_cmd_t *out)
{
    esp_err_t err = validate_out_and_target(target_id, out);

    assert(out != NULL);

    if (err != ESP_OK) {
        return err;
    }

    err = motor_codec_init_cmd(target_id, MOTOR_SECTION_SYSTEM, MOTOR_OBJECT_ER,
                               MOTOR_CMD_BASE_ER, ack_requested, 2U, "ER_CLEAR_ALL", out);
    if (err != ESP_OK) {
        return err;
    }

    assert(out->msg.data_length_code <= 8U);
    out->msg.data[0] = (uint8_t)MOTOR_ER_INDEX_CLEAR_ALL;
    out->msg.data[1] = 0U;
    set_index_metadata(out, (uint16_t)MOTOR_ER_INDEX_CLEAR_ALL);
    return ESP_OK;
}

esp_err_t motor_cmd_sy_reboot(uint8_t target_id, motor_cmd_t *out)
{
    return build_index_only(target_id, MOTOR_SECTION_SYSTEM, MOTOR_OBJECT_SY,
                            MOTOR_CMD_BASE_SY, false, (uint16_t)MOTOR_SY_OP_REBOOT, "SY_REBOOT", out);
}

esp_err_t motor_cmd_sy_restore_factory_defaults(uint8_t target_id, motor_cmd_t *out)
{
    return build_index_only(target_id, MOTOR_SECTION_SYSTEM, MOTOR_OBJECT_SY,
                            MOTOR_CMD_BASE_SY, false, (uint16_t)MOTOR_SY_OP_RESTORE_FACTORY_DEFAULTS,
                            "SY_RESTORE_FACTORY_DEFAULTS", out);
}

esp_err_t motor_cmd_mt_get(uint8_t target_id, bool ack_requested, motor_mt_index_t index, motor_cmd_t *out)
{
    if (!validate_mt_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_only(target_id, MOTOR_SECTION_MOTOR_DRIVER, MOTOR_OBJECT_MT,
                            MOTOR_CMD_BASE_MT, ack_requested, (uint16_t)index, "MT_GET", out);
}

esp_err_t motor_cmd_mt_set_u16(uint8_t target_id, bool ack_requested, motor_mt_index_t index, uint16_t value, motor_cmd_t *out)
{
    if (!validate_mt_index(index) || !validate_mt_value(index, value)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_u16(target_id, MOTOR_SECTION_MOTOR_DRIVER, MOTOR_OBJECT_MT,
                           MOTOR_CMD_BASE_MT, ack_requested, (uint16_t)index, value, "MT_SET_U16", out);
}

esp_err_t motor_cmd_il_get(uint8_t target_id, bool ack_requested, motor_il_index_t index, motor_cmd_t *out)
{
    if (!validate_il_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_only(target_id, MOTOR_SECTION_IO, MOTOR_OBJECT_IL,
                            MOTOR_CMD_BASE_IL, ack_requested, (uint16_t)index, "IL_GET", out);
}

esp_err_t motor_cmd_il_set_u16(uint8_t target_id, bool ack_requested, motor_il_index_t index, uint16_t value, motor_cmd_t *out)
{
    if (!validate_il_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_u16(target_id, MOTOR_SECTION_IO, MOTOR_OBJECT_IL,
                           MOTOR_CMD_BASE_IL, ack_requested, (uint16_t)index, value, "IL_SET_U16", out);
}

esp_err_t motor_cmd_mo_get(uint8_t target_id, bool ack_requested, motor_cmd_t *out)
{
    return build_no_payload(target_id, MOTOR_SECTION_MOTOR_DRIVER, MOTOR_OBJECT_MO,
                            MOTOR_CMD_BASE_MO, ack_requested, "MO_GET", out);
}

esp_err_t motor_cmd_mo_set(uint8_t target_id, bool ack_requested, motor_mo_state_t state, motor_cmd_t *out)
{
    if (!validate_mo_state(state)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_u8_payload(target_id, MOTOR_SECTION_MOTOR_DRIVER, MOTOR_OBJECT_MO,
                            MOTOR_CMD_BASE_MO, ack_requested, (uint8_t)state, "MO_SET", out);
}

esp_err_t motor_cmd_bg_begin(uint8_t target_id, bool ack_requested, motor_cmd_t *out)
{
    return build_no_payload(target_id, MOTOR_SECTION_MOTION_CONTROL, MOTOR_OBJECT_BG,
                            MOTOR_CMD_BASE_BG, ack_requested, "BG_BEGIN", out);
}

esp_err_t motor_cmd_st_stop(uint8_t target_id, bool ack_requested, motor_cmd_t *out)
{
    return build_no_payload(target_id, MOTOR_SECTION_MOTION_CONTROL, MOTOR_OBJECT_ST,
                            MOTOR_CMD_BASE_ST, ack_requested, "ST_STOP", out);
}

esp_err_t motor_cmd_pa_get(uint8_t target_id, bool ack_requested, motor_cmd_t *out)
{
    return build_no_payload(target_id, MOTOR_SECTION_MOTION_CONTROL, MOTOR_OBJECT_PA,
                            MOTOR_CMD_BASE_PA, ack_requested, "PA_GET", out);
}

esp_err_t motor_cmd_pa_set(uint8_t target_id, bool ack_requested, int32_t position, motor_cmd_t *out)
{
    return build_i32_payload(target_id, MOTOR_SECTION_MOTION_CONTROL, MOTOR_OBJECT_PA,
                             MOTOR_CMD_BASE_PA, ack_requested, position, "PA_SET", out);
}

esp_err_t motor_cmd_mp_get(uint8_t target_id, bool ack_requested, motor_mp_index_t index, motor_cmd_t *out)
{
    if (!validate_mp_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_only(target_id, MOTOR_SECTION_INTERPOLATED_MOTION, MOTOR_OBJECT_MP,
                            MOTOR_CMD_BASE_MP, ack_requested, (uint16_t)index, "MP_GET", out);
}

esp_err_t motor_cmd_mp_set_u16(uint8_t target_id, bool ack_requested, motor_mp_index_t index, uint16_t value, motor_cmd_t *out)
{
    if (!validate_mp_index(index) || !validate_mp_value(index, value)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_u16(target_id, MOTOR_SECTION_INTERPOLATED_MOTION, MOTOR_OBJECT_MP,
                           MOTOR_CMD_BASE_MP, ack_requested, (uint16_t)index, value, "MP_SET_U16", out);
}

esp_err_t motor_cmd_og_set_origin(uint8_t target_id, bool ack_requested, motor_cmd_t *out)
{
    return build_no_payload(target_id, MOTOR_SECTION_MOTION_CONTROL, MOTOR_OBJECT_OG,
                            MOTOR_CMD_BASE_OG, ack_requested, "OG_SET_ORIGIN", out);
}

esp_err_t motor_cmd_lm_get(uint8_t target_id, bool ack_requested, motor_lm_index_t index, motor_cmd_t *out)
{
    if (!validate_lm_index(index)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_only(target_id, MOTOR_SECTION_MOTION_CONTROL, MOTOR_OBJECT_LM,
                            MOTOR_CMD_BASE_LM, ack_requested, (uint16_t)index, "LM_GET", out);
}

esp_err_t motor_cmd_lm_set_i32(uint8_t target_id, bool ack_requested, motor_lm_index_t index, int32_t value, motor_cmd_t *out)
{
    if (!validate_lm_index(index) || !validate_lm_value(index, value)) {
        return ESP_ERR_INVALID_ARG;
    }

    return build_index_i32(target_id, MOTOR_SECTION_MOTION_CONTROL, MOTOR_OBJECT_LM,
                           MOTOR_CMD_BASE_LM, ack_requested, (uint16_t)index, value, "LM_SET_I32", out);
}

esp_err_t motor_cmd_sd_get(uint8_t target_id, bool ack_requested, motor_cmd_t *out)
{
    return build_no_payload(target_id, MOTOR_SECTION_MOTION_CONTROL, MOTOR_OBJECT_SD,
                            MOTOR_CMD_BASE_SD, ack_requested, "SD_GET", out);
}

esp_err_t motor_cmd_sd_set_u32(uint8_t target_id, bool ack_requested, uint32_t value, motor_cmd_t *out)
{
    return build_u32_payload(target_id, MOTOR_SECTION_MOTION_CONTROL, MOTOR_OBJECT_SD,
                             MOTOR_CMD_BASE_SD, ack_requested, value, "SD_SET_U32", out);
}

esp_err_t motor_cmd_bl_get(uint8_t target_id, bool ack_requested, motor_cmd_t *out)
{
    return build_no_payload(target_id, MOTOR_SECTION_MOTION_CONTROL, MOTOR_OBJECT_BL,
                            MOTOR_CMD_BASE_BL, ack_requested, "BL_GET", out);
}

esp_err_t motor_cmd_bl_set_u32(uint8_t target_id, bool ack_requested, uint32_t value, motor_cmd_t *out)
{
    return build_u32_payload(target_id, MOTOR_SECTION_MOTION_CONTROL, MOTOR_OBJECT_BL,
                             MOTOR_CMD_BASE_BL, ack_requested, value, "BL_SET_U32", out);
}

/*
 * TODO:
 * Add family-specific TX builders for PT, PVT, and DV once their public
 * payload contracts are finalized.
 */
