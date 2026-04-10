/*
 * Responsibility:
 * Human-readable rendering and logging for motor_diag descriptions.
 */

#include "motor_diag_internal.h"

#include <inttypes.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"

static const char *TAG = "motor_diag";

static const char *cmd_kind_suffix(motor_diag_cmd_kind_t kind)
{
    switch (kind) {
        case MOTOR_DIAG_CMD_KIND_GET:
            return "get";
        case MOTOR_DIAG_CMD_KIND_SET:
            return "set";
        case MOTOR_DIAG_CMD_KIND_ACTION:
            return "action";
        default:
            return NULL;
    }
}

static size_t appendf(char *buf, size_t cap, size_t used, const char *fmt, ...)
{
    int written;
    va_list ap;

    if (buf == NULL || cap == 0U || used >= cap) {
        return used;
    }

    va_start(ap, fmt);
    written = vsnprintf(buf + used, cap - used, fmt, ap);
    va_end(ap);

    if (written < 0) {
        return used;
    }

    if ((size_t)written >= cap - used) {
        return cap - 1U;
    }

    return used + (size_t)written;
}

static size_t append_value(char *buf, size_t cap, size_t used, const motor_diag_value_desc_t *value)
{
    uint8_t i;

    if (value == NULL || !value->has_value) {
        if (value != NULL && value->kind == MOTOR_DIAG_VALUE_KIND_RAW_BYTES && value->raw_len > 0U) {
            used = appendf(buf, cap, used, "raw=");
            for (i = 0; i < value->raw_len; ++i) {
                used = appendf(buf, cap, used, "%s0x%02X", i == 0U ? "" : " ", value->raw[i]);
            }
        }
        return used;
    }

    switch (value->kind) {
        case MOTOR_DIAG_VALUE_KIND_ENUM:
        case MOTOR_DIAG_VALUE_KIND_TEXT:
            if (value->text != NULL) {
                used = appendf(buf, cap, used, "%s", value->text);
            } else {
                used = appendf(buf, cap, used, "%" PRId64,
                               value->has_raw_number ? value->raw_number : value->number);
            }
            break;
        case MOTOR_DIAG_VALUE_KIND_U8:
        case MOTOR_DIAG_VALUE_KIND_U16:
        case MOTOR_DIAG_VALUE_KIND_U32:
        case MOTOR_DIAG_VALUE_KIND_I32:
            used = appendf(buf, cap, used, "%" PRId64,
                           value->has_raw_number ? value->raw_number : value->number);
            break;
        default:
            break;
    }

    if (value->units != NULL) {
        used = appendf(buf, cap, used, " %s", value->units);
    }

    return used;
}

size_t motor_diag_format_cmd(char *buf, size_t cap, const motor_diag_cmd_desc_t *desc)
{
    size_t used = 0U;

    if (buf == NULL || cap == 0U || desc == NULL) {
        return 0U;
    }

    buf[0] = '\0';
    used = appendf(buf, cap, used, "TX node=%u ", desc->node_id);
    used = appendf(buf, cap, used, "%s", desc->family_symbol != NULL ? desc->family_symbol : "??");
    if (desc->has_index) {
        used = appendf(buf, cap, used, "[%u]", desc->index);
    }

    switch (desc->kind) {
        case MOTOR_DIAG_CMD_KIND_GET:
            used = appendf(buf, cap, used, " GET");
            break;
        case MOTOR_DIAG_CMD_KIND_SET:
            used = appendf(buf, cap, used, " SET");
            break;
        case MOTOR_DIAG_CMD_KIND_ACTION:
            used = appendf(buf, cap, used, " ACTION");
            break;
        default:
            break;
    }

    if (desc->semantic_name != NULL) {
        used = appendf(buf, cap, used, " (%s)", desc->semantic_name);
    }

    if (desc->kind == MOTOR_DIAG_CMD_KIND_SET && (desc->value.has_value || desc->value.raw_len > 0U)) {
        used = appendf(buf, cap, used, " = ");
        used = append_value(buf, cap, used, &desc->value);
    }

    if (desc->note != NULL) {
        used = appendf(buf, cap, used, " (%s)", desc->note);
    }

    return used;
}

size_t motor_diag_format_rx(char *buf, size_t cap, const motor_diag_rx_desc_t *desc)
{
    size_t used = 0U;
    uint8_t i;

    if (buf == NULL || cap == 0U || desc == NULL) {
        return 0U;
    }

    buf[0] = '\0';
    used = appendf(buf, cap, used, "RX node=%u ", desc->node_id);

    switch (desc->kind) {
        case MOTOR_DIAG_RX_KIND_ACK:
        {
            const char *semantic_kind_text = cmd_kind_suffix(desc->semantic_cmd_kind);
            used = appendf(buf, cap, used, "ACK ");
            used = appendf(buf, cap, used, "%s", desc->family_symbol != NULL ? desc->family_symbol : "??");
            if (desc->has_index) {
                used = appendf(buf, cap, used, "[%u]", desc->index);
            }
            if (desc->has_semantic_lineage && desc->semantic_family_symbol != NULL) {
                used = appendf(buf, cap, used, " (for %s", desc->semantic_family_symbol);
                if (desc->has_semantic_index) {
                    used = appendf(buf, cap, used, "[%u]", desc->semantic_index);
                }
                if (semantic_kind_text != NULL) {
                    used = appendf(buf, cap, used, " %s", semantic_kind_text);
                }
                used = appendf(buf, cap, used, ")");
            }
            if (desc->semantic_name != NULL) {
                used = appendf(buf, cap, used, " %s", desc->semantic_name);
            }
            if (desc->value.has_value) {
                used = appendf(buf, cap, used, " = ");
                used = append_value(buf, cap, used, &desc->value);
            } else if (desc->note != NULL) {
                used = appendf(buf, cap, used, " raw=");
                for (i = 0; i < desc->raw_len; ++i) {
                    used = appendf(buf, cap, used, "%s0x%02X", i == 0U ? "" : " ", desc->raw[i]);
                }
            }
            break;
        }
        case MOTOR_DIAG_RX_KIND_ERROR:
            used = appendf(buf, cap, used, "ERROR");
            if (desc->has_error_code) {
                used = appendf(buf, cap, used, " code=0x%02X", desc->error_code);
            }
            if (desc->error_name != NULL) {
                used = appendf(buf, cap, used, " (%s)", desc->error_name);
            }
            if (desc->has_related_family) {
                used = appendf(buf, cap, used, " related=%s", desc->related_family_symbol);
                if (desc->has_related_index) {
                    used = appendf(buf, cap, used, "[%u]", desc->related_index);
                }
            }
            break;
        case MOTOR_DIAG_RX_KIND_ER_RESPONSE:
            used = appendf(buf, cap, used, "ER");
            if (desc->has_index) {
                used = appendf(buf, cap, used, "[%u]", desc->index);
            }
            used = appendf(buf, cap, used, " RESPONSE");
            if (desc->semantic_name != NULL) {
                used = appendf(buf, cap, used, " (%s)", desc->semantic_name);
            }
            if (desc->has_error_code) {
                used = appendf(buf, cap, used, " error=0x%02X", desc->error_code);
            }
            if (desc->error_name != NULL) {
                used = appendf(buf, cap, used, " (%s)", desc->error_name);
            }
            if (desc->has_related_family) {
                used = appendf(buf, cap, used, " related=%s", desc->related_family_symbol);
                if (desc->has_related_index) {
                    used = appendf(buf, cap, used, "[%u]", desc->related_index);
                }
            } else if (desc->has_related_cw) {
                used = appendf(buf, cap, used, " related_cw=0x%02X", desc->related_cw);
                if (desc->has_related_index) {
                    used = appendf(buf, cap, used, " subindex=%u", desc->related_index);
                }
            }
            break;
        case MOTOR_DIAG_RX_KIND_RT_ALARM:
            used = appendf(buf, cap, used, "RT alarm");
            break;
        case MOTOR_DIAG_RX_KIND_RT_STATUS:
            used = appendf(buf, cap, used, "RT status");
            break;
        case MOTOR_DIAG_RX_KIND_RT_UNKNOWN:
            used = appendf(buf, cap, used, "RT raw");
            break;
        default:
            used = appendf(buf, cap, used, "raw");
            break;
    }

    if ((desc->kind == MOTOR_DIAG_RX_KIND_RT_ALARM || desc->kind == MOTOR_DIAG_RX_KIND_RT_STATUS) &&
        desc->semantic_name != NULL) {
        used = appendf(buf, cap, used, " %s", desc->semantic_name);
        if (desc->value.has_value) {
            used = appendf(buf, cap, used, ", current position = ");
            used = append_value(buf, cap, used, &desc->value);
        }
    }

    if ((desc->kind == MOTOR_DIAG_RX_KIND_RT_UNKNOWN || desc->kind == MOTOR_DIAG_RX_KIND_UNCLASSIFIED) &&
        desc->raw_len > 0U) {
        for (i = 0; i < desc->raw_len; ++i) {
            used = appendf(buf, cap, used, " d%u=0x%02X", i, desc->raw[i]);
        }
    }

    if (desc->note != NULL) {
        used = appendf(buf, cap, used, " (%s)", desc->note);
    }

    return used;
}

void motor_diag_log_cmd(const motor_cmd_t *cmd)
{
    char line[160];
    motor_diag_cmd_desc_t desc;

    if (!motor_diag_describe_cmd(cmd, &desc)) {
        return;
    }

    (void)motor_diag_format_cmd(line, sizeof(line), &desc);
    ESP_LOGI(TAG, "%s", line);
}

void motor_diag_log_rx(const motor_rx_t *rx)
{
    char line[160];
    motor_diag_rx_desc_t desc;

    if (!motor_diag_describe_rx(rx, &desc)) {
        return;
    }

    (void)motor_diag_format_rx(line, sizeof(line), &desc);
    ESP_LOGI(TAG, "%s", line);
}
