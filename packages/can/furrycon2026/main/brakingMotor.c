#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "app_twai_config.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "motor_setup.h"
#include "twai_port.h"

static const char *TAG = "brakingMotor";

/* App Policy */
#define SETUP_MOTOR_NODE_ID 0U
#define SETUP_TWAI_BITRATE  500000U
#define SETUP_RX_TIMEOUT_MS 1000U
#define SETUP_TASK_STACK_SIZE 8192U
#define SETUP_TASK_PRIORITY   (tskIDLE_PRIORITY + 1U)

/* Plan Step Helpers */
#define READ_STEP(name_, object_, index_) \
    { \
        .name = (name_), \
        .kind = MOTOR_SETUP_STEP_KIND_READ_ONLY, \
        .exec_action = { \
            .role = MOTOR_EXEC_ROLE_BRAKE, \
            .node_id = SETUP_MOTOR_NODE_ID, \
            .operation = MOTOR_EXEC_OPERATION_GET, \
            .object = (object_), \
            .ack_requested = true, \
            .has_index = true, \
            .index = (uint16_t)(index_), \
        }, \
        .compare_kind = MOTOR_SETUP_COMPARE_NONE, \
    }

/* Setup Plans */
static const motor_setup_step_t s_setup_steps[] = {
    READ_STEP("PP GET CAN bitrate", MOTOR_OBJECT_PP, MOTOR_PP_INDEX_BITRATE),
};

static const motor_setup_plan_t s_setup_plan = {
    .name = "brake motor setup",
    .steps = s_setup_steps,
    .step_count = sizeof(s_setup_steps) / sizeof(s_setup_steps[0]),
};

/* TWAI Lifecycle */
static twai_port_cfg_t build_twai_cfg(uint32_t bitrate)
{
    twai_port_cfg_t cfg = app_make_twai_port_cfg();

    cfg.bitrate = bitrate;
    cfg.mode = TWAI_PORT_MODE_NORMAL;
    cfg.filter_mode = TWAI_PORT_FILTER_EXT_ONLY;
    cfg.default_timeout_ticks = pdMS_TO_TICKS(SETUP_RX_TIMEOUT_MS);

    return cfg;
}

static esp_err_t start_twai_with_bitrate(uint32_t bitrate)
{
    twai_port_cfg_t cfg = build_twai_cfg(bitrate);

    ESP_LOGI(TAG,
             "Initializing TWAI tx_gpio=%d rx_gpio=%d bitrate=%" PRIu32 " timeout_ms=%u",
             cfg.tx_gpio,
             cfg.rx_gpio,
             cfg.bitrate,
             (unsigned)SETUP_RX_TIMEOUT_MS);

    esp_err_t err = twai_port_init(&cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_port_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = twai_port_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_port_start failed: %s", esp_err_to_name(err));
        (void)twai_port_deinit();
        return err;
    }

    ESP_LOGI(TAG, "TWAI ready");
    return ESP_OK;
}

static void setup_twai_shutdown_if_initialized(void)
{
    if (twai_port_is_initialized()) {
        (void)twai_port_deinit();
    }
}

/* Plan Execution */
static const motor_exec_result_t *step_terminal_result(const motor_setup_step_result_t *step)
{
    if (step == NULL) {
        return NULL;
    }

    if (step->verification_attempted) {
        return &step->verification_result;
    }

    return &step->execution_result;
}

static void log_step_history(const motor_setup_result_t *result,
                             const motor_setup_step_result_t *history)
{
    size_t i;

    if (result == NULL || history == NULL) {
        return;
    }

    for (i = 0; i < result->step_results_written; ++i) {
        const motor_setup_step_result_t *step = &history[i];
        const motor_exec_result_t *terminal = step_terminal_result(step);

        if (terminal == NULL) {
            continue;
        }

        if (step->failure_reason == MOTOR_SETUP_FAILURE_NONE) {
            ESP_LOGI(TAG,
                     "setup step=%u name=%s summary=%s",
                     (unsigned)step->step_index,
                     step->step_name,
                     terminal->completion_summary);
            continue;
        }

        ESP_LOGE(TAG,
                 "setup step=%u name=%s failure_reason=%d summary=%s",
                 (unsigned)step->step_index,
                 step->step_name,
                 (int)step->failure_reason,
                 terminal->completion_summary);
    }
}

static esp_err_t run_named_plan(const motor_setup_plan_t *plan)
{
    const size_t step_count = plan != NULL ? plan->step_count : 0U;
    const size_t history_size = step_count * sizeof(motor_setup_step_result_t);
    motor_setup_step_result_t *history = NULL;
    motor_setup_run_cfg_t cfg;
    motor_setup_result_t result;
    motor_setup_log_cfg_t log_cfg = motor_setup_default_log_cfg();
    esp_err_t err;

    memset(&result, 0, sizeof(result));
    cfg = motor_setup_default_run_cfg();

    if (plan == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    ESP_LOGI(TAG,
             "Preparing plan history name=%s step_count=%u entry_size=%u total_bytes=%u stack_hwm_words=%u",
             plan->name,
             (unsigned)step_count,
             (unsigned)sizeof(motor_setup_step_result_t),
             (unsigned)history_size,
             (unsigned)uxTaskGetStackHighWaterMark(NULL));

    history = calloc(step_count, sizeof(*history));
    if (history == NULL) {
        ESP_LOGE(TAG,
                 "Failed to allocate plan history name=%s bytes=%u stack_hwm_words=%u",
                 plan->name,
                 (unsigned)history_size,
                 (unsigned)uxTaskGetStackHighWaterMark(NULL));
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI(TAG,
             "Allocated plan history name=%s bytes=%u ptr=%p stack_hwm_words=%u",
             plan->name,
             (unsigned)history_size,
             (void *)history,
             (unsigned)uxTaskGetStackHighWaterMark(NULL));

    cfg.step_results = history;
    cfg.step_results_capacity = step_count;

    err = motor_setup_set_log_cfg(&log_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_setup_set_log_cfg failed: %s", esp_err_to_name(err));
        free(history);
        return err;
    }

    err = motor_setup_validate_plan(plan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "plan validation failed name=%s rc=%s", plan->name, esp_err_to_name(err));
        free(history);
        return err;
    }

    ESP_LOGI(TAG,
             "Starting plan name=%s steps=%u node=%u twai_bitrate=%" PRIu32 " stack_hwm_words=%u",
             plan->name,
             (unsigned)plan->step_count,
             (unsigned)SETUP_MOTOR_NODE_ID,
             (uint32_t)SETUP_TWAI_BITRATE,
             (unsigned)uxTaskGetStackHighWaterMark(NULL));

    err = motor_setup_run_plan(plan, &cfg, &result);
    ESP_LOGI(TAG,
             "Plan returned name=%s rc=%s status=%d completed=%u/%u stack_hwm_words=%u",
             plan->name,
             esp_err_to_name(err),
             (int)result.status,
             (unsigned)result.completed_steps,
             (unsigned)result.total_steps,
             (unsigned)uxTaskGetStackHighWaterMark(NULL));
    log_step_history(&result, history);
    free(history);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "motor_setup_run_plan failed name=%s rc=%s", plan->name, esp_err_to_name(err));
        return err;
    }

    if (result.status == MOTOR_SETUP_STATUS_SUCCESS) {
        ESP_LOGI(TAG,
                 "Plan succeeded name=%s completed=%u/%u",
                 plan->name,
                 (unsigned)result.completed_steps,
                 (unsigned)result.total_steps);
        return ESP_OK;
    }

    ESP_LOGE(TAG,
             "Plan failed name=%s step=%u step_name=%s reason=%d completed=%u/%u",
             plan->name,
             (unsigned)result.failing_step_index,
             result.failing_step_name != NULL ? result.failing_step_name : "<null>",
             (int)result.failure_reason,
             (unsigned)result.completed_steps,
             (unsigned)result.total_steps);
    return ESP_FAIL;
}

/* Task Flow */
static esp_err_t run_setup_flow(void)
{
    esp_err_t err = start_twai_with_bitrate(SETUP_TWAI_BITRATE);

    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "Setup task TWAI initialized stack_hwm_words=%u",
                 (unsigned)uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(pdMS_TO_TICKS(100));
        err = run_named_plan(&s_setup_plan);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    setup_twai_shutdown_if_initialized();
    return err;
}

static void setup_task(void *arg)
{
    esp_err_t err;

    (void)arg;

    ESP_LOGI(TAG,
             "Setup task started stack_size=%u stack_hwm_words=%u",
             (unsigned)SETUP_TASK_STACK_SIZE,
             (unsigned)uxTaskGetStackHighWaterMark(NULL));

    err = run_setup_flow();

    ESP_LOGI(TAG,
             "Setup task completed rc=%s stack_hwm_words=%u",
             esp_err_to_name(err),
             (unsigned)uxTaskGetStackHighWaterMark(NULL));

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Application finished with error: %s", esp_err_to_name(err));
    }

    vTaskDelete(NULL);
}

void app_main(void)
{
    BaseType_t rc;

    ESP_LOGI(TAG,
             "Creating setup task stack_size=%u priority=%u main_stack_hwm_words=%u",
             (unsigned)SETUP_TASK_STACK_SIZE,
             (unsigned)SETUP_TASK_PRIORITY,
             (unsigned)uxTaskGetStackHighWaterMark(NULL));

    rc = xTaskCreate(setup_task,
                     "setup_task",
                     SETUP_TASK_STACK_SIZE,
                     NULL,
                     SETUP_TASK_PRIORITY,
                     NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "Failed to create setup task");
        return;
    }

    ESP_LOGI(TAG, "Setup task created");
}
