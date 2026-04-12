#include <inttypes.h>
#include <stdlib.h>
#include <string.h>

#include "app_twai_config.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "motor_exec.h"
#include "motor_setup.h"
#include "serial_input.h"
#include "twai_port.h"

static const char *TAG = "brakingMotor";

/* App Policy */
#define SETUP_MOTOR_NODE_ID 6U
#define SETUP_RX_TIMEOUT_MS 1000U
#define SETUP_TASK_STACK_SIZE 8192U
#define SETUP_TASK_PRIORITY   (tskIDLE_PRIORITY + 1U)
#define PT_LOOP_PERIOD_MS     500U
#define BRAKING_PT_PULSES_PER_STEP 10000
#define PT_POSITION_CLAMP_PULSES 1000

typedef struct {
    SemaphoreHandle_t done_sem;
    motor_exec_result_t result;
    bool completed;
} brake_pt_wait_t;

static int32_t braking_to_pt_position(uint8_t braking)
{
    return ((int32_t)braking - 3) * BRAKING_PT_PULSES_PER_STEP;
}

static int32_t clamp_pt_step(int32_t last_sent_pt, int32_t desired_pt)
{
    const int32_t delta = desired_pt - last_sent_pt;

    if (delta > PT_POSITION_CLAMP_PULSES) {
        return last_sent_pt + PT_POSITION_CLAMP_PULSES;
    }

    if (delta < -PT_POSITION_CLAMP_PULSES) {
        return last_sent_pt - PT_POSITION_CLAMP_PULSES;
    }

    return desired_pt;
}

/* TWAI Lifecycle */
static const char *twai_mode_name(twai_port_mode_t mode)
{
    switch (mode) {
        case TWAI_PORT_MODE_LISTEN_ONLY:
            return "listen-only";
        case TWAI_PORT_MODE_NO_ACK:
            return "no-ack";
        case TWAI_PORT_MODE_NORMAL:
        default:
            return "normal";
    }
}

static twai_port_cfg_t build_twai_cfg(void)
{
    twai_port_cfg_t cfg = app_make_twai_port_cfg();

    cfg.filter_mode = TWAI_PORT_FILTER_EXT_ONLY;
    cfg.default_timeout_ticks = pdMS_TO_TICKS(SETUP_RX_TIMEOUT_MS);

    return cfg;
}

static esp_err_t start_twai(void)
{
    twai_port_cfg_t cfg = build_twai_cfg();

    ESP_LOGI(TAG,
             "Initializing TWAI tx_gpio=%d rx_gpio=%d bitrate=%" PRIu32 " mode=%s timeout_ms=%u",
             cfg.tx_gpio,
             cfg.rx_gpio,
             cfg.bitrate,
             twai_mode_name(cfg.mode),
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

static void brake_pt_completion_cb(const motor_exec_result_t *result, void *ctx)
{
    brake_pt_wait_t *wait = (brake_pt_wait_t *)ctx;

    if (wait == NULL || result == NULL) {
        return;
    }

    wait->result = *result;
    wait->completed = true;
    if (wait->done_sem != NULL) {
        xSemaphoreGive(wait->done_sem);
    }
}

static esp_err_t run_brake_pt_command(int32_t queued_position)
{
    brake_pt_wait_t wait = {0};
    motor_exec_submit_opts_t opts = {
        .on_complete = brake_pt_completion_cb,
        .ctx = &wait,
        .timing = motor_exec_default_timing(),
    };
    motor_exec_submit_result_t submit;

    wait.done_sem = xSemaphoreCreateBinary();
    if (wait.done_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }

    submit = motor_exec_brake_pt_set(SETUP_MOTOR_NODE_ID, true, 0U, queued_position, &opts);
    if (!submit.accepted) {
        vSemaphoreDelete(wait.done_sem);
        return submit.err != ESP_OK ? submit.err : ESP_FAIL;
    }

    if (!(xSemaphoreTake(wait.done_sem,
                         opts.timing.timeout_ticks + opts.timing.no_ack_grace_ticks + pdMS_TO_TICKS(25)) ==
          pdTRUE &&
          wait.completed)) {
        vSemaphoreDelete(wait.done_sem);
        return ESP_ERR_TIMEOUT;
    }

    vSemaphoreDelete(wait.done_sem);
    return wait.result.status == MOTOR_EXEC_STATUS_SUCCESS ? ESP_OK : ESP_FAIL;
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
             app_make_twai_port_cfg().bitrate,
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
    esp_err_t err = start_twai();

    if (err == ESP_OK) {
        ESP_LOGI(TAG,
                 "Setup task TWAI initialized stack_hwm_words=%u",
                 (unsigned)uxTaskGetStackHighWaterMark(NULL));
        vTaskDelay(pdMS_TO_TICKS(100));
        err = run_named_plan(motor_setup_brake_pt_pv_demo_plan());
        if (err == ESP_OK) {
            TickType_t next_plan_tick = xTaskGetTickCount();
            const TickType_t period_ticks = pdMS_TO_TICKS(PT_LOOP_PERIOD_MS);
            serial_input_frame_t frame = {0};
            int32_t last_sent_pt = 0;
            bool frame_valid = false;
            bool frame_updated_this_cycle = false;

            err = serial_input_init();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "serial_input_init failed: %s", esp_err_to_name(err));
            } else {
                while (1) {
                    const TickType_t now = xTaskGetTickCount();
                    const TickType_t timeout_ticks = (now < next_plan_tick) ? (next_plan_tick - now) : 0U;

                    frame_updated_this_cycle = false;
                    err = serial_input_read(&frame, timeout_ticks);
                    if (err == ESP_OK) {
                        frame_valid = true;
                        frame_updated_this_cycle = true;
                        ESP_LOGI(TAG,
                                 "serial seq=%u throttle=%u steering=%u braking=%u",
                                 (unsigned)frame.seq,
                                 (unsigned)frame.throttle,
                                 (unsigned)frame.steering,
                                 (unsigned)frame.braking);
                    } else if (err != ESP_ERR_TIMEOUT) {
                        ESP_LOGE(TAG, "serial_input_read failed: %s", esp_err_to_name(err));
                    }

                    if (xTaskGetTickCount() < next_plan_tick) {
                        continue;
                    }

                    if (frame_valid) {
                        const int32_t desired_pt = braking_to_pt_position(frame.braking);
                        const int32_t next_sent_pt = clamp_pt_step(last_sent_pt, desired_pt);

                        ESP_LOGI(TAG,
                                 "pt uart/frame state=%s seq=%u throttle=%u steering=%u braking=%u desired_pt=%ld next_pt=%ld last_pt=%ld",
                                 frame_updated_this_cycle ? "updated" : "stale",
                                 (unsigned)frame.seq,
                                 (unsigned)frame.throttle,
                                 (unsigned)frame.steering,
                                 (unsigned)frame.braking,
                                 (long)desired_pt,
                                 (long)next_sent_pt,
                                 (long)last_sent_pt);

                        err = run_brake_pt_command(next_sent_pt);
                        if (err == ESP_OK) {
                            last_sent_pt = next_sent_pt;
                        }
                    } else {
                        const int32_t desired_pt = braking_to_pt_position(0U);
                        const int32_t next_sent_pt = clamp_pt_step(last_sent_pt, desired_pt);

                        ESP_LOGW(TAG,
                                 "pt uart/frame state=missing using default braking=%u desired_pt=%ld next_pt=%ld last_pt=%ld",
                                 0U,
                                 (long)desired_pt,
                                 (long)next_sent_pt,
                                 (long)last_sent_pt);

                        err = run_brake_pt_command(next_sent_pt);
                        if (err == ESP_OK) {
                            last_sent_pt = next_sent_pt;
                        }
                    }

                    if (err != ESP_OK) {
                        break;
                    }

                    next_plan_tick += period_ticks;
                    if (xTaskGetTickCount() > next_plan_tick) {
                        next_plan_tick = xTaskGetTickCount() + period_ticks;
                    }
                }
            }
        }
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
