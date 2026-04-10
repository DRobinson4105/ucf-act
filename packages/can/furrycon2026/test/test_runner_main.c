/*
 * Responsibility:
 * Dedicated Unity runner firmware entrypoint for project test builds.
 */

#include <stdio.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "unity.h"
#include "test_log.h"

#define TEST_RUNNER_STACK_SIZE 12288U
#define TEST_RUNNER_PRIORITY   (tskIDLE_PRIORITY + 1U)

void test_twai_port_set_up(void);
void test_twai_port_tear_down(void);

static const char **s_failed_tests;
static int s_failed_test_capacity;
static int s_failed_test_count;
static bool s_failure_summary_truncated;

void setUp(void)
{
    test_twai_port_set_up();
}

void tearDown(void)
{
    if (Unity.CurrentTestFailed && !Unity.CurrentTestIgnored) {
        if (s_failed_test_count < s_failed_test_capacity) {
            s_failed_tests[s_failed_test_count++] = Unity.CurrentTestName;
        } else {
            s_failure_summary_truncated = true;
        }
    }

    test_twai_port_tear_down();
}

static void unity_runner_task(void *arg)
{
    (void)arg;

    const int test_count = unity_get_test_count();

    TEST_LOG_BANNER("Running all registered tests");
    LOG_OUTPUT("registered_tests=%d", test_count);
    s_failed_tests = calloc((size_t)test_count, sizeof(*s_failed_tests));
    s_failed_test_capacity = (s_failed_tests != NULL) ? test_count : 0;
    s_failed_test_count = 0;
    s_failure_summary_truncated = false;
    if (s_failed_tests == NULL && test_count > 0) {
        LOG_ERROR("failed test summary disabled: allocation failed for %d tests", test_count);
    }
    UNITY_BEGIN();
    unity_run_all_tests();
    UNITY_END();

    TEST_LOG_BANNER("Failed tests summary");
    if (s_failed_test_count == 0) {
        LOG_OUTPUT("none");
    } else {
        LOG_ERROR("count=%d", s_failed_test_count);
        for (int i = 0; i < s_failed_test_count; ++i) {
            printf(TEST_LOG_COLOR_ERROR "  failed[%d]: %s" TEST_LOG_COLOR_RESET "\n",
                   i + 1, s_failed_tests[i]);
        }
    }
    if (s_failure_summary_truncated) {
        LOG_ERROR("summary truncated: capacity=%d", s_failed_test_capacity);
    }

    TEST_LOG_BANNER("Test run complete");

    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}

void app_main(void)
{
    BaseType_t rc;

    TEST_LOG_BANNER("Starting Unity runner task");
    LOG_OUTPUT("stack_size=%u priority=%u", (unsigned)TEST_RUNNER_STACK_SIZE, (unsigned)TEST_RUNNER_PRIORITY);

    rc = xTaskCreate(unity_runner_task,
                     "unity_runner",
                     TEST_RUNNER_STACK_SIZE,
                     NULL,
                     TEST_RUNNER_PRIORITY,
                     NULL);
    if (rc != pdPASS) {
        LOG_ERROR("failed to create Unity runner task");
        while (1) {
            vTaskDelay(portMAX_DELAY);
        }
    }

    vTaskDelete(NULL);
}
