/*
 * Responsibility:
 * Shared test logging helpers for consistent local Unity output.
 */

#pragma once

#include <stdio.h>

#define TEST_LOG_COLOR_SECTION "\x1b[1;34m"
#define TEST_LOG_COLOR_INPUT   "\x1b[0;36m"
#define TEST_LOG_COLOR_OUTPUT  "\x1b[0;32m"
#define TEST_LOG_COLOR_ERROR   "\x1b[0;31m"
#define TEST_LOG_COLOR_RESET   "\x1b[0m"

#define TEST_LOG_BANNER(fmt, ...) \
    printf(TEST_LOG_COLOR_SECTION "\n[TEST] " fmt TEST_LOG_COLOR_RESET "\n", ##__VA_ARGS__)

#define LOG_SECTION(fmt, ...) \
    printf(TEST_LOG_COLOR_SECTION "\n[SECTION] " fmt TEST_LOG_COLOR_RESET "\n", ##__VA_ARGS__)

#define LOG_INPUT(fmt, ...) \
    printf(TEST_LOG_COLOR_INPUT "  input: " fmt TEST_LOG_COLOR_RESET "\n", ##__VA_ARGS__)

#define LOG_OUTPUT(fmt, ...) \
    printf(TEST_LOG_COLOR_OUTPUT "  output: " fmt TEST_LOG_COLOR_RESET "\n", ##__VA_ARGS__)

#define LOG_ERROR(fmt, ...) \
    printf(TEST_LOG_COLOR_ERROR "  error: " fmt TEST_LOG_COLOR_RESET "\n", ##__VA_ARGS__)
