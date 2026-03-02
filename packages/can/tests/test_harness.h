/**
 * @file test_harness.h
 * @brief Shared test infrastructure for host-native unit tests.
 *
 * Provides the TEST() macro, pass/run counters, and summary helpers
 * so each test file can focus on test logic instead of boilerplate.
 *
 * Usage:
 *   #include "test_harness.h"
 *   ...
 *   int main(void) {
 *       printf("=== my_module tests ===\n\n");
 *       TEST(test_foo);
 *       TEST(test_bar);
 *       TEST_REPORT();
 *       TEST_EXIT();
 *   }
 */
#ifndef TEST_HARNESS_H
#define TEST_HARNESS_H

#include <assert.h>
#include <stdio.h>

static int tests_run = 0;
static int tests_passed = 0;

#define TEST(name)                        \
	do                                    \
	{                                     \
		tests_run++;                      \
		printf("  [TEST] %-55s ", #name); \
		name();                           \
		tests_passed++;                   \
		printf("PASS\n");                 \
	} while (0)

#define TEST_REPORT() printf("\n=== %d / %d tests passed ===\n", tests_passed, tests_run)

#define TEST_EXIT() return (tests_passed == tests_run) ? 0 : 1

#endif /* TEST_HARNESS_H */
