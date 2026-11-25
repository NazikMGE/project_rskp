#pragma once

#include <math.h>
#include <stdio.h>

/**
 * @file test_util.h
 * @brief Minimal unit-test harness for the host tests (no dependencies).
 *
 * A test is a `static void name(void)` function registered with RUN().
 * CHECK* macros record a failure and let the test continue, so one run
 * reports every broken expectation. tu_summary() prints the totals and
 * returns the process exit code (0 only when everything passed).
 */

static int tu_checks;       /**< checks evaluated in the whole run */
static int tu_failures;     /**< failed checks in the whole run */
static int tu_tests;        /**< tests executed */
static int tu_tests_failed; /**< tests with at least one failed check */

#define CHECK(cond)                                                            \
  do {                                                                         \
    tu_checks++;                                                               \
    if (!(cond)) {                                                             \
      tu_failures++;                                                           \
      printf("    FAILED %s:%d: %s\n", __FILE__, __LINE__, #cond);             \
    }                                                                          \
  } while (0)

#define CHECK_EQ_INT(actual, expected)                                         \
  do {                                                                         \
    long tu_a = (long)(actual);                                                \
    long tu_e = (long)(expected);                                              \
    tu_checks++;                                                               \
    if (tu_a != tu_e) {                                                        \
      tu_failures++;                                                           \
      printf("    FAILED %s:%d: %s = %ld, expected %ld\n", __FILE__, __LINE__, \
             #actual, tu_a, tu_e);                                             \
    }                                                                          \
  } while (0)

#define CHECK_NEAR(actual, expected, tol)                                      \
  do {                                                                         \
    double tu_a = (double)(actual);                                            \
    double tu_e = (double)(expected);                                          \
    tu_checks++;                                                               \
    if (!(fabs(tu_a - tu_e) <= (double)(tol))) {                               \
      tu_failures++;                                                           \
      printf("    FAILED %s:%d: %s = %.6g, expected %.6g +/- %.3g\n",          \
             __FILE__, __LINE__, #actual, tu_a, tu_e, (double)(tol));          \
    }                                                                          \
  } while (0)

#define RUN(test) tu_run(#test, test)

static inline void tu_run(const char *name, void (*fn)(void)) {
  int failures_before = tu_failures;
  int checks_before = tu_checks;
  fn();
  tu_tests++;
  int failed = tu_failures - failures_before;
  if (failed != 0) {
    tu_tests_failed++;
  }
  printf("[ %s ] %-38s %3d checks\n", failed == 0 ? "PASS" : "FAIL", name,
         tu_checks - checks_before);
}

static inline int tu_summary(const char *suite) {
  printf("%s: %d tests, %d passed, %d failed; %d checks, %d failed\n", suite,
         tu_tests, tu_tests - tu_tests_failed, tu_tests_failed, tu_checks,
         tu_failures);
  return tu_failures == 0 ? 0 : 1;
}
