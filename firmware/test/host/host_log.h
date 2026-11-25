#pragma once

#include <stdio.h>

/**
 * @file host_log.h
 * @brief In-memory journal behind the host `esp_log.h` stub.
 *
 * Keeps the most recent log lines produced by the firmware modules so a test
 * can assert on the module's own diagnostics, and optionally mirrors every
 * line to a stream (the control example saves the detector journal that way).
 */

/** Longest stored line, including the terminating zero. */
#define HOST_LOG_LINE_MAX 200

/** Number of most recent lines kept. */
#define HOST_LOG_KEEP 64

/** Drop all stored lines and reset the line counter. */
void host_log_reset(void);

/** Mirror every following line to @p stream (NULL switches mirroring off). */
void host_log_mirror(FILE *stream);

/** Total number of lines written since the last reset. */
unsigned host_log_count(void);

/**
 * @brief Most recent stored line that contains @p needle.
 * @return Pointer to the line, or NULL when no stored line matches.
 */
const char *host_log_find_last(const char *needle);
