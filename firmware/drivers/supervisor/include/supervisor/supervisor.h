#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @file supervisor.h
 * @brief Fault watchdog with a bounded reboot budget.
 *
 * Recovers the device from transient faults — a wedged BNO08x, a failed
 * sensor bring-up — by rebooting, but caps the number of consecutive
 * fault reboots so a permanent hardware fault cannot trap the device in
 * a reboot loop. The counter lives in RTC memory and survives
 * `esp_restart()`; a cold power-on or a confirmed healthy run clears it.
 */

/** Consecutive fault reboots tolerated before the supervisor gives up. */
#define SUPERVISOR_MAX_FAULT_REBOOTS 3

/**
 * @brief Initialise the supervisor. Call once, early in `app_main`.
 *
 * Resets the budget on a cold power-on and charges an unaccounted
 * panic/watchdog reset to it, so a crash loop is bounded as well.
 */
void supervisor_init(void);

/**
 * @brief True once the fault-reboot budget is spent.
 *
 * When true the caller must stop rebooting and bring the device up in a
 * degraded-but-stable state instead.
 */
bool supervisor_reboot_budget_exhausted(void);

/**
 * @brief Charge a fault to the budget and reboot if it still allows.
 *
 * With budget left: logs @p reason, flushes the console, and resets the
 * chip — does not return. With the budget spent: logs a fatal line and
 * returns, leaving the caller to enter a safe state.
 *
 * @param reason Short human-readable fault description for the log.
 */
void supervisor_fault_reboot(const char *reason);

/**
 * @brief Clear the fault-reboot budget after a confirmed healthy run.
 *
 * Called once the watched subsystem has been stable long enough that
 * any earlier fault counts as transient and recovered.
 */
void supervisor_mark_healthy(void);

#ifdef __cplusplus
}
#endif
