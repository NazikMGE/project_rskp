#pragma once

/**
 * @file esp_log.h
 * @brief Host-side stand-in for the ESP-IDF logging header.
 *
 * The DSP services (shot_detect, stability) depend on ESP-IDF only through
 * `esp_log.h`. This stub lets the unmodified firmware sources compile with a
 * desktop GCC: every ESP_LOGx call is formatted into a small in-memory
 * journal that the unit tests can inspect (e.g. to tell which confirmation
 * path a shot took) and that the control example writes to a file.
 */

#ifdef __cplusplus
extern "C" {
#endif

/** Format one log line into the host journal (see host_log.c). */
void host_log_write(char level, const char *tag, const char *fmt, ...)
    __attribute__((format(printf, 3, 4)));

#define ESP_LOGE(tag, fmt, ...) host_log_write('E', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGW(tag, fmt, ...) host_log_write('W', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGI(tag, fmt, ...) host_log_write('I', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGD(tag, fmt, ...) host_log_write('D', tag, fmt, ##__VA_ARGS__)
#define ESP_LOGV(tag, fmt, ...) host_log_write('V', tag, fmt, ##__VA_ARGS__)

#ifdef __cplusplus
}
#endif
