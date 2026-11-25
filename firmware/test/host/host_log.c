#include "host_log.h"

#include <stdarg.h>
#include <string.h>

#include "esp_log.h"

static char s_lines[HOST_LOG_KEEP][HOST_LOG_LINE_MAX];
static unsigned s_count;
static FILE *s_mirror;

void host_log_reset(void) {
  memset(s_lines, 0, sizeof(s_lines));
  s_count = 0;
}

void host_log_mirror(FILE *stream) { s_mirror = stream; }

unsigned host_log_count(void) { return s_count; }

void host_log_write(char level, const char *tag, const char *fmt, ...) {
  char *line = s_lines[s_count % HOST_LOG_KEEP];
  int n = snprintf(line, HOST_LOG_LINE_MAX, "%c (%s) ", level, tag);
  if (n < 0 || n >= HOST_LOG_LINE_MAX) {
    n = 0;
  }
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(line + n, (size_t)(HOST_LOG_LINE_MAX - n), fmt, ap);
  va_end(ap);
  s_count++;
  if (s_mirror != NULL) {
    fprintf(s_mirror, "%s\n", line);
  }
}

const char *host_log_find_last(const char *needle) {
  unsigned stored = s_count < HOST_LOG_KEEP ? s_count : HOST_LOG_KEEP;
  for (unsigned i = 0; i < stored; i++) {
    const char *line = s_lines[(s_count - 1u - i) % HOST_LOG_KEEP];
    if (strstr(line, needle) != NULL) {
      return line;
    }
  }
  return NULL;
}
