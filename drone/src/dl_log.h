/* dl_log.h — tiny level-based logger. stderr by default; syslog when
 * running under systemd/OpenRC (set via dl_log_init).
 */
#pragma once

#include <stdarg.h>

typedef enum {
    DL_LOG_DEBUG = 0,
    DL_LOG_INFO  = 1,
    DL_LOG_WARN  = 2,
    DL_LOG_ERR   = 3,
    DL_LOG_FATAL = 4,
} dl_log_level_t;

/* Initialise. `ident` is syslog's program-name (NULL => skip syslog).
 * `min_level` filters messages below this level. */
void dl_log_init(const char *ident, dl_log_level_t min_level);

void dl_log_set_level(dl_log_level_t min_level);

void dl_log_log(dl_log_level_t lvl, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

#define dl_log_debug(...) dl_log_log(DL_LOG_DEBUG, __VA_ARGS__)
#define dl_log_info(...)  dl_log_log(DL_LOG_INFO,  __VA_ARGS__)
#define dl_log_warn(...)  dl_log_log(DL_LOG_WARN,  __VA_ARGS__)
#define dl_log_err(...)   dl_log_log(DL_LOG_ERR,   __VA_ARGS__)
#define dl_log_fatal(...) dl_log_log(DL_LOG_FATAL, __VA_ARGS__)
