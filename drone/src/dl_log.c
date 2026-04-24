/* dl_log.c */
#include "dl_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <time.h>

static dl_log_level_t s_min_level = DL_LOG_INFO;
static int            s_syslog_open = 0;

static const char *level_name(dl_log_level_t lvl) {
    switch (lvl) {
        case DL_LOG_DEBUG: return "DEBUG";
        case DL_LOG_INFO:  return "INFO";
        case DL_LOG_WARN:  return "WARN";
        case DL_LOG_ERR:   return "ERROR";
        case DL_LOG_FATAL: return "FATAL";
    }
    return "?";
}

static int level_to_syslog(dl_log_level_t lvl) {
    switch (lvl) {
        case DL_LOG_DEBUG: return LOG_DEBUG;
        case DL_LOG_INFO:  return LOG_INFO;
        case DL_LOG_WARN:  return LOG_WARNING;
        case DL_LOG_ERR:   return LOG_ERR;
        case DL_LOG_FATAL: return LOG_CRIT;
    }
    return LOG_INFO;
}

void dl_log_init(const char *ident, dl_log_level_t min_level) {
    s_min_level = min_level;
    if (ident) {
        openlog(ident, LOG_PID | LOG_NDELAY, LOG_DAEMON);
        s_syslog_open = 1;
    }
}

void dl_log_set_level(dl_log_level_t min_level) {
    s_min_level = min_level;
}

void dl_log_log(dl_log_level_t lvl, const char *fmt, ...) {
    if (lvl < s_min_level) return;

    char buf[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    /* stderr output with timestamp — useful when running outside systemd. */
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    struct tm tm;
    localtime_r(&ts.tv_sec, &tm);
    char tbuf[32];
    strftime(tbuf, sizeof(tbuf), "%Y-%m-%d %H:%M:%S", &tm);
    fprintf(stderr, "%s.%03ld %-5s %s\n",
            tbuf, ts.tv_nsec / 1000000, level_name(lvl), buf);

    if (s_syslog_open) {
        syslog(level_to_syslog(lvl), "%s", buf);
    }
}
