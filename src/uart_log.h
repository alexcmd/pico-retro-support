#pragma once
#include <stdio.h>
#include "pico/time.h"

typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR,
} log_level_t;

#ifndef LOG_MIN_LEVEL
#define LOG_MIN_LEVEL LOG_LEVEL_DEBUG
#endif

static const char *const _log_prefix[] = { "DBG", "INF", "WRN", "ERR" };

#define _LOG(lvl, fmt, ...)                                                  \
    do {                                                                     \
        if ((lvl) >= LOG_MIN_LEVEL) {                                        \
            printf("[%010llu][%s] " fmt "\r\n",                              \
                   (unsigned long long)to_us_since_boot(get_absolute_time()),\
                   _log_prefix[(lvl)], ##__VA_ARGS__);                       \
        }                                                                    \
    } while (0)

#define LOG_DEBUG(fmt, ...)  _LOG(LOG_LEVEL_DEBUG, fmt, ##__VA_ARGS__)
#define LOG_INFO(fmt, ...)   _LOG(LOG_LEVEL_INFO,  fmt, ##__VA_ARGS__)
#define LOG_WARN(fmt, ...)   _LOG(LOG_LEVEL_WARN,  fmt, ##__VA_ARGS__)
#define LOG_ERROR(fmt, ...)  _LOG(LOG_LEVEL_ERROR, fmt, ##__VA_ARGS__)
