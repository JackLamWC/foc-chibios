#ifndef LOG_H
#define LOG_H

#include "ch.h"

#define LOG(tag,...) chprintf((BaseSequentialStream *)&SD2, "[%s] " __VA_ARGS__, tag)

#endif // LOG_H