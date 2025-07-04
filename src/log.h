#ifndef LOG_H
#define LOG_H

#include "ch.h"

#define LOG(...) chprintf((BaseSequentialStream *)&SD2, __VA_ARGS__)

#endif // LOG_H