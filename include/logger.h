#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

#define LOG_LEVEL_NONE  0
#define LOG_LEVEL_ERROR 1
#define LOG_LEVEL_WARN  2
#define LOG_LEVEL_INFO  3
#define LOG_LEVEL_DEBUG 4

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_DEBUG  // Default log level
#endif

#define LOG_PRINT(level, tag, fmt, ...) \
  Serial.printf("[%s][%s] " fmt "\n", level, tag, ##__VA_ARGS__)

#if LOG_LEVEL >= LOG_LEVEL_ERROR
  #define LOGE(tag, fmt, ...) LOG_PRINT("ERROR", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGE(tag, fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
  #define LOGW(tag, fmt, ...) LOG_PRINT("WARN", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGW(tag, fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
  #define LOGI(tag, fmt, ...) LOG_PRINT("INFO", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGI(tag, fmt, ...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
  #define LOGD(tag, fmt, ...) LOG_PRINT("DEBUG", tag, fmt, ##__VA_ARGS__)
#else
  #define LOGD(tag, fmt, ...)
#endif

#endif // LOGGER_H
