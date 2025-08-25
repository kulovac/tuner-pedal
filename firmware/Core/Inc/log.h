#ifndef __LOG_H
#define __LOG_H

#ifdef __cplusplus
extern "C" {
#endif

// Log level constants
#define LOG_LEVEL_DISABLED -1
#define LOG_LEVEL_ERROR 0
#define LOG_LEVEL_WARN 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_DEBUG 3
#define LOG_LEVEL_TRACE 4

#define LOG_COLOUR_ERROR "\x1b[31m"
#define LOG_COLOUR_WARN "\x1b[33m"
#define LOG_COLOUR_INFO "\x1b[32m"
#define LOG_COLOUR_DEBUG "\x1b[34m"
#define LOG_COLOUR_TRACE "\x1b[35m"
#define LOG_COLOUR_RESET "\x1b[0m"

#ifndef LOG_LEVEL
// By default, have no logging enabled
#define LOG_LEVEL LOG_LEVEL_DISABLED
#endif

typedef struct __UART_HandleTypeDef UART_HandleTypeDef;

void logger_init(UART_HandleTypeDef *uart);
void log_printf(const char *fmt, ...);

#if LOG_LEVEL >= LOG_LEVEL_ERROR
#define LOG_ERROR(...)                                                         \
  log_printf("[" LOG_COLOUR_ERROR "ERROR" LOG_COLOUR_RESET "] " __VA_ARGS__)
#else
#define LOG_ERROR(...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_WARN
#define LOG_WARN(...)                                                          \
  log_printf("[" LOG_COLOUR_WARN "WARN" LOG_COLOUR_RESET "]  " __VA_ARGS__)
#else
#define LOG_WARN(...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_INFO
#define LOG_INFO(...)                                                          \
  log_printf("[" LOG_COLOUR_INFO "INFO" LOG_COLOUR_RESET "]  " __VA_ARGS__)
#else
#define LOG_INFO(...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_DEBUG
#define LOG_DEBUG(...)                                                         \
  log_printf("[" LOG_COLOUR_DEBUG "DEBUG" LOG_COLOUR_RESET "] " __VA_ARGS__)
#else
#define LOG_DEBUG(...)
#endif

#if LOG_LEVEL >= LOG_LEVEL_TRACE
#define LOG_TRACE(...)                                                         \
  log_printf("[" LOG_COLOUR_TRACE "TRACE" LOG_COLOUR_RESET "] " __VA_ARGS__)
#else
#define LOG_TRACE(...)
#endif

#ifdef __cplusplus
}
#endif

#endif /* __LOG_H */