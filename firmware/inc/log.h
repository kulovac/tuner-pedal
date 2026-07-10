#ifndef LOG_H
#define LOG_H

#define LOG_BUFFER_SIZE 128

#define LOG_LEVEL_TRACE 0
#define LOG_LEVEL_DEBUG 1
#define LOG_LEVEL_INFO 2
#define LOG_LEVEL_WARN 3
#define LOG_LEVEL_ERROR 4
#define LOG_LEVEL_NONE 5

#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_LEVEL_INFO
#endif

#define LOG_CLR_TRACE "\x1b[90m" // Gray
#define LOG_CLR_DEBUG "\x1b[36m" // Cyan
#define LOG_CLR_INFO "\x1b[32m"  // Green
#define LOG_CLR_WARN "\x1b[33m"  // Yellow
#define LOG_CLR_ERROR "\x1b[31m" // Red
#define LOG_CLR_RESET "\x1b[0m"

#if LOG_LEVEL != LOG_LEVEL_NONE
void init_logger(void);
void usart_printf(const char *str, ...);
#else
#define init_logger() ((void)0)
#define usart_printf() ((void)0)
#endif

// TRACE
#if LOG_LEVEL <= LOG_LEVEL_TRACE
#define log_trace(fmt, ...)                                                    \
    usart_printf(LOG_CLR_TRACE "[TRACE] " LOG_CLR_RESET fmt "\r\n",            \
                 ##__VA_ARGS__)
#else
#define log_trace(fmt, ...) ((void)0)
#endif

// DEBUG
#if LOG_LEVEL <= LOG_LEVEL_DEBUG
#define log_debug(fmt, ...)                                                    \
    usart_printf(LOG_CLR_DEBUG "[DEBUG] " LOG_CLR_RESET fmt "\r\n",            \
                 ##__VA_ARGS__)
#else
#define log_debug(fmt, ...) ((void)0)
#endif

// INFO
#if LOG_LEVEL <= LOG_LEVEL_INFO
#define log_info(fmt, ...)                                                     \
    usart_printf(LOG_CLR_INFO "[INFO] " LOG_CLR_RESET fmt "\r\n", ##__VA_ARGS__)
#else
#define log_info(fmt, ...) ((void)0)
#endif

// WARN
#if LOG_LEVEL <= LOG_LEVEL_WARN
#define log_warn(fmt, ...)                                                     \
    usart_printf(LOG_CLR_WARN "[WARN] " LOG_CLR_RESET fmt "\r\n", ##__VA_ARGS__)
#else
#define log_warn(fmt, ...) ((void)0)
#endif

// ERROR
#if LOG_LEVEL <= LOG_LEVEL_ERROR
#define log_error(fmt, ...)                                                    \
    usart_printf(LOG_CLR_ERROR "[ERROR] " LOG_CLR_RESET fmt "\r\n",            \
                 ##__VA_ARGS__)
#else
#define log_error(fmt, ...) ((void)0)
#endif

#endif // LOG_H
