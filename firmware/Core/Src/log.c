#include "log.h"
#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stddef.h>
#include <stdio.h>

#define LOG_BUFFER_LEN 8192

static size_t logId = 0;
static UART_HandleTypeDef *logUart;

/**
 * @brief Defines which uart device to print logs to.
 * Requires initialization only once at the start of
 * the program.
 * @param uart: The uart connection parameter struct
 * @retval None
 */
void logger_init(UART_HandleTypeDef *uart) { logUart = uart; }

/**
 * @brief Embedded version of the `printf` function.
 * Allows sending and formatting strings over a
 * uart connection
 * @param fmt: format string
 * @retval None
 */
void log_printf(const char *fmt, ...) {
  char buffer[LOG_BUFFER_LEN];

  // Start with the log counter
  int offset = snprintf(buffer, LOG_BUFFER_LEN, "[%u] ", logId++);
  if (offset < 0 || offset >= LOG_BUFFER_LEN) {
    // encoding error or counter too big
    return;
  }

  // do the `f` stuff
  va_list args;
  va_start(args, fmt);
  int len = vsnprintf(buffer + offset, LOG_BUFFER_LEN - offset, fmt, args);
  va_end(args);

  if (len < 0)
    // again, encoding error
    return;

  // make sure no buffer overflow
  int total_len = len + offset;
  if (total_len > LOG_BUFFER_LEN - 3)
    total_len = LOG_BUFFER_LEN - 3;

  int newlineLen = snprintf(buffer + total_len, 3, "\r\n");

  HAL_UART_Transmit(logUart, (uint8_t *)buffer, total_len + newlineLen,
                    HAL_MAX_DELAY);
}
