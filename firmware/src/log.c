#include "log.h"
#include "stm32f411xe.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include <stdarg.h>
#include <stdio.h>

static void usart_puts(const char *);

void init_logger(void) {
    LL_GPIO_InitTypeDef tx_pin = {.Pin = LL_GPIO_PIN_9,
                                  .Mode = LL_GPIO_MODE_ALTERNATE,
                                  .Speed = LL_GPIO_SPEED_FREQ_HIGH,
                                  .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                                  .Pull = LL_GPIO_PULL_UP,
                                  .Alternate = LL_GPIO_AF_7};
    LL_GPIO_Init(GPIOA, &tx_pin);

    LL_USART_InitTypeDef logger = {.BaudRate = 115200,
                                   .DataWidth = LL_USART_DATAWIDTH_8B,
                                   .StopBits = LL_USART_STOPBITS_1,
                                   .Parity = LL_USART_PARITY_NONE,
                                   .TransferDirection = LL_USART_DIRECTION_TX,
                                   .HardwareFlowControl =
                                       LL_USART_HWCONTROL_NONE,
                                   .OverSampling = LL_USART_OVERSAMPLING_8};
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1);
    LL_USART_Init(USART1, &logger);
    LL_USART_Enable(USART1);
}

void usart_printf(const char *str, ...) {
    char formatted[LOG_BUFFER_SIZE];
    va_list args;

    va_start(args, str);
    vsnprintf(formatted, LOG_BUFFER_SIZE, str, args);
    va_end(args);

    usart_puts(formatted);
}

static void usart_puts(const char *str) {
    while (*str != '\0') {
        while (!LL_USART_IsActiveFlag_TXE(USART1))
            ;
        LL_USART_TransmitData8(USART1, *str++);
    }
}
