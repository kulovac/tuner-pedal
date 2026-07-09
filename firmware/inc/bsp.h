#ifndef BSP_H
#define BSP_H

#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"

// TODO: Prevent calling clock enable more than once one the same periph. e.g.
// use ifndef guards

#define STATUS_LED_PORT GPIOA
#define STATUS_LED_PIN LL_GPIO_PIN_8
#define STATUS_LED_CLK_ENABLE()                                                \
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA)

#define LOGGER_USART USART1
#define LOGGER_USART_CLK_ENABLE()                                              \
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_USART1)

#define LOGGER_TX_PORT GPIOA
#define LOGGER_TX_PIN LL_GPIO_PIN_9
#define LOGGER_TX_AF LL_GPIO_AF_7
#define LOGGER_TX_CLK_ENABLE()                                                 \
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA)

#endif /* BSP_H */
