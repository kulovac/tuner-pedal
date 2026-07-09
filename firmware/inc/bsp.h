#ifndef BSP_H
#define BSP_H

#include "stm32f4xx_ll_gpio.h"

#define STATUS_LED_PORT GPIOA
#define STATUS_LED_PIN LL_GPIO_PIN_8

#define LOGGER_USART USART1
#define LOGGER_TX_PORT GPIOA
#define LOGGER_TX_PIN LL_GPIO_PIN_9
#define LOGGER_TX_AF LL_GPIO_AF_7

void bsp_init(void);

#endif /* BSP_H */
