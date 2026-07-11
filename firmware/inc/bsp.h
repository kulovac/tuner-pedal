#ifndef BSP_H
#define BSP_H

#include "stm32f4xx_ll_gpio.h"

#define STATUS_LED_PORT GPIOA
#define STATUS_LED_PIN LL_GPIO_PIN_8

#define LOGGER_USART USART1
#define LOGGER_TX_PORT GPIOA
#define LOGGER_TX_PIN LL_GPIO_PIN_9
#define LOGGER_TX_AF LL_GPIO_AF_7

#define ADC_I2S SPI1
#define ADC_I2S_PORT GPIOA
#define ADC_I2S_BCLK_PIN LL_GPIO_PIN_5
#define ADC_I2S_WS_PIN LL_GPIO_PIN_4
#define ADC_I2S_SD_PIN LL_GPIO_PIN_7
#define ADC_I2S_AF LL_GPIO_AF_5

void bsp_init(void);

enum CHSIDE { CHLEFT = 0, CHRIGHT = 1 };

#endif /* BSP_H */
