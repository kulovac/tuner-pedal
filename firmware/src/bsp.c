#include "bsp.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"

void bsp_init(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    LL_GPIO_InitTypeDef led = {.Pin = STATUS_LED_PIN,
                               .Mode = LL_GPIO_MODE_OUTPUT,
                               .Speed = LL_GPIO_SPEED_FREQ_LOW,
                               .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                               .Pull = LL_GPIO_PULL_NO,
                               .Alternate = LL_GPIO_AF_0};
    LL_GPIO_Init(STATUS_LED_PORT, &led);
}
