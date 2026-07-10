#include "bsp.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_utils.h"
#include "system_stm32f4xx.h"

#define CLOCK_FREQ 100000000UL

static void clock_init(void);

void bsp_init(void) {
    clock_init();

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

    // TODO: Refactor these inits into static helpers

    // Status LED
    LL_GPIO_InitTypeDef led = {.Pin = STATUS_LED_PIN,
                               .Mode = LL_GPIO_MODE_OUTPUT,
                               .Speed = LL_GPIO_SPEED_FREQ_LOW,
                               .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                               .Pull = LL_GPIO_PULL_NO,
                               .Alternate = LL_GPIO_AF_0};
    LL_GPIO_Init(STATUS_LED_PORT, &led);

    // External ADC
    LL_GPIO_InitTypeDef i2s_pins = {.Pin = ADC_I2S_BCLK_PIN | ADC_I2S_WS_PIN |
                                           ADC_I2S_SD_PIN,
                                    .Mode = LL_GPIO_MODE_ALTERNATE,
                                    .Speed = LL_GPIO_SPEED_FREQ_HIGH,
                                    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                                    .Pull = LL_GPIO_PULL_NO,
                                    .Alternate = ADC_I2S_AF};
    LL_GPIO_Init(ADC_I2S_PORT, &i2s_pins);

    LL_I2S_InitTypeDef i2s = {.Mode = LL_I2S_MODE_SLAVE_RX,
                              .DataFormat = LL_I2S_DATAFORMAT_24B,
                              .Standard = LL_I2S_STANDARD_PHILIPS,
                              .MCLKOutput = LL_I2S_MCLK_OUTPUT_DISABLE,
                              .AudioFreq = LL_I2S_AUDIOFREQ_16K,
                              .ClockPolarity = LL_I2S_POLARITY_LOW};
    LL_I2S_Init(ADC_I2S, &i2s);
}

static void clock_init(void) {
    // NOTE: All values are set to their max
    // according to the datasheet
    //
    // AHB(max) = 100MHz
    // APB1(max) = 50Mhz
    // APB2(max) = 100Mhz

    // TODO: lower clock frequency as much as
    // possible and determine power savings

    LL_UTILS_PLLInitTypeDef pll_init = {
        .PLLM = LL_RCC_PLLM_DIV_8, // 16MHz / 8  = 2MHz
        .PLLN = 100,               // 2MHz * 100 = 200MHz
        .PLLP = LL_RCC_PLLP_DIV_2  // 200MHz / 2 = 100MHz
    };

    LL_UTILS_ClkInitTypeDef clk_init = {
        .AHBCLKDivider = LL_RCC_SYSCLK_DIV_1, // 100MHz
        .APB1CLKDivider = LL_RCC_APB1_DIV_2,  // 50MHz
        .APB2CLKDivider = LL_RCC_APB2_DIV_1,  // 100MHz
    };

    ErrorStatus e = LL_PLL_ConfigSystemClock_HSI(&pll_init, &clk_init);
    if (e == ERROR) {
        // TODO: error handling, but you'll know if nothing turns on...
        for (;;) {
        }
    }

    SystemCoreClock = CLOCK_FREQ;
    LL_Init1msTick(SystemCoreClock);
}
