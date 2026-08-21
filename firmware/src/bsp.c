#include "bsp.h"
#include "stm32f4xx.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_utils.h"
#include "system_stm32f4xx.h"

#define CLOCK_FREQ 100000000UL

static void clock_init(void);
static void led_init(void);
static void i2s_adc_init(void);
static void i2s_dma_init(void);
static void spi_tft_init(void);
static void spi_dma_init(void);

void bsp_init(void) {
    clock_init();

    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA2);

    led_init();
    i2s_adc_init();
    spi_tft_init();
}

static void spi_tft_init(void) {
    // TFT Display
    LL_GPIO_InitTypeDef spi_pins = {.Pin = TFT_SPI_MOSI_PIN | TFT_SPI_SCK_PIN,
                                    .Mode = LL_GPIO_MODE_ALTERNATE,
                                    .Speed = LL_GPIO_SPEED_FREQ_VERY_HIGH,
                                    .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                                    .Pull = LL_GPIO_PULL_NO,
                                    .Alternate = TFT_SPI_AF};
    LL_GPIO_InitTypeDef spi_ctl_pins = {.Pin = TFT_SPI_CS_PIN | TFT_SPI_DC_PIN,
                                        .Mode = LL_GPIO_MODE_OUTPUT,
                                        .Speed = LL_GPIO_SPEED_FREQ_MEDIUM,
                                        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                                        .Pull = LL_GPIO_PULL_NO,
                                        .Alternate = LL_GPIO_AF_0};
    LL_GPIO_InitTypeDef tft_ctl_pins = {.Pin = TFT_BL_PIN | TFT_RT_PIN,
                                        .Mode = LL_GPIO_MODE_OUTPUT,
                                        .Speed = LL_GPIO_SPEED_FREQ_LOW,
                                        .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                                        .Pull = LL_GPIO_PULL_NO,
                                        .Alternate = LL_GPIO_AF_0};
    LL_GPIO_Init(TFT_SPI_PORT, &spi_pins);
    LL_GPIO_Init(TFT_SPI_PORT, &spi_ctl_pins);
    LL_GPIO_Init(TFT_BL_PORT, &tft_ctl_pins);

    LL_SPI_InitTypeDef spi = {.TransferDirection = LL_SPI_FULL_DUPLEX,
                              .Mode = LL_SPI_MODE_MASTER,
                              .DataWidth = LL_SPI_DATAWIDTH_8BIT,
                              .ClockPolarity = LL_SPI_POLARITY_LOW,
                              .ClockPhase = LL_SPI_PHASE_1EDGE,
                              .NSS = LL_SPI_NSS_SOFT,
                              .BaudRate = LL_SPI_BAUDRATEPRESCALER_DIV2,
                              .BitOrder = LL_SPI_MSB_FIRST,
                              .CRCCalculation = LL_SPI_CRCCALCULATION_DISABLE,
                              .CRCPoly = 0x0};
    LL_SPI_Init(TFT_SPI, &spi);
    LL_SPI_Enable(TFT_SPI);

    spi_dma_init();
}

static void spi_dma_init(void) {
    // SPI->TFT DMA
    LL_DMA_InitTypeDef spi_dma = {
        .Channel = TFT_SPI_DMA_CHANNEL,
        .PeriphOrM2MSrcAddress = (uint32_t)&(TFT_SPI->DR),
        .MemoryOrM2MDstAddress = 0,
        .Direction = LL_DMA_DIRECTION_MEMORY_TO_PERIPH,
        .Mode = LL_DMA_MODE_NORMAL,
        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE,
        .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE,
        .NbData = 0,
        .Priority = LL_DMA_PRIORITY_HIGH,
        .FIFOMode = LL_DMA_FIFOMODE_DISABLE};
    LL_DMA_Init(TFT_SPI_DMA, TFT_SPI_DMA_STREAM, &spi_dma);

    LL_DMA_EnableIT_TC(TFT_SPI_DMA, TFT_SPI_DMA_STREAM);

    NVIC_SetPriority(TFT_SPI_DMA_IRQn, 1);
    NVIC_EnableIRQ(TFT_SPI_DMA_IRQn);

    LL_SPI_EnableDMAReq_TX(TFT_SPI);
}

static void i2s_adc_init(void) {
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
    // WARN: do not enable I2S due to L/R channel synchronization issues
    // this is resolved within i2s.{c,h}

    i2s_dma_init();
}

static void i2s_dma_init(void) {
    // I2S -> Audio Buffer DMA
    LL_DMA_InitTypeDef i2s_dma = {
        .Channel = ADC_I2S_DMA_CHANNEL,
        .PeriphOrM2MSrcAddress = (uint32_t)&(ADC_I2S->DR),
        .MemoryOrM2MDstAddress = 0, // Will be set when we start it
        .Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY,
        .Mode = LL_DMA_MODE_CIRCULAR,
        .PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT,
        .MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT,
        .PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_HALFWORD,
        .MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_HALFWORD,
        .NbData = 0,
        .Priority = LL_DMA_PRIORITY_VERYHIGH,
        .FIFOMode = LL_DMA_FIFOMODE_DISABLE};
    LL_DMA_Init(ADC_I2S_DMA, ADC_I2S_DMA_STREAM, &i2s_dma);

    // Enable both Half-Transfer (HT) and Transfer
    // Complete (TC)for ping-pong buffering
    LL_DMA_EnableIT_HT(ADC_I2S_DMA, ADC_I2S_DMA_STREAM);
    LL_DMA_EnableIT_TC(ADC_I2S_DMA, ADC_I2S_DMA_STREAM);

    NVIC_SetPriority(ADC_I2S_DMA_IRQn, 0);
    NVIC_EnableIRQ(ADC_I2S_DMA_IRQn);

    LL_SPI_EnableDMAReq_RX(ADC_I2S);
}

static void led_init(void) {
    // Status LED
    LL_GPIO_InitTypeDef led = {.Pin = STATUS_LED_PIN,
                               .Mode = LL_GPIO_MODE_OUTPUT,
                               .Speed = LL_GPIO_SPEED_FREQ_LOW,
                               .OutputType = LL_GPIO_OUTPUT_PUSHPULL,
                               .Pull = LL_GPIO_PULL_NO,
                               .Alternate = LL_GPIO_AF_0};
    LL_GPIO_Init(STATUS_LED_PORT, &led);
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
