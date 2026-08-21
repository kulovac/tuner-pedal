#include "i2s.h"
#include "bsp.h"
#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_spi.h"

static void i2s_sync_and_start(void);
static void i2s_start_dma_stream(uint16_t *rx_buf, uint32_t total_samples);

volatile transfer_state_t transfer = TRANSFERING;

void i2s_start_dma(uint16_t *rx_buf, uint32_t total_samples) {
    i2s_start_dma_stream(rx_buf, total_samples);
    i2s_sync_and_start();
}

void DMA2_Stream0_IRQHandler(void) {
    if (LL_DMA_IsActiveFlag_HT0(ADC_I2S_DMA)) {
        LL_DMA_ClearFlag_HT0(ADC_I2S_DMA);
        transfer = HALF_TRANSFER;
    }

    if (LL_DMA_IsActiveFlag_TC0(ADC_I2S_DMA)) {
        LL_DMA_ClearFlag_TC0(ADC_I2S_DMA);
        transfer = TRANSFER_COMPLETE;
    }
}

static void i2s_sync_and_start(void) {
    // Make sure I2S is disabled before we sync
    LL_I2S_Disable(ADC_I2S);

    // Standard Philips I2S: WS Low = Left, WS High = Right.
    // We want the hardware to detect a High-to-Low transition.

    // Wait until WS is LOW (Ensure we aren't currently in the Right channel)
    while (LL_GPIO_IsInputPinSet(ADC_I2S_PORT, ADC_I2S_WS_PIN))
        ;

    // Wait until WS transitions to HIGH (We are now in the Right channel)
    while (!LL_GPIO_IsInputPinSet(ADC_I2S_PORT, ADC_I2S_WS_PIN))
        ;

    // We are now safely in the middle of the Right channel.
    // Enable the peripheral. The very next edge it detects will be
    // the start of the Left channel!
    LL_I2S_Enable(ADC_I2S);
}

static void i2s_start_dma_stream(uint16_t *rx_buf, uint32_t total_samples) {
    transfer = TRANSFERING;

    LL_DMA_DisableStream(ADC_I2S_DMA, ADC_I2S_DMA_STREAM);

    LL_DMA_SetMemoryAddress(ADC_I2S_DMA, ADC_I2S_DMA_STREAM, (uint32_t)rx_buf);
    LL_DMA_SetDataLength(ADC_I2S_DMA, ADC_I2S_DMA_STREAM, total_samples);

    LL_DMA_ClearFlag_HT0(ADC_I2S_DMA);
    LL_DMA_ClearFlag_TC0(ADC_I2S_DMA);
    LL_DMA_ClearFlag_TE0(ADC_I2S_DMA);

    LL_DMA_EnableStream(ADC_I2S_DMA, ADC_I2S_DMA_STREAM);
}
