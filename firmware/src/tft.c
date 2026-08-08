#include "tft.h"
#include "bsp.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_utils.h"
#include <stddef.h>
#include <stdint.h>

#define SLPOUT 0x11
#define COLMOD 0x3A
#define COLMOD_12BIT 0b011
#define COLMOD_16BIT 0b101
#define COLMOD_18BIT 0b110
#define COLMOD_NONE 0b111
#define DISPON 0x29
#define CASET 0x2A
#define RASET 0x2B
#define RAMWR 0x2C

// Internal helper to transmit 1 byte and wait until completely finished
static inline void tft_spi_tx(uint8_t data) {
    while (!LL_SPI_IsActiveFlag_TXE(TFT_SPI))
        ;
    LL_SPI_TransmitData8(TFT_SPI, data);
}

// Send a 1-byte Command (D/C pulled LOW)
static void tft_write_cmd(uint8_t cmd) {
    while (LL_SPI_IsActiveFlag_BSY(TFT_SPI))
        ;
    LL_GPIO_ResetOutputPin(TFT_SPI_PORT, TFT_SPI_DC_PIN); // DC LOW (Command)
    tft_spi_tx(cmd);
}

// Send a 1-byte Data payload (D/C pulled HIGH)
static void tft_write_data8(uint8_t data) {
    while (LL_SPI_IsActiveFlag_BSY(TFT_SPI))
        ;
    LL_GPIO_SetOutputPin(TFT_SPI_PORT, TFT_SPI_DC_PIN); // DC HIGH (Data)
    tft_spi_tx(data);
}

// Send a 16-bit RGB565 Color word (MSB first)
void tft_write_data16(uint16_t data) {
    while (LL_SPI_IsActiveFlag_BSY(TFT_SPI))
        ;
    LL_GPIO_SetOutputPin(TFT_SPI_PORT, TFT_SPI_DC_PIN); // DC HIGH (Data)
    tft_spi_tx(data >> 8);                              // Send High Byte
    tft_spi_tx(data & 0xFF);                            // Send Low Byte
}

// Opens a rectangular drawing window on the screen
void tft_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    tft_write_cmd(CASET); // X Coordinates
    tft_write_data16(x0 + 2);
    tft_write_data16(x1 + 2);

    tft_write_cmd(RASET); // Y Coordinates
    tft_write_data16(y0 + 1);
    tft_write_data16(y1 + 1);

    tft_write_cmd(RAMWR); // Prepare to receive color data
}

void tft_clear_screen(uint16_t color, uint16_t x0, uint16_t y0, uint16_t x1,
                      uint16_t y1) {
    tft_set_window(x0, y0, x1, y1);

    for (size_t i = 0; i < (x1 - x0 + 1) * (y1 - y0 + 1); ++i)
        tft_write_data16(color);
}

void tft_reset_display(void) {
    LL_GPIO_ResetOutputPin(TFT_RT_PORT, TFT_RT_PIN);
    // Minimum delay to trigger reset is 10us
    LL_mDelay(1);
    LL_GPIO_SetOutputPin(TFT_RT_PORT, TFT_RT_PIN);
    // Maximum delay for reset state to finish is 120ms
    LL_mDelay(120);
}

void init_display(void) {
    // Select the display spi device
    LL_GPIO_ResetOutputPin(TFT_SPI_PORT, TFT_SPI_CS_PIN);
    // Set the backlight pin ON and the nRESET pin HIGH
    LL_GPIO_SetOutputPin(TFT_BL_PORT, TFT_BL_PIN | TFT_RT_PIN);

    tft_reset_display();

    tft_write_cmd(SLPOUT); // Wakes up the oscillator
    LL_mDelay(120); // Datasheet requires 120ms after SLPOUT before drawing

    tft_write_cmd(COLMOD);         // Interface Pixel Format
    tft_write_data8(COLMOD_16BIT); // RGB565

    tft_write_cmd(DISPON); // Display ON
}
