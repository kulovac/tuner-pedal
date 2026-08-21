#include "bsp.h"
#include "dsp.h"
#include "gfx.h"
#include "i2s.h"
#include "log.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_utils.h"
#include "system_stm32f4xx.h"
#include "tft.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define NOTE_SCALE 4
#define NOTE_W (8 * 2 * NOTE_SCALE)
#define NOTE_H (8 * NOTE_SCALE)
#define CENT_SCALE 3
#define CENT_W (8 * 3 * CENT_SCALE)
#define CENT_H (8 * CENT_SCALE)
#define SPI_RX_BUFFER_SIZE (BUFFER_SIZE * 8)

static float buffer[BUFFER_SIZE];
static uint16_t spi_rx[SPI_RX_BUFFER_SIZE];

static void error_handler(void);
static void display_tuning(float freq, float cents, const char *note);
static void preprocess_buffer(uint16_t spi_rx[SPI_RX_BUFFER_SIZE],
                              float buffer[BUFFER_SIZE]);

int main(void) {
    bsp_init();
    init_logger();
    init_dsp();
    init_display();

    LL_GPIO_SetOutputPin(STATUS_LED_PORT, STATUS_LED_PIN);
    log_trace("Finished initialization");

    tft_clear_screen(GFX_COLOR_BLACK, 0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);
    i2s_start_dma(spi_rx, sizeof(spi_rx) / sizeof(spi_rx[0]));

    float freq = 0.0;
    float cents = 0.0;
    const char *note = "-";

    log_trace("Entering main loop");
    for (;;) {
        if (transfer == TRANSFERING) {
            display_tuning(freq, cents, note);

            log_info("Recorded freq: %d.%02d \t Note: %s \t Cents: %c%d.%02d",
                     (int32_t)lroundf(freq * 100.0f) / 100,
                     (int32_t)lroundf(freq * 100.0f) % 100, note,
                     ((int32_t)lroundf(cents * 100.0f) < 0) ? '-' : '+',
                     abs((int32_t)lroundf(cents * 100.0f)) / 100,
                     abs((int32_t)lroundf(cents * 100.0f)) % 100);
        } else if (transfer == HALF_TRANSFER || transfer == TRANSFER_COMPLETE) {
            preprocess_buffer(spi_rx, buffer);

            // Run DSP on the left-channel float buffer
            freq = compute_yin(buffer);
            cents = cents_diff(freq);
            note = get_note(freq);
        } else {
            log_assert(0, "unreachable: exhausts all transfer states");
        }
    }

    error_handler();
}

static void preprocess_buffer(uint16_t spi_rx[SPI_RX_BUFFER_SIZE],
                              float buffer[BUFFER_SIZE]) {
    const size_t half_elements = SPI_RX_BUFFER_SIZE / 2;
    uint16_t *raw_src =
        (transfer == HALF_TRANSFER) ? &spi_rx[0] : &spi_rx[half_elements];

    transfer = TRANSFERING;
    size_t float_idx = 0;

    // Step by 4 to skip the Right channel
    // (Left MSB, Left LSB, Right MSB, Right LSB)
    for (size_t i = 0; i < half_elements; i += 4) {
        uint16_t msb = raw_src[i];
        uint16_t lsb = raw_src[i + 1];

        int32_t merge = (int32_t)(((uint32_t)msb << 16) | lsb);
        buffer[float_idx++] = (float)(merge >> 8);
    }
}

static void display_tuning(float freq, float cents, const char *note) {
    static uint16_t note_buf[NOTE_W * NOTE_H];
    static uint16_t cent_buf[CENT_W * CENT_H];
    static const char *prev_note = NULL;
    static int8_t prev_cents = INT8_MAX;

    static enum { NOTE, CENTS } state = NOTE;

    if (!tft_is_ready())
        return;

    if (state == NOTE) {
        if (prev_note != note) {
            gfx_render_string_centered(note_buf, NOTE_W, NOTE_H, note,
                                       GFX_COLOR_WHITE, GFX_COLOR_BLACK,
                                       NOTE_SCALE);

            const uint16_t x0 = (TFT_WIDTH - 2 * NOTE_SCALE * 8) / 2;
            const uint16_t y0 = TFT_HEIGHT / 2 - NOTE_SCALE * 8 - 6;
            gfx_draw(note_buf, x0, y0, x0 + NOTE_W - 1, y0 + NOTE_H - 1);

            prev_note = note;
        }

        state = CENTS;
    } else if (state == CENTS) {
        int8_t rounded_cents = (int8_t)lroundf(cents);

        if (prev_cents != rounded_cents) {
            char cents_str[16];
            snprintf(cents_str, 16, "%+d", rounded_cents);

            gfx_render_string_centered(cent_buf, CENT_W, CENT_H, cents_str,
                                       GFX_COLOR_WHITE, GFX_COLOR_BLACK,
                                       CENT_SCALE);

            const uint16_t x0 = (TFT_WIDTH - 3 * CENT_SCALE * 8) / 2;
            const uint16_t y0 = TFT_HEIGHT / 2 + 6;
            gfx_draw(cent_buf, x0, y0, x0 + CENT_W - 1, y0 + CENT_H - 1);

            prev_cents = rounded_cents;
        }

        state = NOTE;
    }
}

static void error_handler(void) {
    LL_Init1msTick(SystemCoreClock);
    for (;;) {
        LL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
        LL_mDelay(500);
    }
}
