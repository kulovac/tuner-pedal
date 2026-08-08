#include "bsp.h"
#include "dsp.h"
#include "gfx.h"
#include "log.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"
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

static void error_handler(void);
static bool collect_sample(int32_t *, enum CHSIDE);
static void display_tuning(float freq, float cents, const char *note);

int main(void) {
    bsp_init();
    init_logger();
    init_dsp();
    init_display();

    LL_GPIO_SetOutputPin(STATUS_LED_PORT, STATUS_LED_PIN);

    log_trace("entering test %d", 1);
    log_debug("entering test %d", 2);
    log_info("entering test %d", 3);
    log_warn("entering test %d", 4);
    log_error("entering test %d", 5);

    float buffer[BUFFER_SIZE];

    tft_clear_screen(GFX_COLOR_BLACK, 0, 0, TFT_WIDTH - 1, TFT_HEIGHT - 1);

    /* Loop forever */
    for (;;) {
        if (LL_I2S_IsActiveFlag_OVR(ADC_I2S))
            LL_I2S_ClearFlag_OVR(ADC_I2S);

        for (size_t i = 0; i < BUFFER_SIZE; ++i) {
            // left channel
            int32_t left;
            while (!collect_sample(&left, CHLEFT))
                ;

            // right channel
            //
            // NOTE: We ignore this one since
            // we are sampling a mono signal
            // and this channel is grounded
            int32_t right;
            while (!collect_sample(&right, CHRIGHT))
                ;

            buffer[i] = (float)left;
        }

        float freq = compute_yin(buffer);
        float cents = cents_diff(freq);
        const char *note = get_note(freq);
        display_tuning(freq, cents, note);

        log_info("Recorded freq: %d.%02d \t Note: %s \t Cents: %c%d.%02d",
                 (int32_t)lroundf(freq * 100.0f) / 100,
                 (int32_t)lroundf(freq * 100.0f) % 100, note,
                 ((int32_t)lroundf(cents * 100.0f) < 0) ? '-' : '+',
                 abs((int32_t)lroundf(cents * 100.0f)) / 100,
                 abs((int32_t)lroundf(cents * 100.0f)) % 100);
    }

    error_handler();
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

static bool collect_sample(int32_t *val, enum CHSIDE ch) {
    while (!LL_I2S_IsActiveFlag_RXNE(ADC_I2S))
        ;
    if (LL_I2S_IsActiveFlag_CHSIDE(ADC_I2S) != ch) {
        LL_I2S_ReceiveData16(ADC_I2S);
        return false;
    }
    uint16_t msb = LL_I2S_ReceiveData16(ADC_I2S);

    while (!LL_I2S_IsActiveFlag_RXNE(ADC_I2S))
        ;
    if (LL_I2S_IsActiveFlag_CHSIDE(ADC_I2S) != ch) {
        LL_I2S_ReceiveData16(ADC_I2S);
        return false;
    }
    uint16_t lsb = LL_I2S_ReceiveData16(ADC_I2S);

    int32_t merge = (int32_t)(((uint32_t)msb << 16) | lsb);
    *val = merge >> 8;

    return true;
}

static void error_handler(void) {
    LL_Init1msTick(SystemCoreClock);
    for (;;) {
        LL_GPIO_TogglePin(STATUS_LED_PORT, STATUS_LED_PIN);
        LL_mDelay(500);
    }
}
