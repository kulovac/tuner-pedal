#include "bsp.h"
#include "dsp.h"
#include "log.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_utils.h"
#include "system_stm32f4xx.h"
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>

static void error_handler(void);
static bool collect_sample(int32_t *, enum CHSIDE);

int main(void) {
    bsp_init();
    init_logger();
    init_dsp();

    LL_GPIO_SetOutputPin(STATUS_LED_PORT, STATUS_LED_PIN);

    log_trace("entering test %d", 1);
    log_debug("entering test %d", 2);
    log_info("entering test %d", 3);
    log_warn("entering test %d", 4);
    log_error("entering test %d", 5);

    float buffer[BUFFER_SIZE];

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

        int32_t freq_scaled = (int32_t)lroundf(freq * 100.0f);
        int32_t cents_scaled = (int32_t)lroundf(cents * 100.0f);

        char cents_sign = (cents_scaled < 0) ? '-' : '+';
        int32_t cents_abs = abs(cents_scaled);

        log_info("Recorded freq: %d.%02d\tNote: %s\tCents: %c%d.%02d",
                 freq_scaled / 100, freq_scaled % 100, note, cents_sign,
                 cents_abs / 100, cents_abs % 100);
    }

    error_handler();
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
