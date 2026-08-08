#ifndef TFT_H
#define TFT_H

#include <stdint.h>

#define TFT_WIDTH 128
#define TFT_HEIGHT 160

void init_display(void);
void tft_reset_display(void);
void tft_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void tft_write_data16(uint16_t data);
void tft_clear_screen(uint16_t color, uint16_t x0, uint16_t y0, uint16_t x1,
                      uint16_t y1);

#endif
