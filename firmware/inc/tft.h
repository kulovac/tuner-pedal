#ifndef TFT_H
#define TFT_H

#include <stdint.h>

// Standard RGB565 Color Definitions
#define TFT_BLACK 0x0000
#define TFT_WHITE 0xFFFF
#define TFT_RED 0xF800
#define TFT_GREEN 0x07E0
#define TFT_BLUE 0x001F
#define TFT_YELLOW 0xFFE0
#define TFT_CYAN 0x07FF

#define TFT_WIDTH 128
#define TFT_HEIGHT 160

void init_display(void);
void tft_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t fg_color,
                     uint16_t bg_color, uint8_t scale);
void tft_clear_screen(uint16_t color, uint16_t x0, uint16_t y0, uint16_t x1,
                      uint16_t y1);

#endif
