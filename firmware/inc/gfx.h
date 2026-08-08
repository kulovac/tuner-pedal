#ifndef GFX_H
#define GFX_H

#include <stdint.h>

// Standard RGB565 Colors
// XXX: stm32 is little endian but the st7735 is big endian
// so I swap the colour bytes around as a temporary solution
#define GFX_COLOR_BLACK __builtin_bswap16(0x0000)
#define GFX_COLOR_WHITE __builtin_bswap16(0xFFFF)
#define GFX_COLOR_RED __builtin_bswap16(0xF800)
#define GFX_COLOR_GREEN __builtin_bswap16(0x07E0)
#define GFX_COLOR_BLUE __builtin_bswap16(0x001F)
#define GFX_COLOR_YELLOW __builtin_bswap16(0xFFE0)
#define GFX_COLOR_CYAN __builtin_bswap16(0x07FF)
#define GFX_COLOR_MAGENTA __builtin_bswap16(0xF81F)

void gfx_render_box(uint16_t *buf, uint16_t box_w, uint16_t box_h,
                    uint16_t color);
void gfx_render_string_centered(uint16_t *buf, uint16_t box_w, uint16_t box_h,
                                const char *str, uint16_t fg_color,
                                uint16_t bg_color, uint8_t scale);
void gfx_draw(uint16_t *buf, uint16_t x0, uint16_t y0, uint16_t x1,
              uint16_t y1);

#endif
