#include "gfx.h"
#include "fonts.h"
#include "log.h"
#include "tft.h"
#include <stddef.h>
#include <stdint.h>
#include <string.h>

void gfx_render_box(uint16_t *buf, uint16_t box_w, uint16_t box_h,
                    uint16_t color) {
    for (size_t i = 0; i < box_w * box_h; ++i)
        buf[i] = color;
}

void gfx_render_string_centered(uint16_t *buf, uint16_t box_w, uint16_t box_h,
                                const char *str, uint16_t fg_color,
                                uint16_t bg_color, uint8_t scale) {
    // Blast the entire buffer with the background color first.
    // This is much faster than checking every pixel later.
    gfx_render_box(buf, box_w, box_h, bg_color);

    // Calculate text dimensions
    size_t len = strlen(str);
    uint16_t text_w = len * 8 * scale;
    uint16_t text_h = 8 * scale;

    // Safety check: If the box is too small, abort to prevent memory corruption
    log_assert(text_w <= box_w && text_h <= box_h,
               "Text width or height exceeded window");

    // Calculate top-left starting coordinates for perfect centering
    uint16_t offset_x = (box_w - text_w) / 2;
    uint16_t offset_y = (box_h - text_h) / 2;

    // Draw the text into the buffer
    for (size_t c = 0; c < len; ++c) {
        uint16_t char_idx = str[c] * 8;
        uint16_t char_start_x = offset_x + (c * 8 * scale);

        for (uint8_t row = 0; row < 8; ++row) {
            uint8_t bitmask = console_font_8x8[char_idx + row];

            for (uint8_t sy = 0; sy < scale; ++sy) {
                uint16_t pixel_y = offset_y + (row * scale) + sy;

                for (uint8_t col = 0; col < 8; ++col) {
                    // If the font bit is 1, overwrite the background color
                    if (bitmask & (0x80 >> col)) {
                        for (uint8_t sx = 0; sx < scale; ++sx) {
                            uint16_t pixel_x =
                                char_start_x + (col * scale) + sx;
                            // Translate 2D coordinates into a 1D array index
                            size_t buf_idx = (pixel_y * box_w) + pixel_x;
                            buf[buf_idx] = fg_color;
                        }
                    }
                }
            }
        }
    }
}

void gfx_draw(uint16_t *buf, uint16_t x0, uint16_t y0, uint16_t x1,
              uint16_t y1) {
    log_assert(tft_is_ready(), "Called gfx_draw before display was ready!");
    tft_set_window(x0, y0, x1, y1);
    tft_start_dma_stream(buf, (x1 - x0 + 1) * (y1 - y0 + 1));
}
