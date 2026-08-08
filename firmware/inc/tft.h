#ifndef TFT_H
#define TFT_H

#include <stdbool.h>
#include <stdint.h>

#define TFT_WIDTH 128
#define TFT_HEIGHT 160

void init_display(void);
void tft_reset_display(void);
void tft_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
void tft_write_data16(uint16_t data);
void tft_clear_screen(uint16_t color, uint16_t x0, uint16_t y0, uint16_t x1,
                      uint16_t y1);
bool tft_is_ready(void);
void tft_start_dma_stream(uint16_t *buf, uint32_t pixel_count);

#endif
