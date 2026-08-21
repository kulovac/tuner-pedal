#ifndef I2S_H
#define I2S_H

#include <stdint.h>

typedef enum { TRANSFERING, HALF_TRANSFER, TRANSFER_COMPLETE } transfer_state_t;
extern volatile transfer_state_t transfer;

void i2s_start_dma(uint16_t *rx_buf, uint32_t total_samples);

#endif
