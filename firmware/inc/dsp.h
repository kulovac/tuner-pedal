#include <stddef.h>
#include <stdint.h>
#define BUFFER_SIZE 1024

/**
 *  @brief computes the base frequency
 *  @param sig Audio signal
 *  @param sr sampling rate
 *  @param w_len size of the analysis window (samples)
 *  @param w_step size of the lag between two consecutives windows (samples)
 *  @param f0_min Minimum fundamental frequency that can be detected (hertz)
 *  @param f0_max Maximum fundamental frequency that can be detected (hertz)
 *  @param harmo_thresh Threshold of detection. The algorithm return the first
 * minimum of the CMND fubction below this treshold.
 */
float compute_yin(float sig[BUFFER_SIZE], int32_t sr, size_t w_len,
                  size_t w_step, int32_t f0_min, int32_t f0_max,
                  float harmo_thresh);
