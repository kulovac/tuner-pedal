#include "dsp.h"
#include "log.h"
#include <arm_math.h>
#include <stddef.h>
#include <stdint.h>

// Size of the analysis window (samples)
#define W_LEN BUFFER_SIZE
// Size of the lag between two consecutives windows (samples)
#define W_STEP 256
// Sample rate of ADC (Hz)
#define SR 16000
// Minimum fundamental frequency that can be detected (Hz)
#define F0_MIN 50
// Maximum fundamental frequency that can be detected (Hz)
#define F0_MAX 2000
// Threshold of detection. The algorithm return the first
// minimum of the CMND function below this treshold.
#define HARMO_THRESH 0.3f
#define TAU_MAX (SR / F0_MIN)
#define TAU_MIN (SR / F0_MAX)
// Size of the FFT buffer (W_LEN + TAU_MAX)
#define FFT_LEN (2 * W_LEN)

static arm_rfft_fast_instance_f32 rfft;

static void difference_function(const float32_t frame[W_LEN],
                                float32_t df[TAU_MAX]);
static void cumsum_f32(float32_t *pSrc, float32_t *pDst, size_t src_len);
static size_t get_pitch(float32_t cmdf[TAU_MAX]);
static float32_t parabolic_interp(float32_t cmdf[TAU_MAX], size_t tau);
static void
cumulative_mean_normalized_difference_function(float32_t df[TAU_MAX]);

void init_dsp(void) {
    // TODO: Finalize the buffer length `BUFFER_SIZE`
    arm_status status = arm_rfft_fast_init_f32(&rfft, FFT_LEN);
    log_assert(status == ARM_MATH_SUCCESS, "Failed to init dsp unit");
}

float compute_yin(float32_t sig[BUFFER_SIZE]) {
    log_trace("Computing yin algorithm");

    float32_t df[TAU_MAX];
    difference_function(sig, df);
    cumulative_mean_normalized_difference_function(df);
    size_t tau = get_pitch(df);
    float32_t tauf = parabolic_interp(df, tau);

    return (float32_t)SR / tauf;
}

static void cumsum_f32(float32_t *pSrc, float32_t *pDst, size_t src_len) {
    pDst[0] = 0;
    for (size_t i = 0; i < src_len; ++i)
        pDst[i + 1] = pDst[i] + pSrc[i];
}

static void difference_function(const float32_t frame[W_LEN],
                                float32_t df[TAU_MAX]) {
    float32_t mult[W_LEN];
    arm_mult_f32(frame, frame, mult, W_LEN);

    float32_t sum[W_LEN + 1];
    cumsum_f32(mult, sum, W_LEN);

    float32_t sig_padded[FFT_LEN] = {0};
    for (size_t i = 0; i < W_LEN; ++i) {
        sig_padded[i] = frame[i];
    }

    float32_t fc[FFT_LEN];
    arm_rfft_fast_f32(&rfft, sig_padded, fc, 0);

    float32_t fc_conj[FFT_LEN];
    fc_conj[0] = fc[0];
    fc_conj[1] = fc[1];
    arm_cmplx_conj_f32(fc + 2, fc_conj + 2, W_LEN - 1);

    float32_t fft_conv[FFT_LEN] = {0};
    arm_mult_f32(fc, fc_conj, fft_conv, 2);
    arm_cmplx_mult_cmplx_f32(fc + 2, fc_conj + 2, fft_conv + 2, W_LEN - 1);

    // XXX: The actual length is TAU_MAX
    float32_t conv[FFT_LEN];
    arm_rfft_fast_f32(&rfft, fft_conv, conv, 1);

    for (size_t i = 0; i < TAU_MAX; ++i) {
        df[i] = sum[W_LEN - i] + sum[W_LEN] - sum[i] - 2 * conv[i];
    }
}

static void
cumulative_mean_normalized_difference_function(float32_t df[TAU_MAX]) {
    float32_t difference_sum[TAU_MAX];
    cumsum_f32(df + 1, difference_sum, TAU_MAX - 1);

    df[0] = 1;
    for (size_t i = 1; i < TAU_MAX; ++i) {
        df[i] = df[i] * i / difference_sum[i];
    }
}

static size_t get_pitch(float32_t cmdf[TAU_MAX]) {
    size_t tau = TAU_MIN;

    while (tau < TAU_MAX) {
        if (cmdf[tau] < HARMO_THRESH) {
            while (tau + 1 < TAU_MAX && cmdf[tau + 1] < cmdf[tau]) {
                ++tau;
            }
            return tau;
        }
        ++tau;
    }

    return 0; // if unvoiced
}

static float32_t parabolic_interp(float32_t cmdf[TAU_MAX], size_t tau) {
    if (tau == 0 || tau == TAU_MAX - 1) {
        return (float32_t)tau;
    }

    float32_t offset =
        (cmdf[tau + 1] - cmdf[tau - 1]) /
        (2.0f * (2.0f * cmdf[tau] - cmdf[tau + 1] - cmdf[tau - 1]));

    return offset + (float32_t)tau;
}
