#include "dsp.h"
#include "log.h"
#include <arm_math.h>
#include <math.h>
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
#define A4_FREQ 440.0f

static arm_rfft_fast_instance_f32 rfft;

static void difference_function(float32_t frame[W_LEN], float32_t df[TAU_MAX]);
static void cumsum_f32(const float32_t *pSrc, float32_t *pDst, size_t src_len);
static size_t get_pitch(const float32_t cmndf[TAU_MAX]);
static float32_t parabolic_interp(const float32_t cmndf[TAU_MAX], size_t tau);
static void
cumulative_mean_normalized_difference_function(float32_t df[TAU_MAX]);

void init_dsp(void) {
    // TODO: Finalize the buffer length `BUFFER_SIZE`
    arm_rfft_fast_init_f32(&rfft, FFT_LEN);
}

inline float32_t cents_diff(float32_t freq) {
    return remainderf(1200.0f * log2f(freq / A4_FREQ), 100.0f);
}

// TODO: return an enum over a string
char *get_note(float freq) {
    if (freq <= 0.0f) {
        return "-"; // Unvoiced / Silence
    }

    int semitones_from_a4 = (int)lroundf(12.0f * log2f(freq / A4_FREQ));

    // Map to a 0-11 pitch class (where C = 0, A = 9)
    // We add 21 (9 for A, plus 12) before the second modulo to guarantee
    // positive results when dealing with negative C integer division in lower
    // octaves.
    int note_index = (semitones_from_a4 % 12 + 21) % 12;

    switch (note_index) {
    case 0:
        return "C";
    case 1:
        return "C#";
    case 2:
        return "D";
    case 3:
        return "D#";
    case 4:
        return "E";
    case 5:
        return "F";
    case 6:
        return "F#";
    case 7:
        return "G";
    case 8:
        return "G#";
    case 9:
        return "A";
    case 10:
        return "A#";
    case 11:
        return "B";
    default:
        return "-";
    }
}

float compute_yin(float32_t sig[BUFFER_SIZE]) {
    log_trace("Computing yin algorithm");

    float32_t df[TAU_MAX];
    difference_function(sig, df);
    cumulative_mean_normalized_difference_function(df);

    size_t tau = get_pitch(df);
    if (tau == 0) {
        return 0.0f;
    }

    float32_t tauf = parabolic_interp(df, tau);

    return (float32_t)SR / tauf;
}

static void cumsum_f32(const float32_t *pSrc, float32_t *pDst, size_t src_len) {
    pDst[0] = 0;
    for (size_t i = 0; i < src_len; ++i)
        pDst[i + 1] = pDst[i] + pSrc[i];
}

static void difference_function(float32_t frame[W_LEN], float32_t df[TAU_MAX]) {
    static float32_t buf_a[FFT_LEN];
    static float32_t buf_b[FFT_LEN];

    arm_copy_f32(frame, buf_a, W_LEN);
    arm_fill_f32(0.0f, buf_a + W_LEN, FFT_LEN - W_LEN);

    arm_mult_f32(frame, frame, frame, W_LEN);

    static float32_t sum[W_LEN + 1];
    cumsum_f32(frame, sum, W_LEN);

    // Forward RFFT: Time domain (buf_a) -> Frequency domain (buf_b)
    arm_rfft_fast_f32(&rfft, buf_a, buf_b, 0);

    // Autocorrelation via Power Spectrum:
    // X(f) * conj(X(f)) = |X(f)|^2
    // DC (index 0) and Nyquist (index 1) are
    // strictly real in packed RFFT format
    buf_b[0] = buf_b[0] * buf_b[0];
    buf_b[1] = buf_b[1] * buf_b[1];

    // Complex frequency bins: (re + j*im) * (re - j*im) = (re^2 + im^2) + j*0
    for (size_t i = 2; i < FFT_LEN; i += 2) {
        float32_t re = buf_b[i];
        float32_t im = buf_b[i + 1];
        buf_b[i] = re * re + im * im; // Real part becomes magnitude squared
        buf_b[i + 1] = 0.0f;          // Imaginary part is strictly zero
    }

    // Inverse RFFT:
    // Frequency domain (buf_b) -> Autocorrelation time domain (buf_a)
    arm_rfft_fast_f32(&rfft, buf_b, buf_a, 1);

    // Calculate final Squared Difference Function
    const float32_t sum_total = sum[W_LEN];
    for (size_t i = 0; i < TAU_MAX; ++i) {
        df[i] = sum[W_LEN - i] + sum_total - sum[i] - (2.0f * buf_a[i]);
    }
}

static void
cumulative_mean_normalized_difference_function(float32_t df[TAU_MAX]) {
    static float32_t difference_sum[TAU_MAX];
    cumsum_f32(df + 1, difference_sum, TAU_MAX - 1);

    df[0] = 1;
    for (size_t i = 1; i < TAU_MAX; ++i) {
        if (difference_sum[i] > 0.00001f) {
            df[i] = df[i] * i / difference_sum[i];
        } else {
            df[i] = 1.0f; // YIN paper standard for zero-force
        }
    }
}

static size_t get_pitch(const float32_t cmndf[TAU_MAX]) {
    size_t tau = TAU_MIN;

    while (tau < TAU_MAX) {
        if (cmndf[tau] < HARMO_THRESH) {
            while (tau + 1 < TAU_MAX && cmndf[tau + 1] < cmndf[tau]) {
                ++tau;
            }
            return tau;
        }
        ++tau;
    }

    return 0; // if unvoiced
}

static float32_t parabolic_interp(const float32_t cmndf[TAU_MAX], size_t tau) {
    if (tau == 0 || tau == TAU_MAX - 1) {
        return (float32_t)tau;
    }

    float32_t offset =
        (cmndf[tau + 1] - cmndf[tau - 1]) /
        (2.0f * (2.0f * cmndf[tau] - cmndf[tau + 1] - cmndf[tau - 1]));

    return offset + (float32_t)tau;
}
