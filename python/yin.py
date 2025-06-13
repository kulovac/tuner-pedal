#!/usr/bin/env python
# pylint: disable=no-member


import time
from os import sep
import cmsisdsp as dsp
import matplotlib.pyplot as plt
import numpy as np
import soundfile as sf


def cumsum_f32(x: np.ndarray):
    cumsum = np.zeros(x.size + 1)
    for i in range(x.size):
        cumsum[i+1] = cumsum[i] + x[i]

    return cumsum


def next_pow2(n: int):
    n -= 1
    n |= n >> 1
    n |= n >> 2
    n |= n >> 4
    n |= n >> 8
    n |= n >> 16

    return n+1
    # return 2 ** n.bit_length()


def differenceFunction(x, N, tau_max, rfft):
    """
    Compute difference function of data x. This corresponds to equation (6) in [1]

    Fastest implementation. Use the same approach than differenceFunction_scipy.
    This solution is implemented directly with Numpy fft.


    :param x: audio data
    :param N: length of data
    :param tau_max: integration window size
    :param rfft: rfft instance from CMSIS
    :return: difference function
    :rtype: list
    """
    x = np.array(x, np.float32)

    # Make sure x is 1D
    if x.ndim != 1:
        raise ValueError("Input x must be a 1D array")

    w = x.size
    tau_max = min(tau_max, w)

    x_cumsum = cumsum_f32(dsp.arm_mult_f32(x, x))

    size = w + tau_max
    size_pad = next_pow2(size)

    x_padded = np.zeros(size_pad, dtype=np.float32)
    x_padded[:w] = x  # safely copy only available samples

    fc = dsp.arm_rfft_fast_f32(rfft, x_padded, 0)
    fc_conj = np.zeros(len(fc), dtype=np.float32)
    fc_conj[:2] = fc[:2]
    fc_conj[2:] = dsp.arm_cmplx_conj_f32(fc[2:])

    fft_conv = np.zeros(len(fc), dtype=np.float32)
    fft_conv[:2] = dsp.arm_mult_f32(fc[:2], fc_conj[:2])
    fft_conv[2:] = dsp.arm_cmplx_mult_cmplx_f32(fc[2:], fc_conj[2:])
    conv = dsp.arm_rfft_fast_f32(rfft, fft_conv, 1)[:tau_max]

    return x_cumsum[w:w - tau_max:-1] + x_cumsum[w] - x_cumsum[:tau_max] - 2 * conv


def cumulativeMeanNormalizedDifferenceFunction(df, N):
    """
    Compute cumulative mean normalized difference function (CMND).

    This corresponds to equation (8) in [1]

    :param df: Difference function
    :param N: length of data
    :return: cumulative mean normalized difference function
    :rtype: list
    """

    difference_sum = cumsum_f32(df[1:])

    cmndf = np.zeros(N, dtype=np.float32)
    cmndf[0] = 1
    for i in range(1, N):
        cmndf[i] = df[i] * i / difference_sum[i]

    return cmndf


def getPitch(cmdf, tau_min, tau_max, harmo_th=0.1):
    """
    Return fundamental period of a frame based on CMND function.

    :param cmdf: Cumulative Mean Normalized Difference function
    :param tau_min: minimum period for speech
    :param tau_max: maximum period for speech
    :param harmo_th: harmonicity threshold to determine if it is necessary to compute pitch frequency
    :return: fundamental period if there is values under threshold, 0 otherwise
    :rtype: float
    """
    tau = tau_min
    while tau < tau_max:
        if cmdf[tau] < harmo_th:
            while tau + 1 < tau_max and cmdf[tau + 1] < cmdf[tau]:
                tau += 1
            return tau
        tau += 1

    return 0    # if unvoiced


def compute_yin(sig, sr, dataFileName=None, w_len=512, w_step=256, f0_min=100, f0_max=500, harmo_thresh=0.1):
    """

    Compute the Yin Algorithm. Return fundamental frequency and harmonic rate.

    :param sig: Audio signal (list of float)
    :param sr: sampling rate (int)
    :param w_len: size of the analysis window (samples)
    :param w_step: size of the lag between two consecutives windows (samples)
    :param f0_min: Minimum fundamental frequency that can be detected (hertz)
    :param f0_max: Maximum fundamental frequency that can be detected (hertz)
    :param harmo_tresh: Threshold of detection. The yalgorithmù return the first minimum of the CMND fubction below this treshold.

    :returns:

        * pitches: list of fundamental frequencies,
        * harmonic_rates: list of harmonic rate values for each fundamental frequency value (= confidence value)
        * argmins: minimums of the Cumulative Mean Normalized DifferenceFunction
        * times: list of time of each estimation
    :rtype: tuple
    """

    print('Yin: compute yin algorithm')
    tau_min = int(sr / f0_max)
    tau_max = int(sr / f0_min)

    # time values for each analysis window
    timeScale = range(0, len(sig) - w_len, w_step)
    times = [t/float(sr) for t in timeScale]
    frames = [sig[t:t + w_len] for t in timeScale]

    pitches = [0.0] * len(timeScale)
    harmonic_rates = [0.0] * len(timeScale)
    argmins = [0.0] * len(timeScale)

    # Initialize rfft instance
    tau_max = min(tau_max, w_len)
    size_pad = next_pow2(tau_max + w_len)
    rfft = dsp.arm_rfft_fast_instance_f32()
    status = dsp.arm_rfft_fast_init_f32(rfft, size_pad)
    if status != 0:
        raise ValueError(
            f"Could not initialize rfft instance with a size pad: {size_pad}")

    for i, frame in enumerate(frames):

        # Compute YIN
        df = differenceFunction(frame, w_len, tau_max, rfft)
        cmdf = cumulativeMeanNormalizedDifferenceFunction(df, tau_max)
        p = getPitch(cmdf, tau_min, tau_max, harmo_thresh)

        # Get results
        if np.argmin(cmdf) > tau_min:
            argmins[i] = float(sr / np.argmin(cmdf))
        if p != 0:  # A pitch was found
            pitches[i] = float(sr / p)
            harmonic_rates[i] = cmdf[p]
        else:  # No pitch, but we compute a value of the harmonic rate
            harmonic_rates[i] = min(cmdf)

    if dataFileName is not None:
        np.savez(dataFileName, times=times, sr=sr, w_len=w_len, w_step=w_step, f0_min=f0_min, f0_max=f0_max,
                 harmo_thresh=harmo_thresh, pitches=pitches, harmonic_rates=harmonic_rates, argmins=argmins)
        print('\t- Data file written in: ' + dataFileName)

    return pitches, harmonic_rates, argmins, times


def main(audioFileName="whereIam.wav", w_len=1024, w_step=256, f0_min=50, f0_max=2000, harmo_thresh=0.3, audioDir="./", dataFileName=None, verbose=4):
    """
    Run the computation of the Yin algorithm on a example file.

    Write the results (pitches, harmonic rates, parameters ) in a numpy file.

    :param audioFileName: name of the audio file
    :type audioFileName: str
    :param w_len: length of the window
    :type wLen: int
    :param wStep: length of the "hop" size
    :type wStep: int
    :param f0_min: minimum f0 in Hertz
    :type f0_min: float
    :param f0_max: maximum f0 in Hertz
    :type f0_max: float
    :param harmo_thresh: harmonic threshold
    :type harmo_thresh: float
    :param audioDir: path of the directory containing the audio file
    :type audioDir: str
    :param dataFileName: file name to output results
    :type dataFileName: str
    :param verbose: Outputs on the console : 0-> nothing, 1-> warning, 2 -> info, 3-> debug(all info), 4 -> plot + all info
    :type verbose: int
    """

    sig, sr = sf.read("./samples/Alesis-Fusion-Clean-Guitar-C3.wav")

    # Convert to mono if stereo
    if sig.ndim > 1:
        sig = sig.mean(axis=1)

    start = time.time()
    pitches, harmonic_rates, argmins, times = compute_yin(
        sig, sr, dataFileName, w_len, w_step, f0_min, f0_max, harmo_thresh)
    end = time.time()
    print("Yin computed in: ", end - start)
    print("Yin computed time per frame: ", (end - start) / (sig.size // w_len))

    duration = len(sig)/float(sr)
    print("Sample collection time: ", duration)

    if verbose > 3:
        ax1 = plt.subplot(4, 1, 1)
        ax1.plot([float(x) * duration / len(sig)
                 for x in range(0, len(sig))], sig)
        ax1.set_title('Audio data')
        ax1.set_ylabel('Amplitude')
        ax2 = plt.subplot(4, 1, 2)
        ax2.plot([float(x) * duration / len(pitches)
                 for x in range(0, len(pitches))], pitches)
        ax2.set_title('F0')
        ax2.set_ylabel('Frequency (Hz)')
        ax3 = plt.subplot(4, 1, 3, sharex=ax2)
        ax3.plot([float(x) * duration / len(harmonic_rates)
                 for x in range(0, len(harmonic_rates))], harmonic_rates)
        ax3.plot([float(x) * duration / len(harmonic_rates) for x in range(0,
                 len(harmonic_rates))], [harmo_thresh] * len(harmonic_rates), 'r')
        ax3.set_title('Harmonic rate')
        ax3.set_ylabel('Rate')
        ax4 = plt.subplot(4, 1, 4, sharex=ax2)
        ax4.plot([float(x) * duration / len(argmins)
                 for x in range(0, len(argmins))], argmins)
        ax4.set_title('Index of minimums of CMND')
        ax4.set_ylabel('Frequency (Hz)')
        ax4.set_xlabel('Time (seconds)')
        plt.show()


if __name__ == '__main__':
    main()
