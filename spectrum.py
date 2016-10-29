import numpy as np
from scipy import fft


'''
Computes the real, Single-Sided Fourier transform for the data in d, with
sampling rate Fs. No padding is performed so that the phase spectrum is preserved
'''
def spectrum(d, Fs, N):
    # pad with (N-M) zeros in center of waveform to preserve phase spectrum
    M = len(d)
    Mo2 = (M-1)//2;
    w = np.blackman(M);
    zw = w * d[0:M];
    # Zero-padding causes phase spectrum problems, so much easier to get more
    # data if better resolution is required
    # zw_zp = np.hstack((zw[Mo2:M], np.zeros(N - M), zw[0:Mo2]));
    zw_zp = zw

    y = fft(zw_zp, N) / M

    # set values below threshold to 0 to avoid noisy phase spectrum
    thresh = np.max(np.abs(y))/10e3
    y2 = y
    y2[np.abs(y) < thresh] = 0

    p = np.unwrap(np.angle(y2))
    p = p[0:N//2]

    y = np.abs(y[0:N//2])
    y[1:] = 2*y[1:]
    f = Fs * np.arange(0., N//2) / N

    return (y, p, f)
