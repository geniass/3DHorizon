import numpy as np
import scipy.linalg
from filterpy.kalman import KalmanFilter
from heave_sim.sensor import SinusoidalMotion
from spectrum import spectrum
import peakutils
import time
# import matplotlib.pylab as plt


'''
Find the parameters for n_modes sinusoids from the acceleration data in acc
with sample rate Fs
Returns: [{i, f, y, p}, ...], where y=magnitude, p=phase
'''


def find_mode_params(acc, Fs, n_modes):
    N = len(acc)

    # some peak finding parameters
    thresh = 0.5
    min_dist = np.ceil(0.1 * Fs / N)    # 0.1 Hz

    # detrend the data before FFT
    acc = acc - np.mean(acc)
    # use Fs/0.01 samples to get 0.01 Hz resolution
    N = int(np.power(2, np.ceil(np.log2(Fs / 0.01))))
    y, p, f = spectrum(acc, Fs, N)
    # plt.plot(f,20*np.log10(y))
    # plt.show()

    # ignore the first 0.05Hz to ignore very low frequency peaks
    y_limited = y
    y_limited[:int(0.1 / (Fs / N))] = 0
    y_limited[int(1 / (Fs / N)):] = 0

    # find n_modes peaks in FFT, sorted by magnitude
    indexes = peakutils.peak.indexes(
        y_limited, thres=thresh, min_dist=min_dist)
    peaks = zip(indexes, f[indexes], y[indexes], p[indexes])
    # convert to dict so elements can't be confused
    peaks = [dict(zip(('i', 'f', 'y', 'p'), p)) for p in peaks]
    peaks = sorted(peaks, key=lambda p: p['f'], reverse=False)[:n_modes]
    # frequency interpolation. may not be necessary
    #indexes, _, _,_ = zip(*peaks)
    #indexes = list(indexes)
    # interp_freqs = peakutils.interpolate(f, y_limited, ind=[p[0] for p in peaks])
    #interp_peaks = zip(interp_freqs, y[indexes], p[indexes])

    peaks = filter(lambda p: p['y'] >= 0.1, peaks)

    return list(peaks)


def get_initial_conditions(modes, acc_train_data, dt):
    # num_states = 2 * n_modes + 1
    x0 = []
    for m in modes:
        fft_motion = SinusoidalMotion(
            acc_mag=m['y'], f=m['f'], phase=m['p'], dt=dt)
        # pos and vel for each mode
        x0_m = [fft_motion.position(0), fft_motion.velocity(0)]
        x0.extend(x0_m)
    # add final offset state
    x0.append(np.mean(acc_train_data))

    return x0


class SinusoidalMotionKalmanFilter:

    '''
    Constructs a Kalman filter object for tracking sinusoidal motion,
    given an acceleration measurement

    r - stddev of acceleration measurement
    q - process noise stddev
    dt - sample time
    n_modes - expected number of sinusoidal modes present in signal
    acc_train_data - acceleration data used to indentify the sinusoidal mode
        parameters
    '''

    def __init__(self, r, q, dt, n_modes, acc_train_data):
        self.n_modes = n_modes

        modes = find_mode_params(acc_train_data, 1 / dt, n_modes)

        if len(modes) == 0:
            n_modes = len(modes)

        # eigenfrequencies for each mode
        ws = [2 * np.pi * m['f'] for m in modes]

        x0 = get_initial_conditions(modes, acc_train_data, dt)

        # n_states = n_modes * 2 + 1
        n_states = len(x0)
        assert(n_modes == (n_states - 1) // 2)

        kf = KalmanFilter(dim_x=n_states, dim_z=1)
        kf.x = x0

        # initial covariance matrix
        # there is some uncertainty in the pos and vel
        # but accel offset is fairly constrained
        kf.P = np.eye(2 * n_modes + 1) * 10
        kf.P[-1, -1] = 1

        kf.R *= r**2

        # this matrix is basically trial and error (i.e. guessing)
        # diagonals are the process noise variance for each state variable
        # higher frequency modes have a higher process noise variance
        # however the accel offset is not expected to change much
        kf.Q = np.eye(2 * n_modes + 1)
        kf.Q[range(0, 2 * n_modes, 2), range(0, 2 * n_modes, 2)
             ] = ((q / 10)**2) * np.arange(1., n_modes + 1)
        kf.Q[range(1, 2 * n_modes, 2), range(1, 2 * n_modes, 2)
             ] = (q**2) * np.arange(1., n_modes + 1)
        kf.Q[-1, -1] = (0.01)**2
        print('Process Noise (Q):\n', np.array_str(
            kf.Q, max_line_width=1000000), '\n')

        # continuous-time state transition matrix
        # each mode causes a submatrix as defined in state_sub_mat
        # to appear along the diagonals of A, surrounded by zeros
        def state_sub_mat(w):
            return np.array([[0, 1],
                             [-w**2, 0]])
        A = np.zeros((n_states, n_states))
        for i in range(n_modes):
            A[2 * i:2 * i + 2, 2 * i:2 * i + 2] = state_sub_mat(ws[i])
        print('Transition matrix (A):\n', np.array_str(
            A, max_line_width=1000000), '\n')
        # discretize using matrix exponential
        kf.F = scipy.linalg.expm(A * dt)
        print(kf.F)

        # construct measurement matrix
        # Each row represents the state of one mode: [-w**2, 0].
        # Then the matrix is flattened to the correct size and the offset is
        # appended
        H = np.transpose(
            np.vstack((-np.square(ws), np.zeros_like(ws)))).ravel()
        kf.H = np.append(H, 1.)
        kf.H = np.asarray([kf.H])
        print('Measurement matrix (H):\n', np.array_str(
            kf.H, max_line_width=1000000), '\n')

        self.kf = kf

        self.print_skip = 0

    def step(self, z):
        t = time.time()
        self.kf.predict()
        self.kf.update(z)
        dt = time.time() - t
        self.print_skip += 1
        if self.print_skip >= 10:
            # print("dt: %f" %(dt))
            self.print_skip = 0
        return self.kf.x

    def predict(self, N):
        '''
        Use the current identified state to predict the motion N timesteps
        forward in time, based on the current state.
        Returns the predicted state matrix N steps in the future.
        '''
        assert(N > 0)
        return np.linalg.matrix_power(self.kf.F, N).dot(self.kf.x)
