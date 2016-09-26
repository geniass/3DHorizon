from sensor import SinusoidalMotion
from kalman import SinusoidalMotionKalmanFilter
from filterpy.stats import NESS
import numpy as np
from numpy.random import randn
import matplotlib.pyplot as plt
import book_plots as bp
import peakutils
import peakdetect
from scipy import signal

np.random.seed(1233)

dt = 1 / 20
R = 0.01
q = 0.06

"""
Generate some data
"""
motion = SinusoidalMotion(dt=dt, noise_std=q, meas_noise_std=R)
N = int(10/dt)
acc = np.zeros((N))
xs, zs = np.zeros((N, 2)), np.zeros((N))
for k in range(N):
    acc[k] = motion.acceleration(k) + randn() * 0.05
    xs[k, 1] = motion.velocity(k)
    xs[k, 0] = motion.position(k)

# initial conditions
# x0 = np.asarray((xs[0, 0], xs[0, 1], motion.acc_offset))
zs = acc + randn(N) * R + motion.acc_offset


"""
Use FFT and peakdetect to find sinusoid parameters
"""
n = 2**12
n_window = int(len(zs)/2)
window = signal.blackmanharris(n_window)
z_fft = np.fft.rfft(zs[0:n_window] * window, n=n) * 4 / n_window
z_fft_mag = np.absolute(z_fft)
f = np.fft.rfftfreq(n, d=dt)

# peak detection
# this stuff isn't working
# peaks = peakdetect_parabola(z_fft_mag, f)
# print(peaks)
# indices = peakutils.indexes(z_fft_mag, thres=0.1, min_dist=1)

# this one works
maxtab, _ = peakdetect.peakdet(z_fft_mag, 0.01*np.max(z_fft_mag), range(int(n/2)+1))
plt.figure()
plt.plot(f, z_fft_mag)
plt.scatter(f[np.array(maxtab)[:, 0].astype(int)], np.array(maxtab)[:, 1], color='blue')
plt.show()

print(f[np.array(maxtab)[0, 0].astype(int)])

fft_motion = SinusoidalMotion(acc_mag=np.array(maxtab)[1, 1], f=f[np.array(maxtab)[1, 0].astype(int)], dt=dt, noise_std=q, meas_noise_std=R)
# initial conditions
x0 = np.asarray((fft_motion.position(0), fft_motion.velocity(0), -np.array(maxtab)[0, 1]))
print(x0)
print(fft_motion.acc_mag)
"""
Apply Kalman filter
"""
# initial covariance matrix
# there is some uncertainty in the pos and vel
# but accel offset is fairly constrained
P = np.array([[1000, 0, 0],
             [0, 1000, 0],
             [0, 0, 100**2]])
# this matrix is basically trial and error (i.e. guessing)
# however the accel offset is not expected to change much
Q = np.array([[(q/10)**2, 0, 0],
             [0, q**2, 0],
             [0, 0, (0.01)**2]])
print('Process Noise (Q):', Q)
kf = SinusoidalMotionKalmanFilter(R, Q, dt, fft_motion.w, x0, P)
xests, variances, ps = [], [], []
for z in zs:
    kf.predict()
    kf.update(z)
    xests.append(kf.x)
    # variances
    variances.append(kf.P.diagonal())
    ps.append(kf.P)
xests = np.asarray(xests)
variances = np.asarray(variances)

"""
Visualisations
"""
# states and measurements
state_fig = plt.figure()
ax1 = state_fig.add_subplot(411)
plt.plot(range(len(xs[:, 0])), xs[:, 0], linestyle='--', label='Ideal')
plt.plot(range(N), xests[:, 0], label='Estimated')
ax1.set_title('$x_0[k]$: Position')

ax_vel = state_fig.add_subplot(412, sharex=ax1)
plt.plot(range(len(xs[:, 1])), xs[:, 1], linestyle='--', label='Ideal')
plt.plot(range(N), xests[:, 1], label='Estimated')
ax_vel.set_title('$x_1[k]$: Velocity')

ax_acc = state_fig.add_subplot(413, sharex=ax1)
plt.plot(range(len(zs)), zs)
ax_acc.set_title('Measured Acceleration')

ax_off = state_fig.add_subplot(414, sharex=ax1)
plt.plot(range(N), np.ones(N) * motion.acc_offset, linestyle='--', label='Ideal')
plt.plot(range(N), xests[:, 2], label='Estimated')
ax_off.set_title('$x_2[k]$: Acceleration Offset')

plt.show()


# plot residuals and theoretical bounds to evaluate performance
res = xs[:, 0] - xests[:, 0]
plt.figure()
plt.plot(res)
# 3 std devs
bp.plot_residual_limits(variances[:, 0], 3)
bp.set_labels('Heave position residuals (3$\sigma$)', 'time (s)', 'meters')
plt.show()

# NESS calculation
true_states = np.zeros((N, 3))
true_states[:, 0:1] = xs[:, 0:1]
true_states[:, 2] = np.ones((N)) * motion.acc_offset
ness = NESS(true_states, xests, ps)
print('mean NEES: ', np.mean(ness))

print('Kalman Gains:', np.asarray(kf.K))
