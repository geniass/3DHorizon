from sensor import SinusoidalMotion
from kalman import SinusoidalMotionKalmanFilter
import numpy as np
import matplotlib.pyplot as plt
import peakdetect
from scipy import signal
import csv

np.random.seed(1233)

dt = 1 / 20
R = 0.1
q = 0.06


zs = []
with open('filt_acc.csv', 'r') as dataFile:
    reader = csv.reader(dataFile, delimiter=",")
    for l in reader:
        zs.append(float(l[0]))
N = len(zs)
zs = np.asarray(zs)

"""
Use FFT and peakdetect to find sinusoid parameters
"""
n = 2**16
n_window = int(len(zs)/2)
window = signal.blackmanharris(n_window)
z_fft = np.fft.rfft(zs[0:n_window] * window, n=n) * 4 / n_window
z_fft_mag = np.absolute(z_fft)
f = np.fft.rfftfreq(n, d=dt)


# this one works
maxtab, _ = peakdetect.peakdet(z_fft_mag, 0.01*np.max(z_fft_mag), range(int(n/2)+1))
plt.figure()
plt.plot(f, z_fft_mag)
plt.scatter(f[np.array(maxtab)[:, 0].astype(int)], np.array(maxtab)[:, 1], color='blue')
plt.show()

print("Peak frequencies:", f[np.array(maxtab)[:, 0].astype(int)])

fft_motion = SinusoidalMotion(acc_mag=np.array(maxtab)[1, 1], f=f[np.array(maxtab)[1, 0].astype(int)], dt=dt, noise_std=q, meas_noise_std=R)
# initial conditions
x0 = np.asarray((fft_motion.position(0), fft_motion.velocity(0), np.array(maxtab)[0, 1]))
print("Initial state:", x0)
print("Average acc mag.:", fft_motion.acc_mag)
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
ax1 = state_fig.add_subplot(311)
plt.plot(range(N), xests[:, 0], label='Estimated')
ax1.set_title('$x_0[k]$: Position')

ax_acc = state_fig.add_subplot(312, sharex=ax1)
plt.plot(range(len(zs)), zs)
ax_acc.set_title('Measured Acceleration')

ax_off = state_fig.add_subplot(313, sharex=ax1)
plt.plot(range(N), xests[:, 2], label='Estimated')
ax_off.set_title('$x_2[k]$: Acceleration Offset')

plt.show()

print('Kalman Gains:', np.asarray(kf.K))
