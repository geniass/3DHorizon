from sensor import SinusoidalMotion
from kalman import SinusoidalMotionKalmanFilter
import numpy as np
import matplotlib.pyplot as plt
import csv
from scipy.optimize import curve_fit


def rms(xs):
    return np.sqrt(np.sum(x**2 for x in xs)/len(xs))

"""
Computes a Fourier series with DC offset and the fundamental
frequency evaluated at x
f - fundamental freqency
a0 - DC offset
a1 - fundamental's magnitude
p1 - fundamental's phase
"""
def fourier1(x, a0, f1, a1, p1):
    ret = a0 + a1 * np.sin(2*np.pi * f1 * x + p1)
    return ret

def fourier2(x, a0, f1, a1, p1, f2, a2, p2):
    ret = a0 + a1 * np.sin(2*np.pi * f1 * x + p1) +\
            a2 * np.sin(2*np.pi*f2 * x + p2)
    return ret

np.random.seed(1233)

dt = 1 / 20
R = 0.1
q = 0.06


zs = []
with open('acc_unfilt.csv', 'r') as dataFile:
    reader = csv.reader(dataFile, delimiter=",")
    for l in reader:
        zs.append(float(l[0]))
N = len(zs)
zs = np.asarray(zs)

"""
Use Fourier series fitting to find sinusoid params
"""
t = np.arange(0, len(zs) * dt, dt)
# fit fourier1 to the acceleration data
# There must be a better way to do this.....
# To prevent local minima, do the curve fitting a few times
# with different initial values. Then use the best parameters
# (lowest mean variance)
popt, pcov = [], np.inf
def mean_cov(cov):
    return np.mean(np.diagonal(cov))

for f in np.arange(0.1, 2, 0.1):
    opt, cov = curve_fit(fourier1, t, zs, p0=[10, f, 1, 0])
    print('f:', f, mean_cov(cov), opt)
    if mean_cov(cov) < pcov:
        pcov = mean_cov(cov)
        popt = opt
print('Best fit: DC=%f, A=%f, f=%f, p=%f' % (popt[0], popt[2], popt[1], popt[3]))
plt.figure()
plt.plot(t, zs, linestyle='--')
plt.plot(t, fourier1(t, popt[0], popt[1], popt[2], popt[3]))
plt.show()

fft_motion = SinusoidalMotion(acc_mag=popt[2], f=popt[1], phase=popt[3],
                              dt=dt, noise_std=q, meas_noise_std=R)
# initial conditions
x0 = np.asarray((fft_motion.position(0), fft_motion.velocity(0), popt[0]))
print("Initial state:", x0)
print("Average acc mag.:", fft_motion.acc_mag)


"""
Apply Kalman filter
"""
# initial covariance matrix
# there is some uncertainty in the pos and vel
# but accel offset is fairly constrained
P = np.array([[10, 0, 0],
             [0, 10, 0],
             [0, 0, 1**2]])
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
