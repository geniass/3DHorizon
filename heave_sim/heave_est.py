from sensor import SinusoidalMotion
from kalman import SinusoidalMotionKalmanFilter
import numpy as np
import matplotlib.pyplot as plt
import csv
import peakutils
from peakutils.plot import plot as pplot
import math
import numpy
from spectrum import spectrum


np.random.seed(1233)

# number of sinusoidal modes expected in signal
num_modes = 3

dt = 1 / 50
R = 0.01
q = 0.06


zs = []
# 'temp_data/10-07-heave.csv'
with open('platform_data/11-10-heave.csv', 'r') as dataFile:
    reader = csv.reader(dataFile, delimiter=",")
    for l in reader:
        zs.append(float(l[2]))
N = len(zs)
zs = np.asarray(zs)

"""
Use FFT and peak detection to model the data
"""
# Need to detrend the data
acc = zs - np.mean(zs)
y, p, f = spectrum(acc, 1/dt, len(acc))

# find num_modes peaks in FFT, sorted by magnitude
y_limited = y
y_limited[:10] = 0
indexes = peakutils.peak.indexes(y_limited, thres=0.1, min_dist=np.ceil(0.1/dt/N))
peaks = sorted(zip(indexes, f[indexes], y[indexes], p[indexes]), key=lambda p: p[2], reverse=True)[:num_modes]
#indexes, _, _,_ = zip(*peaks)
#indexes = list(indexes)
interp_freqs = peakutils.interpolate(f, y_limited, ind=[p[0] for p in peaks])
interp_peaks = zip(interp_freqs, y[indexes], p[indexes])
pplot(f, y_limited, [p[0] for p in peaks])
plt.show()

# plot reconstructed signal
t = np.linspace(0., (len(acc)-1)*dt, len(acc))
sig = y[0] * np.ones(len(t))
for i in indexes[:num_modes]:
#for i in indexes:
    sig += y[i] * np.cos(2*np.pi*f[i]*t + p[i])
    print("f=%f: A=%f, p=%f" % (f[i], y[i], p[i]))
interp_sig = y[0] * np.ones(len(t))
for peak in interp_peaks:
    interp_sig += peak[1] * np.cos(2*np.pi*peak[0]*t + peak[2])
    print("interp: f=%f: A=%f, p=%f" % (peak[0], peak[1], peak[2]))
plt.plot(t,sig,t,acc)
plt.show()

# get initial conditions for each mode
# num_states = 2 * num_modes + 1
x0 = []
ws = []
for idx in indexes[:num_modes]:
    fft_motion = SinusoidalMotion(acc_mag=y[idx], f=f[idx], phase=p[idx], dt=dt)
    # pos and vel for each mode
    x0_m = [fft_motion.position(0), fft_motion.velocity(0)]
    x0.extend(x0_m)

    ws.append(2*np.pi*f[idx])
# add final offset state
x0.append(np.mean(zs))

print("Initial state:", x0)
#print("Average acc mag.:", fft_motion.acc_mag)


"""
Apply Kalman filter
"""
# initial covariance matrix
# there is some uncertainty in the pos and vel
# but accel offset is fairly constrained
P = np.eye(2*num_modes + 1) * 10
P[-1,-1] = 1

# this matrix is basically trial and error (i.e. guessing)
# diagonals are the process noise variance for each state variable
# higher frequency modes have a higher process noise variance
# however the accel offset is not expected to change much
Q = np.eye(2*num_modes+1)
Q[range(0, 2*num_modes, 2), range(0, 2*num_modes, 2)] = ((q/10)**2) * np.arange(1., num_modes+1)
Q[range(1, 2*num_modes, 2), range(1, 2*num_modes, 2)] = (q**2) * np.arange(1., num_modes+1)
Q[-1, -1] = (0.01)**2
print('Process Noise (Q):', numpy.array_str(Q, max_line_width=1000000))

kf = SinusoidalMotionKalmanFilter(R, Q, dt, ws, x0, P)
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
pos_sum = np.sum(xests[:, 0:-1:2], axis=1)
plt.plot(range(N), pos_sum, label='Estimated')
ax1.set_title('Estimated Position')

ax_acc = state_fig.add_subplot(312, sharex=ax1)
plt.plot(range(len(zs)), zs)
ax_acc.set_title('Measured Acceleration')

ax_off = state_fig.add_subplot(313, sharex=ax1)
plt.plot(range(N), xests[:, -1], label='Estimated')
ax_off.set_title('$x_2[k]$: Acceleration Offset')

plt.show()

print('Kalman Gains:', np.asarray(kf.K))


# save the results
np.savetxt('kalman_output.csv', np.column_stack((pos_sum, zs)), delimiter=',')
