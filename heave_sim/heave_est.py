from kalman import SinusoidalMotionKalmanFilter
from kalman import find_mode_params
import numpy as np
import matplotlib.pyplot as plt
import csv
from peakutils.plot import plot as pplot
import math


np.random.seed(1233)

# number of sinusoidal modes expected in signal
num_modes = 1

dt = 1 / 250
R = 0.01
q = 0.00001


zs = []
# 'platform_data/11-10-heave.csv
with open('platform_data/31-10-heave2.csv', 'r') as dataFile:
    reader = csv.reader(dataFile, delimiter=",")
    for l in reader:
        zs.append(-float(l[2]))
N = len(zs)
zs = np.asarray(zs)

"""
Use FFT and peak detection to model the data
"""
modes = find_mode_params(zs, 1/dt, num_modes)

# plot reconstructed signal
acc = zs - np.mean(zs)
t = np.linspace(0., (len(acc)-1)*dt, len(acc))
sig = np.zeros(len(t))
for m in modes:
    if m['y'] < 0.1:
        continue
    sig += m['y'] * np.cos(2*np.pi*m['f']*t + m['p'])
    print("f=%f: A=%f, p=%f" % (m['f'], m['y'], m['p']))
plt.plot(t,sig,t,acc)
plt.show()


"""
Apply Kalman filter
"""
kf = SinusoidalMotionKalmanFilter(R, q, dt, num_modes, zs)
xests, variances, ps = [], [], []
for z in zs:
    xests.append(kf.step(z))
xests = np.asarray(xests)


xpred = []
for k in range(1, N):
    #print(kf.predict(1).shape)
    xpred.append(kf.predict(k))
xpred = np.asarray(xpred)

"""
Visualisations
"""
# states and measurements
state_fig = plt.figure()
ax1 = state_fig.add_subplot(311)
pos_sum = np.sum(xests[:, 0:-1:2], axis=1)
zeros = np.zeros(N-1)
print(np.sum(xpred[:, 0:-1:2], axis=1).shape)
pos_pred_sum = np.concatenate((pos_sum, np.sum(xpred[:, 0:-1:2], axis=1)))
plt.plot(range(N), pos_sum, label='Estimated')
#plt.plot(range(2*N-1), pos_pred_sum)
ax1.set_title('Estimated Position')

ax_acc = state_fig.add_subplot(312, sharex=ax1)
plt.plot(range(len(zs)), zs)
ax_acc.set_title('Measured Acceleration')

ax_off = state_fig.add_subplot(313, sharex=ax1)
plt.plot(range(N), xests[:, -1], label='Estimated')
ax_off.set_title('$x_2[k]$: Acceleration Offset')

'''
ax1 = state_fig.add_subplot(414)
print(xpred)
pos_sum = np.sum(xpred[:, 0:-1:2], axis=1)
plt.plot(range(N), pos_sum, label='predicted')
ax1.set_title('Estimated Position')
'''

plt.show()

print('Kalman Gains:', np.asarray(kf.kf.K))


# save the results
np.savetxt('kalman_output.csv', np.column_stack((pos_sum, zs)), delimiter=',')
