from sensor import SinusoidalMotion
from kalman import SinusoidalMotionKalmanFilter
from filterpy.stats import NESS
import numpy as np
import matplotlib.pyplot as plt
import book_plots as bp

np.random.seed(1233)

dt = 1 / 20
R = 0.1
Q = 0.0006

"""
Generate some data
"""
motion = SinusoidalMotion(dt=dt, noise_std=Q, meas_noise_std=R)
xs, zs = [], []
for k in range(5000):
    x = motion.update()
    z = motion.sense()
    xs.append(x)
    zs.append(z)
xs = np.asarray(xs)
zs = np.asarray(zs)

"""
Apply Kalman filter
"""
kf = SinusoidalMotionKalmanFilter(R, Q, dt, motion.w)
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
plt.figure()
plt.plot(range(len(zs)), zs)
plt.show()

xs = np.asarray(xs)
plt.figure()
plt.plot(range(len(zs)), xs[:, 0], linestyle='--')
plt.plot(range(len(zs)), xests[:, 0])
plt.show()

# plot residuals and theoretical bounds to evaluate performance
res = xs[:, 0] - xests[:, 0]
plt.figure()
plt.plot(res)
# 3 std devs
bp.plot_residual_limits(variances[:, 0], 3)
bp.set_labels('Heave position residuals (3$\sigma$)', 'time (s)', 'meters')
plt.show()

nees = NESS(xs[:, 0:1], xests, ps)
print('mean NEES: ', np.mean(nees))
