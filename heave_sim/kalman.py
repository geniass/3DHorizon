import numpy as np
import scipy.linalg
from filterpy.kalman import KalmanFilter
from filterpy.common import Q_discrete_white_noise


def SinusoidalMotionKalmanFilter(R_std, Q, dt, w, P=100):
    n_states = 2
    kf = KalmanFilter(dim_x=n_states, dim_z=1)
    kf.x = np.zeros(n_states)
    kf.P[0, 0] = P
    kf.P[1, 1] = P
    kf.R *= R_std**2
    kf.Q = Q_discrete_white_noise(2, dt, Q)
    # this approximation actually works better than
    # Q_discrete_white_noise for some reason...
    kf.Q = np.array([[0, 0],
                     [0, Q]])
    print(kf.Q)
    A = np.array([[0,     1],
                  [-w**2, 0]])
    kf.F = scipy.linalg.expm(A * dt)
    kf.H = np.array([[-w**2, 0]])
    return kf
