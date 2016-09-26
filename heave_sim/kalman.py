import numpy as np
import scipy.linalg
from filterpy.kalman import KalmanFilter


'''
Constructs a Kalman filter object for tracking sinusoidal motion,
given an acceleration measurement

R_std - stddev of acceleration measurement
Q - process noise covariance matrix
dt - sample time
w - sinusoid frequency (rad/s)
x0 - initial state vector
P - initial covariance matrix
'''
def SinusoidalMotionKalmanFilter(R_std, Q, dt, w, x0,
                                 P=100*np.eye(3)):
    # states: position, velocity, acceleration offset
    n_states = 3
    kf = KalmanFilter(dim_x=n_states, dim_z=1)
    kf.x = x0
    kf.P = P

    kf.R *= R_std**2
    kf.Q = Q

    # continuous-time state transition matrix
    A = np.array([[0,     1, 0],
                  [-w**2, 0, 0],
                  [0,     0, 0]])
    # discretize using matrix exponential
    kf.F = scipy.linalg.expm(A * dt)
    kf.H = np.array([[-w**2, 0, 1]])

    return kf
