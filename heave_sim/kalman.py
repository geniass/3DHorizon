import numpy as np
import scipy.linalg
from filterpy.kalman import KalmanFilter


'''
Constructs a Kalman filter object for tracking sinusoidal motion,
given an acceleration measurement

R_std - stddev of acceleration measurement
Q - process noise covariance matrix
dt - sample time
ws - list of sinusoid frequencies (rad/s)
x0 - initial state vector (pos and vel for each mode, and offset)
P - initial covariance matrix
'''
def SinusoidalMotionKalmanFilter(R_std, Q, dt, ws, x0, P):
    # states: (pos and vel for each mode, and offset)
    n_states = len(x0)
    n_modes = len(ws)

    # n_states = num_modes * 2 + 1
    assert(n_modes == (n_states-1)//2)

    kf = KalmanFilter(dim_x=n_states, dim_z=1)
    kf.x = x0
    kf.P = P

    kf.R *= R_std**2
    kf.Q = Q

    def state_sub_mat(w):
        return np.array([[0    , 1],
                          [-w**2, 0]])

    # continuous-time state transition matrix
    # each mode causes a submatrix as defined in state_sub_mat
    # to appear along the diagonals of A, surrounded by zeros
    A = np.zeros((n_states, n_states))
    for i in range(n_modes):
        A[2*i:2*i+2, 2*i:2*i+2] = state_sub_mat(ws[i])
    print(A)
    # discretize using matrix exponential
    kf.F = scipy.linalg.expm(A * dt)

    # construct measurement matrix
    # Each row represents the state of one mode: [-w**2, 0].
    # Then the matrix is flattened to the correct size and the offset is appended
    H = np.transpose(np.vstack((-np.square(ws), np.zeros_like(ws)))).ravel()
    print(np.transpose(np.vstack((-np.square(ws), np.zeros_like(ws)))).ravel())
    kf.H = np.append(H, 1.)
    kf.H = np.asarray([kf.H])
    print("H: ", (kf.H))

    return kf
