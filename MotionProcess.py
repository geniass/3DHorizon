from threading import Event
from collections import deque
from heave_sim.kalman import SinusoidalMotionKalmanFilter
import numpy as np

import multiprocessing as mp


# Kalman filter parameters
# number of sinusoidal modes expected in signal
num_modes = 1
Fs = 250
dt = 1 / Fs
R = 0.01
q = 0.00006
# num samples required for 0.05Hz resolution
N_window = int(Fs / 0.05)


class MotionProcess(mp.Process):

    '''
    Must supply this class with a valid TI_IMU instance
    as well as dict created by multiprocessing.Manager()
    '''

    def __init__(self, imu, manager_pos_dict):
        mp.Process.__init__(self)

        self.motion_event = mp.Event()

        self.imu = imu
        self.kf = None
        self.pos = manager_pos_dict
        self.pos['x'] = 0.
        self.pos['y'] = 0.
        self.pos['z'] = 0.
        self.pos['roll'] = 0.
        self.pos['pitch'] = 0.
        self.pos['yaw'] = 0.

        self.acc_buffer = deque([0] * N_window, maxlen=N_window)
        self.num_samples = 0

    def run(self):
        self.imu.start()

        while True:
            state = self.imu.get_state()
            self.num_samples += 1
            self.acc_buffer.append(-state['z'])

        #    self.pos['z'] = self.acc_buffer[-1]

            if self.num_samples == N_window:
                # can also use % (modulo) to re-initialise periodically
                self.kf = SinusoidalMotionKalmanFilter(
                    R, q, dt, num_modes, self.acc_buffer)

            if self.num_samples >= N_window:
                """
                Apply Kalman filter to most recent sample
                """
                x = self.kf.step(self.acc_buffer[-1])
                pos_sum = 100 * np.sum(x[0:-2:2], axis=0)
                self.pos['z'] = pos_sum

            self.pos['roll'] = state['roll']
            self.pos['pitch'] = state['pitch']
            self.pos['yaw'] = state['yaw']

            self.motion_event.set()

    def data_ready(self):
        return self.motion_event.is_set()
