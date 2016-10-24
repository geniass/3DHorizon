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
R = 2e-5
q = 0.00006
N_window = 10 * Fs

class MotionProcess(mp.Process):

    def __init__(self, imu, motion_queue):
        mp.Process.__init__(self)

        self.imu = imu
        self.motion_queue = motion_queue
        self.kf = None

        self.acc_buffer = deque([0] * N_window, maxlen=N_window)
        self.num_samples = 0

    def run(self):
        self.imu.start()

        pos = {'x': 0., 'y': 0., 'z': 0., 'roll': 0., 'pitch': 0., 'yaw': 0.}

        while True:
            state = self.imu.get_state()
            self.num_samples += 1
            self.acc_buffer.append(-state['z'])

        #    pos['z'] = self.acc_buffer[-1]

            if self.num_samples == N_window:
                # can also use % (modulo) to re-initialise periodically
                self.kf = SinusoidalMotionKalmanFilter(R, q, dt, num_modes, self.acc_buffer)

            if self.num_samples >= N_window:
                """
                Apply Kalman filter to most recent sample
                """
                x = self.kf.step(self.acc_buffer[-1])
                pos_sum = 100 * np.sum(x[0:-2:2], axis=0)
                pos['z'] = pos_sum

            pos['roll'] = state['roll']
            pos['pitch'] = state['pitch']
            pos['yaw'] = state['yaw']

            # only send new pos info if the queue is not full, otherwise the
            # queue will block
            #if self.motion_queue.full():
            self.motion_queue.put(pos)
