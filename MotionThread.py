from collections import deque
from heave_sim.kalman import SinusoidalMotionKalmanFilter
import numpy as np

from direct.stdpy import threading


# Kalman filter parameters
# number of sinusoidal modes expected in signal
num_modes = 1
Fs = 50
dt = 1 / Fs
R = 2e-5
q = 0.00006
N_window = 10 * Fs

class MotionThread(threading.Thread):

    def __init__(self, imu, motion_queue):
        threading.Thread.__init__(self)

        self.imu = imu
        self.motion_queue = motion_queue
        self.kf = None

        self.acc_buffer = deque([0] * N_window, maxlen=N_window)
        self.num_samples = 0

    def run(self):
        if not self.imu.is_alive():
            self.imu.start()

        pos = {'x': 0., 'y': 0., 'z': 0., 'roll': 0., 'pitch': 0., 'yaw': 0.}

        while True:
            if not self.imu.data_ready():
                continue

            state = self.imu.get_state()
            self.num_samples += 1
            self.acc_buffer.append(-state[2])

            if self.num_samples == N_window:
                # can also use % (modulo) to re-initialise periodically
                self.kf = SinusoidalMotionKalmanFilter(R, q, dt, num_modes, self.acc_buffer)

            if self.num_samples >= N_window:
                """
                Apply Kalman filter to most recent sample
                """
                x = self.kf.step(self.acc_buffer[-1])
                pos_sum = 100 * np.sum(x[0:-2:2], axis=0)
                #pos_sum = self.acc_buffer[-1]
                pos['z'] = pos_sum

            pos['roll'] = state[3]
            pos['pitch'] = state[4]
            pos['yaw'] = state[5]

            # only send new pos info if the queue is not full, otherwise the
            # queue will block
            #if self.motion_queue.full():
            self.motion_queue.put(pos)
