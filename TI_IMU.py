#!/bin/env python3

import threading

class TI_IMU (threading.Thread):
    LINE_START = "IMU:"

    def __init__(self, comport):
        threading.Thread.__init__(self)
        self.comport = comport

        self.x = self.y = self.z = 0
        self.roll = self.pitch = self.yaw = 0

        self.last_received = ""

        self.condition = threading.Event()

    def run(self):
        buffer_string = ''
        while True:
            buffer_string = buffer_string + \
                bytes.decode(self.comport.read(self.comport.inWaiting()))
            if '\n' in buffer_string:
                lines = buffer_string.split('\n')
                self.last_received = lines[-2]
                buffer_string = lines[-1]

                self.update()

    def update(self):
        start_idx = -1
        line = ""
        while start_idx < 0:
            line = self.last_received
            start_idx = line.find(TI_IMU.LINE_START)
            if start_idx < 0:
                continue
            line = line[start_idx:]
        vals = [float(v) for v
                in line.split(' ')
                if v.find(TI_IMU.LINE_START) < 0]

        if len(vals) == 6:
            self.x, self.y, self.z, self.roll, self.pitch, self.yaw = tuple(
                vals)
            self.condition.set()

    def data_ready(self):
        return self.condition.is_set()

    def get_state(self):
    #    self.condition.wait()
        self.condition.clear()
        return (self.x, self.y, self.z,
                self.roll,  self.pitch, self.yaw)
