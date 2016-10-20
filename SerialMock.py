import time
import csv
import numpy as np

from serial.serialutil import SerialBase


'''
Fake Serial port class that loads a csv file and sends the data as if over a
real serial port. For some reason using this class causes a much larger delay
than a real serial port.
'''
class SerialMock(SerialBase):

    SAMPLE_RATE = 50

    def __init__(self):
        self.is_open = False

        SerialBase.__init__(self, port="serial_mock")

    def _reconfigure_port(self):
        return

    def open(self):
        self.prev_time = time.time()

        self.idx = 0
        with open('test_data.csv', 'r') as f:
            i = 0
            reader = csv.reader(f, delimiter = "," )
            self.data = []
            for row in reader:
                self.data.append([float(d) for d in row])
                self.data[-1][2] = np.sin(2*np.pi*1/SerialMock.SAMPLE_RATE*i)
                i += 1
        self.is_open = True

    def close(self):
        self.is_open = False

    @property
    def in_waiting(self):
        if time.time() - self.prev_time > 1/SerialMock.SAMPLE_RATE:
            return len(serial_string(self.data[self.idx]))
        else:
            return 0

    def read(self, num_bytes):
        if num_bytes == 0:
            return b''

        self.prev_time = time.time()
        s = serial_string(self.data[self.idx])
        if self.idx + 1 < len(self.data):
            self.idx += 1
        else:
            self.idx = 0
        return s

def serial_string(d):
    return bytes(":%f %f %f %f %f %f\n" % tuple(d), 'utf-8')
