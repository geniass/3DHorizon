import time
import csv


class SerialMock:

    SAMPLE_RATE = 50

    def __init__(self, *args):
        self.prev_time = time.time()

        self.idx = 0
        with open('test_data.csv', 'r') as f:
            reader = csv.reader(f, delimiter = "," )
            self.data = []
            for row in reader:
                # = (float(i[3]),float(i[4]),float(i[5]))   #i[3] is roll, i[4] is pitch and [5] is yaw
                self.data.append([float(d) for d in row])

    def inWaiting(self):
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
    return bytes("IMU: %f %f %f %f %f %f\n" % tuple(d), 'utf-8')
