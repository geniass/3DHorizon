#!/bin/env python3

from serial.threaded import ReaderThread, FramedPacket
from queue import Queue

class TI_IMU:

    class IMUFramedPacket(FramedPacket):
        START = b"("
        STOP = b")"

        def __init__(self, packet_queue):
            FramedPacket.__init__(self)
            self.packet_queue = packet_queue

        def __call__(self):
            return self

        def handle_packet(self, packet):
            vals = []
            try:
                vals = [float(v.strip()) for v
                        in (packet).split()]
            except ValueError:
                # sometimes the packet is corrupted and can't be parsed
                # This usually only happens once during startup, so just ignore this packet
                pass
            else:
                if len(vals) == 6:
                    state = dict(zip(['x', 'y', 'z', 'roll', 'pitch', 'yaw'],
                                        vals))
                    self.packet_queue.put(state)

        def handle_out_of_packet_data(self, data):
            print("OP: " + bytes.decode(data))


    def __init__(self, comport):
        self.comport = comport

        self.packet_queue = Queue(maxsize=0)

        self.protocol = TI_IMU.IMUFramedPacket(self.packet_queue)
        self.reader_thread = ReaderThread(self.comport, self.protocol)

    def start(self):
        self.reader_thread.start()

    def get_state(self):
        return self.packet_queue.get()
