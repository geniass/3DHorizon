#!/bin/env python3
import serial
import sys
import argparse
from TI_IMU import TI_IMU

parser = argparse.ArgumentParser(
    description='Receive data from the IMU and save to a file')
parser.add_argument('comport', help='the serial port to listen on')
parser.add_argument('baud', type=int, help='the serial port baud rate')
parser.add_argument('-o', '--outfile', help='file to use as the output file')

args = parser.parse_args()

if not args.outfile:
    outfile = sys.stdout
else:
    outfile = open(args.outfile, 'w')


with serial.Serial(args.comport, args.baud) as ser:
    imu = TI_IMU(ser)
    imu.start()
    while True:
        # get_state is blocking so it will only be called when data is avaiable
        s = imu.get_state()
        strline = ",".join([str(v) for v in [s['x'], s['y'], s['z'], s['roll'], s['pitch'], s['yaw']]])

        outfile.write(strline + "\n")
        outfile.flush()

outfile.close()
