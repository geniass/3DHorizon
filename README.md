# 3D Artificial horizon for countering motion sickness

In addition to the Python code for the 3D artificial horizon, this repo contains code for the following:
* serial communication with a microcontroller
* data logging script
* heave position estimation algorithm

## Source code breakdown:
* main*.py - main artificial horizon Python scripts for different environments
* TI_IMU.py - Serial interface for TI TM4C123 microcontroller
* SerialMock.py - mock serial port (drop-in replacement) so artificial horizon can be tested without the microcontroller attached
* MotionProcess.py - runs the serial interface and heave estimation procedure in a seperate process
* heave_sim/heave_est.py - runs a simulation of the heave estimation algorithm using data previously recorded to a csv file. Useful for parameter tuning
* heave_sim/kalman.py - Kalman filter definition and FFT identification procedure

## Requirements:
* Panda3D (1.9.1) - can be installed somewhere in the system path
  * if you are using a virtualenv, you need to copy the `panda3d.pth` file from your `site-packages` directory (depending on your version of Python etc.) into the virtualenv's `site-packages` directory. There's probably a better way of doing this
* run `pip install -r requirements.txt` to install the required libraries
  * `filterpy` - Kalman filter library
  * `matplotlib` - graph plotting
  * `numpy` - high-performance maths in Python
  * `scipy` - FFT
  * `PeakUtils` - peak detection
  * `pyserial` - serial interface

This may or may not work on Windows (mostly the serial port code), because Windows is... special.
