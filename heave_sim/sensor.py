from numpy.random import randn
from math import sin, cos, pi


class SinusoidalMotion:

    """
    Generates sinusoidal position, velocity and acceleration data
    acc_mag is the magnitude of the acceleration in m/s^2
    f is the sinusoid's frequency in Hz
    """

    def __init__(self, acc_mag=1, f=1, phase=0, dt=1 / 100, noise_std=0.01, meas_noise_std=0.1):
        self.w = 2 * pi * f
        self.acc_mag = acc_mag
        self.p = phase
        self.k = 0
        self.dt = dt
        self.noise_std = noise_std
        self.meas_noise_std = meas_noise_std

        self.acc_offset = -9.4
        self.acc = self.acceleration(0)
        self.vel = self.velocity(0)
        self.pos = self.position(0)

    def update(self):
        self.acc = self.acceleration(self.k)
        self.vel = self.velocity(self.k)
        self.pos = self.position(self.k)

        self.k += 1
        return (self.pos, self.vel, self.acc)

    def sense(self):
        return self.acc + \
                self.acc_offset + \
                randn() * self.meas_noise_std

    def noise(self):
        return randn() * self.noise_std

    def acceleration(self, k):
        return self.acc_mag * sin(self.w * self.dt * k + self.p)

    def velocity(self, k):
        return -(self.acc_mag / self.w) * \
                cos(self.w * self.dt * k + self.p)

    def position(self, k):
        return -(self.acc_mag / self.w**2) * \
                sin(self.w * self.dt * k + self.p)
