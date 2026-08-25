"""
Dynamic sensor model of MPU-6050 orientation sensor using low pass filter
"""
import numpy as np
from scipy import signal

class OrientationSensor:
    def __init__(self, cutoff_freq = 10):
        self.cutoff_freq = cutoff_freq
        self.tau = 1/(2*np.pi*cutoff_freq)
        self.tf = signal.TransferFunction([1], [self.tau, 1])

    def step_response(self, t=None):
        if t is None:
            t = np.linspace(0, 0.2, 500)
        t_out, y_out = signal.step(self.tf, T=t)
        return t_out, y_out
