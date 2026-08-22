"""
Motor model: static (thrust/current vs throttle) + dynamic (first-order response).

Numbers from Carbon Aeronautics manual for the GEPRC 1105 5000kV motor with a Gemfan 3018 two-blade prop.
"""

import numpy as np
from scipy import signal

class Motor:
    def __init__(self, thrust_ratio = 160, current_ratio = 4.4, current_offset = 0.132, tau = 0.03):
        # static model coefficients
        self.thrust_ratio = thrust_ratio
        self.current_ratio = current_ratio
        self.current_offset = current_offset

        # dynamic model
        self.tau = tau
        self.tf = signal.TransferFunction([1], [self.tau, 1])

    def thrust(self, throttle):
        """Calculate thrust at given throttle"""
        throttle = np.clip(throttle, 0, 1)
        return self.thrust_ratio * throttle

    def current(self, throttle):
        """Calculate current at given throttle."""
        throttle = np.clip(throttle, 0, 1)
        return self.current_ratio * throttle + self.current_offset

    def efficiency(self, throttle):
        """Calculate efficiency at given throttle."""
        throttle = np.clip(throttle, 0, 1)
        return self.thrust(throttle) / self.current(throttle)

    def step_response(self, t = None):
        """Return time and throttle after a unit step throttle command"""
        if t is None:
            t = np.linspace(0,0.2,500)
        t_out, y_out = signal.step(self.tf, T=t)
        return t_out, y_out

    def simulate(self, t, throttle_signal):
        """Simulate motors actual throttle accounting for tau lag given a throttle signal"""
        t_out, y_out, _ = signal.lsim(self.tf, U = throttle_signal, T=t)
        return t_out, y_out

