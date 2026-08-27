import numpy as np
from scipy import signal

class PID:
    """
    C(s) = P + I/s + D*s / (1 + s*Ts/2)
    Derived via the bilinear (Tustin) transform of the discrete PID update equation
    Iterm(k) = Iterm(k-1) + I*(Error(k)+Error(k-1))*Ts/2
    Dterm(k) = D*(Error(k)-Error(k-1))/Ts
    The P-term transform is trivial (P(z)=P(s)); I and D each pick up a 1+s*Ts/2 factor
    in the numerator/denominator respectively because Tustin maps z = (1+sTs/2)/(1-sTs/2).
    """

    def __init__(self, P=0, I=0, D=0, Ts = 0.004):
        """
        :param Ts: Time per iteration of the PID loop
        """
        self.P_tf = signal.TransferFunction([P], [1])
        self.I_tf = signal.TransferFunction([I], [1, 0])
        self.D_tf = signal.TransferFunction([D, 0], [Ts/2, 1])
        self.C_tf = signal.TransferFunction([(D + P * Ts / 2), (P + I * Ts / 2), I], [(Ts / 2), 1, 0])

    def step_response(self, t=None):
        if t is None:
            t = np.linspace(0, 1, 500)
        t_out, y_out = signal.step(self.C_tf, T=t)
        return t_out, y_out