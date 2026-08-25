"""
Root locus gain derivation.

Given a target closed-loop pole from desired overshoot/settling time
and an open-loop plant, solves for a PI controller C(s) = K*(s+b)/s
that places a closed-loop pole exactly there using the angle and
magnitude conditions of the root locus method.
"""

import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import brentq
import control

def target_pole_from_spec(overshoot_pct, settling_time):
    """
    Given a desired overshoot percent and 2% settling time in seconds, we can find a target
    closed loop pole location in the s-plane, using the standard second-order
    system relationships between damping ratio zeta, natural frequency wn.
    """
    zeta = -np.log(overshoot_pct / 100) / np.sqrt(np.pi ** 2 + np.log(overshoot_pct / 100) ** 2)
    wn = -np.log(0.02) / (zeta * settling_time)
    s_star = complex(-zeta * wn, wn * np.sqrt(1 - zeta ** 2))
    return s_star, zeta, wn


def _evaluate_tf(tf, s):
    """
    just a polynomial evaluation of a scipy TransferFunction
    """
    return np.polyval(tf.num, s) / np.polyval(tf.den, s)

