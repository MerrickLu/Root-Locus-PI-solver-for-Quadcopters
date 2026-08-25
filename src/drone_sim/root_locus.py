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

def _open_loop_value(open_loop_tfs, s):
    """
    Evaluates product of the open loop transfer functions evaluated at s in series connection
    """
    val = 1 + 0j
    for tf in open_loop_tfs:
        val *= _evaluate_tf(tf, s)
    return val

def find_pi_gains(open_loop_tfs, overshoot_pct, settling_time):
    """
    Searches for P and I gains for controller C(s) = K*(s+b)/s = P + I/s such that
    the closed loop pole is located at the target closed loop pole (assumes unity
    feedback).

    Returns (P, I, s_star) where s_star is the target pole
    """
    s_star, zeta, wn = target_pole_from_spec(overshoot_pct, settling_time)

    # Angle condition states that the controller's zero location b must be such that
    # the total angle of C(s*)/K * GH(s*) equals 180 degrees (Alternatively, this is the
    # sum of the system poles - sum of the sytem zeros)
    def wrapped_angle_error(b):
        controller_shape_angle = np.angle((s_star + b) / s_star)
        total = controller_shape_angle + np.angle(_open_loop_value(open_loop_tfs, s_star))
        return np.angle(np.exp(1j * (total - np.pi))) # want this expression to be 0

    def find_valid_bracket(func, b_min=1e-4, b_max=1e4, steps=200):
        """
        finds a bracket [a, b] where func(a) and func(b) have opposite signs.
        (for use with brentq later)
        """
        b_vals = np.logspace(np.log10(b_min), np.log10(b_max), steps)
        f_vals = [func(b) for b in b_vals]

        for i in range(len(f_vals) - 1):
            if f_vals[i] * f_vals[i + 1] <= 0:
                return b_vals[i], b_vals[i + 1]

        raise ValueError("Could not find a valid root locus zero b in the search range. "
                         "The target target pole s* may be unachievable with a PI controller.")

    b_low, b_high = find_valid_bracket(wrapped_angle_error)
    b = brentq(wrapped_angle_error, b_low, b_high)

    # Magnitude condition states that the abs value of open loop tfs multiplied by K must
    # equal 1
    controller_shape_mag = abs((s_star + b) / s_star)
    K = 1 / (controller_shape_mag * abs(_open_loop_value(open_loop_tfs, s_star)))

    P = K
    I = K * b
    return P, I, s_star

def verify_dominant_pole(P, I, open_loop_tfs, s_star, min_separation_ratio = 3.0):
    """
    Closes loop with P, I gains found and checks whether target pole s_star is the dominant
    one. Does this by checking whether every other closed-loop pole decays faster by at least
    min_separation_ratio

    Returns a dict with closed-loop poles, whether s_star is dominant, and the overshoot and
    settling time
    """
    C_ctl = control.tf([P, I], [1, 0])
    open_loop_ctl = control.tf(open_loop_tfs[0].num, open_loop_tfs[0].den)
    for tf in open_loop_tfs[1:]:
        open_loop_ctl += control.tf(control.series(C_ctl, open_loop_ctl), 1)

    closed_loop = control.feedback(control.series(C_ctl, open_loop_ctl))
    poles = closed_loop.poles()

    target_real = s_star.real
    other_poles = [p for p in poles if not np.isclose(p, s_star, atol = 1e-2)
                   and not np.isclose(p, s_star.conjugate(), atol = 1e-2)]
    worst_ratio = min(abs(p.real) / abs(target_real) for p in other_poles) if other_poles else np.inf
    dominant = worst_ratio >= min_separation_ratio

    t = np.linspace(0, 10 / abs(target_real), 5000)
    t_out, y_out = control.step_response(closed_loop, T=t)
    final = y_out[-1]
    overshoot_actual = (np.max(y_out) - final) / final * 100
    band = 0.02 * final
    outside = np.where(np.abs(y_out - final) > band)[0]
    settle_actual = t_out[outside[-1]] if len(outside) else 0.0

    return {
        "poles": poles,
        "other_poles": other_poles,
        "worst_separation_ratio": worst_ratio,
        "dominant_pole_assumption_holds": dominant,
        "overshoot_actual_pct": overshoot_actual,
        "settling_time_actual": settle_actual,
    }

def step_response(open_loop_tfs, P, I, T):
    C_ctl = control.tf([P, I], [1, 0])
    open_loop_ctl = control.tf(open_loop_tfs[0].num, open_loop_tfs[0].den)
    for tf in open_loop_tfs[1:]:
        open_loop_ctl = control.series(open_loop_ctl, control.tf(tf.num, tf.den))
    closed_loop = control.feedback(control.series(C_ctl, open_loop_ctl), 1)
    t_out, y_out = control.step_response(closed_loop, T=T)
    return t_out, y_out




