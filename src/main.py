from motor import Motor
from sensor import OrientationSensor
import numpy as np
import matplotlib.pyplot as plt
import quadcopter_dynamics as qd
import root_locus as rl

def print_axis_summary(name: str, result: dict, P, I, s_star) -> None:
    """
    print a summary for an axis dynamic verification
    """
    divider = "=" * 60
    print(f"\n{divider}")
    print(f" {name.upper()} DYNAMICS")
    print(divider)
    print(f"P: {P:.3f} | I: {I:.3f} | s*: {s_star:.3f}")
    print(f"Closed-loop poles : {result['poles']}")
    print(f"Dominant Pole Hold: {result['dominant_pole_assumption_holds']} "
          f"(Ratio: {result['worst_separation_ratio']:.2f}x | Min: 3.00x)")
    print("-" * 60)
    print(f"Target Specs      : 10.0% overshoot | 0.500s settling")
    print(f"Response   : {result['overshoot_actual_pct']:.1f}% overshoot | "
          f"{result['settling_time_actual']:.3f}s settling")

# system Setup
motor = Motor(160, 4.4, 0.132, 0.03)
sensor = OrientationSensor(10)
Ix = qd.moment_of_inertia_x_or_y(0.008, 0.08, 0.007, 0.04)
Iy = qd.moment_of_inertia_x_or_y(0.008, 0.08, 0.007, 0.04)
Iz = qd.moment_of_inertia_z(0.008, 0.08, 0.08, 0.007, 0.04, 0.04)
dynamics = qd.Quadcopter(mass=0.250, Ix = Ix, Iy = Iy, Iz= Iz)

# axis configuration
axes_tf = {
    "Roll": dynamics.roll_tf,
    "Pitch": dynamics.pitch_tf,
    "Yaw": dynamics.yaw_tf,
}

# processing and plotting Setup
t = np.linspace(0, 1.5, 5000)
plt.figure(figsize=(10, 5))

for name, axis_tf in axes_tf.items():
    # find PI gains for 10% overshoot, 0.5s settling.
    P, I, s_star = rl.find_pi_gains(
        [motor.tf, axis_tf, sensor.tf], overshoot_pct=10, settling_time=0.5
    )

    # verify dominant pole assumptions
    system_tfs = [motor.tf, axis_tf, sensor.tf]
    result = rl.verify_dominant_pole(P, I, system_tfs, s_star)
    print_axis_summary(name, result, P, I, s_star)

    # Compute step response and plot
    t_out, y_out = rl.step_response(system_tfs, P, I, t)
    plt.plot(t_out, y_out, label=f"{name} Axis", linewidth=1.8)

# format and show plot
plt.axhline(1.0, color='black', linestyle='--', alpha=0.5, label='Target (1.0)')
plt.title("Quadcopter Closed-Loop Step Response (Roll, Pitch, Yaw)", fontsize=12, fontweight='bold')
plt.xlabel("Time (s)")
plt.ylabel("Normalized Output")
plt.grid(True, linestyle=':', alpha=0.7)
plt.legend(loc='lower right')
plt.tight_layout()
plt.show()