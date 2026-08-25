from motor import Motor
from sensor import OrientationSensor
import numpy as np
import matplotlib.pyplot as plt
import quadcopter_dynamics as qd
import root_locus as rl

motor = Motor(160, 4.4, 0.132, 0.03)
sensor = OrientationSensor(10)
Ix = qd.moment_of_inertia_x_or_y(0.008, 0.08, 0.007, 0.04)
Iy = qd.moment_of_inertia_x_or_y(0.008, 0.08, 0.007, 0.04)
Iz = qd.moment_of_inertia_z(0.008, 0.08, 0.08, 0.007, 0.04, 0.04)
dynamics = qd.Quadcopter(mass=0.250, Ix = Ix, Iy = Iy, Iz= Iz)

# Find PI gains for 10% overshoot, 0.5s settling.
P, I, s_star = rl.find_pi_gains(
    [motor.tf, dynamics.yaw_tf, sensor.tf], overshoot_pct=10, settling_time=0.5
)
print(f"Derived P={P:.3f}, I={I:.3f}")

# roll dynamics
result = rl.verify_dominant_pole(P, I, [motor.tf, dynamics.roll_tf, sensor.tf], s_star)
print("Roll Dynamics:")
print(f"Closed-loop poles: {result['poles']}")
print(f"Dominant pole assumption holds: {result['dominant_pole_assumption_holds']} "
      f"(worst separation ratio: {result['worst_separation_ratio']:.2f}x, want >= 3x)")
print(f"Requested: 10% overshoot, 0.5s settling")
print(f"Actual:    {result['overshoot_actual_pct']:.1f}% overshoot, "
      f"{result['settling_time_actual']:.3f}s settling")

# pitch dynamics
result = rl.verify_dominant_pole(P, I, [motor.tf, dynamics.pitch_tf, sensor.tf], s_star)
print("Pitch Dynamics:")
print(f"Closed-loop poles: {result['poles']}")
print(f"Dominant pole assumption holds: {result['dominant_pole_assumption_holds']} "
      f"(worst separation ratio: {result['worst_separation_ratio']:.2f}x, want >= 3x)")
print(f"Requested: 10% overshoot, 0.5s settling")
print(f"Actual:    {result['overshoot_actual_pct']:.1f}% overshoot, "
      f"{result['settling_time_actual']:.3f}s settling")

# yaw dynamics
result = rl.verify_dominant_pole(P, I, [motor.tf, dynamics.yaw_tf, sensor.tf], s_star)
print("Yaw Dynamics:")
print(f"Closed-loop poles: {result['poles']}")
print(f"Dominant pole assumption holds: {result['dominant_pole_assumption_holds']} "
      f"(worst separation ratio: {result['worst_separation_ratio']:.2f}x, want >= 3x)")
print(f"Requested: 10% overshoot, 0.5s settling")
print(f"Actual:    {result['overshoot_actual_pct']:.1f}% overshoot, "
      f"{result['settling_time_actual']:.3f}s settling")

