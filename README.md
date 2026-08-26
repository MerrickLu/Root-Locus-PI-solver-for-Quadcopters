# Root-Locus PI Controller Solver for Quadcopters
A Python tool designed to analyze quadcopter pitch/roll/yaw dynamics and automatically solve for Proportional-Integral (PI) gain parameters 
given a overshoot percentage and settling time using Root Locus analysis

> Still in progress, transfer function modelling and pole placement logic are complete. Still need a derivative term, and UI and gain visualization.

## Overview
This is an automated PI gain selection tool to achieve desired damping ratio and settling time for quadcopter rate loops. I built this to learn Python and the 
mathematics of modelling control systems. The idea is that this is step one before eventually writing real flight controller firmware in C.  

## Code Structure
Numbers are based on motor and airframe specs from Carbon Aeronautics quadcopter build manual 
(Projects 21-24 cover the model, this is my own implementation of it) 
- **`motor.py`**: motor model. Static part (thrust/current as linear functions of throttle)
  plus a first-order lag `1/(τs+1)`.
- **`sensor.py`**: models the gyro's onboard low pass filter as another
  first-order lag based on its cutoff frequency.
- **`quadcopter_dynamics.py`**: rigid-body plant. Roll/pitch/yaw
  modelled as pure integrator `gain/s`, found using motor thrust,
  moment arms, and moment of inertia (`Ix`/`Iy`/`Iz`, approximated by assuming
  point masses for the motors and ESCs).
- **`pid.py`**: PID controller written as a continuous transfer
  function via the bilinear (Tustin) transform to match what would
  run on a microcontroller at a given loop rate `Ts`. (Not used
  in `main.py` yet since `root_locus.py` builds its own PI controller.)
- **`root_locus.py`**: Given target overshoot (%) and a 
  settling time (s), computes a target closed-loop pole, then uses the
  root-locus angle condition (solved numerically with `brentq` on a dynamic
  bracket [a, b]) and magnitude condition to find `P` and `I` for a
  `C(s) = K(s+b)/s` controller. Also has a function that closes the loop with
  the found gains and checks whether the target pole is dominant.
- **`main.py`**: prints a summary per axis (roll/pitch/yaw), plots the closed-loop step responses.

## root_locus.py
Overshoot % and settling time are converted into a target closed-loop pole
using the standard relations:

    ζ  = -ln(OS/100) / sqrt(π² + ln²(OS/100))
    ωn = -ln(0.02) / (ζ · t_settling)          # for a 2% settling time
    s* = -ζωn + jωn√(1-ζ²)

`find_pi_gains()` then solves for a controller C(s) = K(s+b)/s that
places a closed-loop pole at s*, using the two root-locus
conditions.

## Example Output
`main.py` plots the closed-loop step response for all three axes:

<img width="1000" height="500" alt="Figure_1" src="https://github.com/user-attachments/assets/907705eb-81d8-49ee-8444-4ae1bb165b4c" />

*Roll, pitch, and yaw traces overlap because there's no cross
coupling between axes or asymmetry yet, so the SISO solutions overlap.*

<details>
<summary>Console output (Roll axis; Pitch/Yaw are identical in this run)</summary>

```text
============================================================
 ROLL DYNAMICS
============================================================
Closed-loop poles : [-71.26603877 +0.j          -7.82404601+10.67494337j
  -7.82404601-10.67494337j  -9.25105562 +0.j        ]
Dominant Pole Hold: False (Ratio: 1.18x | Min: 3.00x)
------------------------------------------------------------
Target Specs      : 10.0% overshoot | 0.500s settling
Actual Response   : 42.2% overshoot | 0.477s settling
============================================================
```
</details>

`Dominant Pole Hold: False` here means the target pole pair's real part
is only ~1.18x closer to the imaginary axis than the nearest other pole 
(Ideally should be more than 3x). This means the dominant-pole
approximation breaks down (see Limitations below) and we can see the
actual overshoot (42.2%) overshoots the 10% target by a wide margin because
the other closed loop poles aren't negligible. 

- **Angle condition** states the sum of angles from open-loop poles to s*, minus
  angles from open-loop zeros to s*, must equal 180°. 
- **Magnitude condition** states the absolute value of open-loop gain at s* must equal 1.
  Solves for the overall gain `K`, which then gives `P = K`, `I = K·b`.

## Limitations of the approach
- **Dominant-pole approximation.**: this approach is based on
  reducing a higher-order system down to a target 2nd-order pole pair by
  placing zeroes. The approximation is assuming that the systems other
  closed loop poles are far away enough to be negligible.
- **Single-axis, SISO, unity feedback.** Roll/pitch/yaw are solved
  independently. Quadcopters have some cross coupling between axes
  (gyroscopic effects, coupled inertia) that this ignores entirely.
- **Linearized, fixed operating point.** Motor thrust/current curves are
  treated as linear around a nominal throttle. 
