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
