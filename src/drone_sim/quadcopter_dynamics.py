import numpy as np
from scipy import signal

class Quadcopter:
    """
    Rigid body quadcopter dynamics. Defines four transfer functions relating PID loop input commands to angular rates
    and vertical velocity.

    Numbers from Carbon Aeronautics manual, "Quadcopter dynamics simulation" (Project 22).

    Manual's derived transfer functions (Project 22), reproduced here as the default example build:
        RateRoll(s)         = 115/s * InputRoll(s)
        RatePitch(s)        = 115/s * InputPitch(s)
        RateYaw(s)          = 4.8/s * InputYaw(s)
    """

    def __init__(
        self,
        mass,                           # kg, total quadcopter mass
        Ix, Iy, Iz,                     # kg*m^2, moments of inertia about x,y,z axes
        num_motors=4,
        motor_thrust_ratio = 160,       # g of thrust per unit throttle [0:1]
        motor_current_ratio = 4.4,      # A per-unit throttle [0:1]
        motor_kv = 5000,                # rpm/V
        distance_motor_x = 0.08,        # m, motor distance from x-axis
        distance_motor_y = 0.08,        # m, motor distance from y-axis
        distance_esc_x = 0.04,          # m, esc distance from x-axis
    ):
        self.mass = mass
        self.Ix = Ix
        self.Iy = Iy
        self.Iz = Iz

        g = 9.81 # m/s^2

        # Roll and pitch: The difference in thrust between opposing motor pairs determines rotation rate
        # Acceleration [deg/s^2] = (180/pi) * num_motors * (thrust_ratio/1000) * (g/1000) * distance_motor / I
        roll_numerator = (
            (180 / np.pi)                   # rad/s to deg/s
            * num_motors                    # 2 motors reduce thrust, 2 motors increase thrust
            * (motor_thrust_ratio / 1000)   # thrust from motors for throttle [0:1000]
            * (1 / 1000)                    # divide by 1000 to convert g to kg
            * g                             # output newtons
            * distance_motor_x              # torque
        )

        pitch_numerator = (
                (180 / np.pi)  # rad/s to deg/s
                * num_motors  # 2 motors reduce thrust, 2 motors increase thrust
                * (motor_thrust_ratio / 1000)  # thrust from motors for throttle [0:1000]
                * (1 / 1000)  # divide by 1000 to convert g to kg
                * g  # output newtons
                * distance_motor_y  # torque
        )

        roll_gain = roll_numerator / Ix
        pitch_gain = pitch_numerator / Iy
        self.roll_tf = signal.TransferFunction([roll_gain], [1, 0])
        self.pitch_tf = signal.TransferFunction([pitch_gain], [1, 0])

        # Yaw: motor reaction torque determines yaw rate
        # KT (torque constant, N*m/A) derived from kV rating
        KT = 60 / (2 * np.pi * motor_kv)
        yaw_gain = (
            (180 / np.pi)                   # Convert from rad to deg
            * KT                            # Ratio of amps to torque
            * num_motors
            * (motor_current_ratio / 1000)  # current for throttle [0:1000]
            / Iz
        )
        self.yaw_tf = signal.TransferFunction([yaw_gain], [1, 0])

        # Vertical: net thrust determines vertical velocity
        # Acceleration [cm/s^2] = num_motors * (thrust_ratio/1000) * (g/1000) / mass * 100
        vertical_gain = (
            num_motors
            * (motor_thrust_ratio / 1000)   # Thrust for throttle [1:1000]
            * (g / 1000)                    # g to kg and then newtons
            / mass
            * 100                           # m to cm
        )
        self.vertical_tf = signal.TransferFunction([vertical_gain], [1, 0])

    def step_response(self):
        """
        Standard step test for each axis
        These will not settle down to a steady state. Useful for checking slope
        """
        results = {}
        for name, tf in [
            ("roll", self.roll_tf),
            ("pitch", self.pitch_tf),
            ("yaw", self.yaw_tf),
            ("vertical", self.vertical_tf)
        ]:
            t, y = signal.step(tf)
            results[name] = (t, y)
        return results


def _moment_of_inertia_x_or_y(motor_mass, motor_distance, esc_mass, esc_distance, num_motors=4):
    """
    Approximates moment of inertia with sum of motor and ESC point masses at their distance from x or y axis.
    """
    return num_motors * motor_mass * motor_distance**2 + \
        num_motors * esc_mass * esc_distance**2

def _moment_of_inertia_z(motor_mass, motor_dx, motor_dy, esc_mass, esc_dx, esc_dy, num_motors=4):
    """
    Approximates moment of inertia with sum of motor and ESC point masses at their distance from z axis.
    """
    return (
            num_motors * motor_mass * (motor_dx ** 2 + motor_dy ** 2)
            + num_motors * esc_mass * (esc_dx ** 2 + esc_dy ** 2)
    )

