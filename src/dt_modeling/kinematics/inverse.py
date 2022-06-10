from typing import Dict, Tuple

from dt_modeling.kinematics.utils import trim_value


class InverseKinematics:
    """
    The `InverseKinematics` maps car speeds at the chassis level to wheel commands that the robot
    should execute.
    It utilises the car geometry to calculate the wheels' rotation needed for the robot to perform
    the desired chassis commands.

    Args:
        wheel_baseline (:obj:`float`):  the distance between the two wheels of the robot
        wheel_radius (:obj:`float`):    radius of the wheel
        v_max (:obj:`float`):           limits the input velocity
        omega_max (:obj:`float`):       limits the input steering angle

    """

    def __init__(self,
                 wheel_baseline: float,
                 wheel_radius: float,
                 v_max: float = 1.0,
                 omega_max: float = 8.0,
                 ):
        # store parameters
        self.wheel_baseline: float = wheel_baseline
        self.wheel_radius: float = wheel_radius
        self.v_max: float = v_max
        self.omega_max: float = omega_max

    def get_current_configuration(self) -> Dict[str, float]:
        return {
            "wheel_baseline": self.wheel_baseline,
            "wheel_radius": self.wheel_radius,
            "v_max": self.v_max,
            "omega_max": self.omega_max,
        }

    def get_wheels_speed(self, v: float, omega: float) -> Tuple[float, float]:
        """
        Maps the given car speeds at the chassis level to wheel rotation rates.

        Args:
            v (:obj:`float`):       desired linear velocity of the chassis in meters/second
            omega (:obj:`float`):   desired angular velocity of the chassis in radians/second

        Returns:
            (:obj:`float`):         rotation speed of the left wheel in radians/second
            (:obj:`float`):         rotation speed of the right wheel in radians/second

        """
        # trim the desired commands such that they are within the limits:
        v = trim_value(v, low=-self.v_max, high=self.v_max)
        omega = trim_value(omega, low=-self.omega_max, high=self.omega_max)

        # compute the wheels' rotation given the robot's geometry
        omega_l = (v - 0.5 * omega * self.wheel_baseline) / self.wheel_radius
        omega_r = (v + 0.5 * omega * self.wheel_baseline) / self.wheel_radius

        return omega_l, omega_r
