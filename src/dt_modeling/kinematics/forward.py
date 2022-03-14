from typing import Dict, Tuple


class ForwardKinematics:
    """
    The `ForwardKinematics` maps wheels' speed to car speeds at the chassis level.

    `ForwardKinematics` utilises the car geometry as well as a number of parameters to calculate
    the velocities that the chassis will experience given the wheels' speed.
    Note that this is a simple open-loop velocity estimation.

    Args:
        wheel_baseline (:obj:`float`):  the distance between the two wheels of the robot
        wheel_radius (:obj:`float`):    radius of the wheel

    """

    def __init__(self,
                 wheel_baseline: float,
                 wheel_radius: float,
                 ):
        # store parameters
        self.wheel_baseline: float = wheel_baseline
        self.wheel_radius: float = wheel_radius

    def get_current_configuration(self) -> Dict[str, float]:
        return {
            "wheel_baseline": self.wheel_baseline,
            "wheel_radius": self.wheel_radius,
        }

    def get_chassis_speed(self, omega_left: float, omega_right: float) -> Tuple[float, float]:
        """
        Maps given wheels' speed to car speeds at the chassis level.

        Args:
            omega_left (:obj:`float`):      angular speed of the left wheel in radians/second
            omega_right (:obj:`float`):     angular speed of the right wheel in radians/second

        Returns:
            (:obj:`float`):                 linear velocity of the chassis in meters/second
            (:obj:`float`):                 angular velocity of the chassis in radians/second

        """
        # compute linear and angular velocity of the chassis
        v = (self.wheel_radius * (omega_right + omega_left)) / 2.0
        w = (self.wheel_radius * (omega_right - omega_left)) / self.wheel_baseline

        return v, w
