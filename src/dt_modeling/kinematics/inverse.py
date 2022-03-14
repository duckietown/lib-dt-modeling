from typing import Dict, Tuple

from dt_modeling.kinematics.utils import trim_value


class InverseKinematics:
    """
    The `InverseKinematics` maps car speeds at the chassis level to wheel commands that the robot
    can execute.

    `InverseKinematics` utilises the car geometry as well as a number of parameters to calculate
    the wheel commands that the wheels should execute in order for the robot to perform the
    desired chassis commands.

    Args:
        wheel_baseline (:obj:`float`):  the distance between the two wheels of the robot
        wheel_radius (:obj:`float`):    radius of the wheel

        gain (:obj:`float`):            scaling factor applied to the desired velocity
        trim (:obj:`float`):            trimming factor used to offset differences in the
                                        behaviour of the left and right motors, it is recommended
                                        to use a value that results in the robot moving in a
                                        straight line when equal forward commands are given
        k (:obj:`float`):               motor constant, assumed equal for both motors
        limit (:obj:`float`):           limits the final commands sent to the motors
        v_max (:obj:`float`):           limits the input velocity
        omega_max (:obj:`float`):       limits the input steering angle

    """

    def __init__(self,
                 wheel_baseline: float,
                 wheel_radius: float,
                 gain: float = 1.0,
                 trim: float = 0.0,
                 k: float = 27.0,
                 limit: float = 1.0,
                 v_max: float = 1.0,
                 omega_max: float = 8.0,
                 ):
        # store parameters
        self.wheel_baseline: float = wheel_baseline
        self.wheel_radius: float = wheel_radius
        self.gain: float = gain
        self.trim: float = trim
        self.k: float = k
        self.limit: float = limit
        self.v_max: float = v_max
        self.omega_max: float = omega_max

    def get_current_configuration(self) -> Dict[str, float]:
        return {
            "wheel_baseline": self.wheel_baseline,
            "wheel_radius": self.wheel_radius,
            "gain": self.gain,
            "trim": self.trim,
            "k": self.k,
            "limit": self.limit,
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

    def get_wheels_duty_cycle(self, v: float, omega: float) -> Tuple[float, float]:
        """
        Maps the given car speeds at the chassis level to duty cycle wheel commands that the robot
        can execute directly.

        Args:
            v (:obj:`float`):       desired linear velocity of the chassis in meters/second
            omega (:obj:`float`):   desired angular velocity of the chassis in radians/second

        Returns:
            (:obj:`float`):         command to be sent to the left wheel
            (:obj:`float`):         command to be sent to the right wheel

        """
        # get wheels' rotation in radians/second
        omega_l, omega_r = self.get_wheels_speed(v, omega)

        # assuming same motor constants k for both motors
        k_r = k_l = self.k

        # adjusting k by gain and trim
        k_l_inv = (self.gain - self.trim) / k_l
        k_r_inv = (self.gain + self.trim) / k_r

        # conversion from motor rotation rate to duty cycle
        u_r = omega_r * k_r_inv
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = trim_value(u_r, -self.limit, self.limit)
        u_l_limited = trim_value(u_l, -self.limit, self.limit)

        return u_l_limited, u_r_limited
