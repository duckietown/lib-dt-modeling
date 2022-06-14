from typing import Dict, Tuple

from dt_modeling.kinematics.utils import trim_value


class PWM:
    """
    The `PWM` maps wheels' speed to duty cycle commands to send to the DC motors.
    It calculates the PWM commands that the DC motors should execute in order for the robot to
    perform a desired wheels' command.

    Args:
        gain (:obj:`float`):            scaling factor applied to the desired velocity
        trim (:obj:`float`):            trimming factor used to offset differences in the
                                        behaviour of the left and right motors, it is recommended
                                        to use a value that results in the robot moving in a
                                        straight line when equal forward commands are given
        k (:obj:`float`):               motor constant, assumed equal for both motors
        limit (:obj:`float`):           limits the final commands sent to the motors

    """

    def __init__(self,
                 gain: float = 1.0,
                 trim: float = 0.0,
                 k: float = 27.0,
                 limit: float = 1.0,
                 ):
        # store parameters
        self.gain: float = gain
        self.trim: float = trim
        self.k: float = k
        self.limit: float = limit

    def get_current_configuration(self) -> Dict[str, float]:
        return {
            "gain": self.gain,
            "trim": self.trim,
            "k": self.k,
            "limit": self.limit,
        }

    def get_wheels_gain(self) -> Tuple[float, float]:
        """
        Returns the wheels' gain adjusted by gain and trim.

        Returns:
            (:obj:`float`):         left wheel's gain
            (:obj:`float`):         right wheel's gain

        """
        # assuming same motor constants k for both motors
        k_r = k_l = self.k

        # adjusting k by gain and trim
        k_l_inv = (self.gain - self.trim) / k_l
        k_r_inv = (self.gain + self.trim) / k_r

        return k_l_inv, k_r_inv

    def get_wheels_duty_cycle(self, omega_l: float, omega_r: float) -> \
            Tuple[float, float]:
        """
        Maps the given wheel speeds to duty cycle commands that the robot can execute directly.

        Args:
            (:obj:`float`):         rotation speed of the left wheel in radians/second
            (:obj:`float`):         rotation speed of the right wheel in radians/second

        Returns:
            (:obj:`float`):         command to be sent to the left wheel
            (:obj:`float`):         command to be sent to the right wheel

        """
        # adjusting k by gain and trim
        k_l_inv, k_r_inv = self.get_wheels_gain()

        # conversion from motor rotation rate to duty cycle
        u_r = omega_r * k_r_inv
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = trim_value(u_r, -self.limit, self.limit)
        u_l_limited = trim_value(u_l, -self.limit, self.limit)

        return u_l_limited, u_r_limited
