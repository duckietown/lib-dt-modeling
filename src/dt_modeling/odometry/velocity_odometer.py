import time
from threading import Semaphore
from typing import Optional

import numpy as np

from dt_modeling.odometry.types import Pose2DEstimate


class VelocityToPose:
    """
    `VelocityToPose` integrates the velocity of the robot over time in order to continuously
    obtain a pose estimate that is relative to the pose at which the integration started.

    """
    def __init__(self):
        # utility objects
        self._lock: Semaphore = Semaphore()
        # internal state
        self._last_theta_dot: float = 0
        self._last_v: float = 0
        self._last_timestamp: Optional[float] = None
        self._has_estimate: bool = False
        # current estimate
        self._pose: Pose2DEstimate = Pose2DEstimate(0, 0, 0, 0)

    def update(self, v: float, omega: float, timestamp: float = None):
        """
        Performs the calclulation from velocity to pose and publishes a messsage with the result.

        Args:
            v (:obj:`float`):           desired linear velocity of the chassis in meters/second
            omega (:obj:`float`):       desired angular velocity of the chassis in radians/second
            timestamp (:obj:`float`):   timestamp in seconds when the reading was performed, if
                                        None is given, the current time is used

        """
        timestamp = timestamp if timestamp is not None else time.time()

        with self._lock:
            if self._last_timestamp is not None:
                # compute delta_t
                dt = timestamp - self._last_timestamp

                # integrate the relative movement between the last pose and the current
                theta_delta = self._last_theta_dot * dt

                # to ensure no division by zero for radius calculation:
                if np.abs(self._last_theta_dot) < 0.000001:
                    # straight line
                    x_delta = self._last_v * dt
                    y_delta = 0
                else:
                    # arc of circle
                    radius = self._last_v / self._last_theta_dot
                    x_delta = radius * np.sin(theta_delta)
                    y_delta = radius * (1.0 - np.cos(theta_delta))

                # add to the previous to get absolute pose relative to the starting position
                theta_res = self._pose.theta + theta_delta
                x_res = self._pose.x + \
                        x_delta * np.cos(self._pose.theta) - \
                        y_delta * np.sin(self._pose.theta)
                y_res = self._pose.y + \
                        y_delta * np.cos(self._pose.theta) + \
                        x_delta * np.sin(self._pose.theta)

                print(dt, x_delta, x_res)

                # update the stored pose
                self._pose.theta = theta_res
                self._pose.x = x_res
                self._pose.y = y_res
                self._pose.time = timestamp
                self._has_estimate = True

            # update internal state
            self._last_timestamp = timestamp
            self._last_theta_dot = omega
            self._last_v = v

    def get_estimate(self) -> Optional[Pose2DEstimate]:
        with self._lock:
            if not self._has_estimate:
                return None
            return self._pose.copy()
