import time
from copy import deepcopy
from threading import Semaphore
from typing import Tuple, Dict, Optional

import numpy as np

from dt_modeling.lane_controller.types import ILaneController

Bounds = Tuple[float, float]


class PIDLaneController(ILaneController):
    """
    The Lane Controller can be used to compute control commands from pose estimations.

    The control commands are in terms of linear and angular velocity (v, omega).
    The inputs are errors in the relative pose of the Duckiebot in the current lane.

    This implementation is a simple PI(D) controller.

    Args:
        v_bar (:obj:`float`): Nominal velocity in m/s
        k_d (:obj:`float`): Proportional term for lateral deviation
        k_theta (:obj:`float`): Proportional term for heading deviation
        k_Id (:obj:`float`): integral term for lateral deviation
        k_Iphi (:obj:`float`): integral term for lateral deviation
        d_thres (:obj:`float`): Maximum value for lateral error
        phi_thres (:obj:`float`): Maximum value for heading error
        d_offset (:obj:`float`): Goal offset from center of the lane
        phi_offset (:obj:`float`): Goal offset from heading parallel to center of the lane
        d_resolution (:obj:`float`): Resolution of lateral position estimate
        phi_resolution (:obj:`float`): Resolution of heading estimate
        omega_ff (:obj:`float`): Feedforward part of controller
        integral_bounds (:obj:`dict`): Bounds for integral term
        stop_slowdown (:obj:`dict`): Start and end distances for slowdown at stops

    """

    DEFAULT_INTEGRAL_BOUNDS = {
        "d": (-0.3, 0.3),
        "phi": (-1.2, 1.2)
    }
    DEFAULT_STOP_SLOWDOWN = {
        "start": 0.6,
        "end": 0.15
    }

    def __init__(self,
                 v_bar: float = 0.2,
                 k_d: float = -6.0,
                 k_theta: float = -5,
                 k_Id: float = -0.3,
                 k_Iphi: float = 0.0,
                 d_thres: float = 0.25,
                 phi_thres: float = np.deg2rad(30),
                 d_offset: float = 0.0,
                 phi_offset: float = 0.0,
                 # TODO: these should match those from the lane filter
                 d_resolution: float = 0.02,
                 phi_resolution: float = np.deg2rad(5),
                 omega_ff: float = 0,
                 integral_bounds: dict = None,
                 stop_slowdown: dict = None
                 ):
        super(PIDLaneController, self).__init__()
        # store parameters
        self.v_bar: float = v_bar
        self.k_d: float = k_d
        self.k_theta: float = k_theta
        self.k_Id: float = k_Id
        self.k_Iphi: float = k_Iphi
        self.d_thres: float = d_thres
        self.phi_thres: float = phi_thres
        self.d_offset: float = d_offset
        self.phi_offset: float = phi_offset
        self.d_resolution: float = d_resolution
        self.phi_resolution: float = phi_resolution
        self.omega_ff: float = omega_ff
        self.integral_bounds: Dict[str, Bounds] = integral_bounds or \
                                                  deepcopy(self.DEFAULT_INTEGRAL_BOUNDS)
        self.stop_slowdown: Dict[str, float] = stop_slowdown or \
                                               deepcopy(self.DEFAULT_STOP_SLOWDOWN)
        # utility objects
        self._lock: Semaphore = Semaphore()
        # internal state
        self._d_I: float = 0.0
        self._phi_I: float = 0.0
        self._prev_d_err: float = 0.0
        self._prev_phi_err: float = 0.0
        self._prev_timestamp: Optional[float] = None
        self.__v: Optional[float] = None
        self.__w: Optional[float] = None

    def initialize(self):
        pass

    def update(self, d_hat: float, phi_hat: float,
               timestamp: Optional[float] = None, is_moving: bool = True,
               stop_distance: Optional[float] = None):
        """
        Main function, computes the control action given the current error signals.

        Given an estimate of the error, computes a control action.
        This is done via a basic PI(D) controller with anti-reset windup logic.

        Args:
            d_hat (:obj:`float`):           estimate in meters of the lateral deviation
            phi_hat (:obj:`float`):         estimate in radians of the heading deviation
            timestamp (:obj:`float`):       time when the observation of the error was performed
            is_moving (:obj:`bool`):        confirmation that the wheel commands have been executed
                                            (to avoid integration while the robot does not move)
            stop_distance (:obj:`float`):   distance to the next the stop, None if unknown
        """
        # compute delta_t across updates
        timestamp = timestamp if timestamp is not None else time.time()
        dt = None if self._prev_timestamp is None else timestamp - self._prev_timestamp

        # compute errors with respect to the given trajectory
        d_err = d_hat - self.d_offset
        phi_err = phi_hat - self.phi_offset

        # cap the errors if they are too large
        if np.abs(d_err) > self.d_thres:
            self._logger.warning(f"Lateral error too large: {d_err} > {self.d_thres}, capped.")
            d_err = np.sign(d_err) * self.d_thres
        if np.abs(phi_err) > self.phi_thres:
            self._logger.warning(f"Heading error too large: {phi_err} > {self.phi_thres}, capped.")
            phi_err = np.sign(phi_err) * self.phi_thres

        # integrate over time
        if dt is not None:
            self._integrate_errors(d_err, phi_err, dt)

        self._d_I = self._adjust_integral(
            d_err,
            self._d_I,
            self.integral_bounds["d"],
            self.d_resolution
        )

        self._phi_I = self._adjust_integral(
            phi_err,
            self._phi_I,
            self.integral_bounds["phi"],
            self.phi_resolution,
        )

        self._reset_if_needed(d_err, phi_err, is_moving)

        # Scale the parameters linear such that their real value is at 0.22m/s
        w = (
                self.k_d * d_err
                + self.k_theta * phi_err
                + self.k_Id * self._d_I
                + self.k_Iphi * self._phi_I
        )

        self._prev_d_err = d_err
        self._prev_phi_err = phi_err

        v = self._compute_velocity(stop_distance)

        # feedforward action
        w += self.omega_ff

        # store time of the last update
        self._prev_timestamp = timestamp

        # store commands
        with self._lock:
            self.__v = v
            self.__w = w

    def compute_commands(self) -> Tuple[float, float]:
        """
        Returns: A tuple with two values:
            v (:obj:`float`):       requested linear velocity in meters/second
            omega (:obj:`float`):   requested angular velocity in radians/second

        """
        with self._lock:
            return self.__v, self.__w

    def _compute_velocity(self, stop_distance: Optional[float]):
        """
        Linearly decrease velocity if approaching a stop place (e.g., an obstacle or stop line).

        If a stop place is detected, the velocity is linearly decreased to achieve a better
        stopping position, otherwise the nominal velocity is returned.

        Args:
            stop_distance (:obj:`float`):   distance to the next the stop, None if unknown
        """
        if stop_distance is None:
            return self.v_bar
        else:
            d1, d2 = self.stop_slowdown["start"], self.stop_slowdown["end"]
            # d1 -> v_bar, d2 -> v_bar/2
            c = (0.5 * (d1 - stop_distance) + (stop_distance - d2)) / (d1 - d2)
            v_new = self.v_bar * c
            v = np.max(
                [self.v_bar / 2.0,
                 np.min([self.v_bar, v_new])]
            )
            return v

    def _integrate_errors(self, d_err: float, phi_err: float, dt: float):
        """
        Integrates error signals in lateral and heading direction.

        Args:
            d_err (:obj:`float`):       error in meters in the lateral direction
            phi_err (:obj:`float`):     error in radians in the heading direction
            dt (:obj:`float`):          time delay in seconds
        """
        self._d_I += d_err * dt
        self._phi_I += phi_err * dt

    def _reset_if_needed(self, d_err: float, phi_err: float, is_moving: bool):
        """
        Resets the integral error if needed.

        Resets the integral errors in `d` and `phi` if either the error sign changes, or if the
        robot is completely stopped (i.e. intersections).

        Args:
            d_err (:obj:`float`):       error in meters in the lateral direction
            phi_err (:obj:`float`):     error in radians in the heading direction
            is_moving (:obj:`bool`):    confirmation that the wheel commands have been executed
                                        (to avoid integration while the robot does not move)
        """
        if np.sign(d_err) != np.sign(self._prev_d_err):
            self._d_I = 0
        if np.sign(phi_err) != np.sign(self._prev_phi_err):
            self._phi_I = 0
        if not is_moving:
            self._d_I = 0
            self._phi_I = 0

    @staticmethod
    def _adjust_integral(error: float, integral: float, bounds: Bounds,
                         resolution: float) -> float:
        """
        Bounds the integral error to avoid windup.

        Adjusts the integral error to remain in defined bounds, and cancels it if the error is
        smaller than the resolution of the error estimation.

        Args:
            error (:obj:`float`):                   current error value
            integral (:obj:`float`):                current integral value
            bounds (:obj:`tuple[float, float]`):    minimum and maximum values for the integral
            resolution (:obj:`float`):              resolution of the error estimate

        Returns:
            integral (:obj:`float`):                adjusted integral value
        """
        low_bound, up_bound = bounds
        if integral > up_bound:
            integral = up_bound
        elif integral < low_bound:
            integral = low_bound
        elif abs(error) < resolution:
            integral = 0
        return integral
