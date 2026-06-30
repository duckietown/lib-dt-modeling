# coding=utf-8

import math
from contextlib import nullcontext
from dataclasses import dataclass
from typing import Callable, ContextManager, Optional, Tuple

import numpy as np

from .dynamics_delay import ApplyDelay
from .generic_kinematics import GenericKinematicsSE2
from .platform_dynamics import PlatformDynamicsFactory
from .se2 import SE2value, se2value
from .types import TSE2value

__all__ = [
    "DynamicModelParameters",
    "DynamicModel",
    "PWMCommands",
    "get_DB18_nominal",
    "get_DB18_uncalibrated",
    "wheel_speed_from_pwm_commands",
    "set_profiler",
]

# Optional profiling hook. The library intentionally knows nothing about any
# concrete profiler implementation; consumers (e.g. the duckiematrix engine)
# inject one via ``set_profiler``. When none is registered, profiling is a no-op.
ProfilerFactory = Callable[[str], ContextManager]
_profiler_factory: Optional[ProfilerFactory] = None


def set_profiler(factory: Optional[ProfilerFactory]) -> None:
    """Register a profiling hook used while integrating the dynamics.

    Args:
        factory: A callable taking a profiling key and returning a context
            manager that times the wrapped block (e.g. ``T2Profiler.profile``).
            Pass ``None`` to disable profiling.
    """
    global _profiler_factory
    _profiler_factory = factory


def _profile(key: str) -> ContextManager:
    if _profiler_factory is None:
        return nullcontext()
    return _profiler_factory(key)


@dataclass
class PWMCommands:
    """
    PWM commands are floats between -1 and 1.
    """

    motor_left: float
    motor_right: float


class DynamicModelParameters(PlatformDynamicsFactory):
    wheel_radius_left: float
    wheel_radius_right: float
    wheel_distance: float
    encoder_resolution_rad: float

    def __init__(self, u1, u2, u3, w1, w2, w3, uar, ual, war, wal):
        # parameters for autonomous dynamics
        self.u1 = u1
        self.u2 = u2
        self.u3 = u3
        self.w1 = w1
        self.w2 = w2
        self.w3 = w3

        # parameters for forced dynamics
        self.u_alpha_r = uar
        self.u_alpha_l = ual
        self.w_alpha_r = war
        self.w_alpha_l = wal
        R = 0.067 / 2  # 6.7 cm diameter
        D = 0.1  # 10 cm
        self.wheel_radius_left = R
        self.wheel_radius_right = R
        self.wheel_distance = D
        ticks = 135
        res = (np.pi * 2) / ticks
        self.encoder_resolution_rad = res

    def initialize(self, c0, t0: float = 0, seed: int = None) -> "DynamicModel":
        return DynamicModel(self, c0, t0, 0.0, 0.0)


def get_DB18_nominal(delay: float) -> PlatformDynamicsFactory:
    # parameters for autonomous dynamics
    u1 = 5
    u2 = 0
    u3 = 0
    w1 = 4
    w2 = 0
    w3 = 0
    # parameters for forced dynamics
    uar = 1.5
    ual = 1.5
    war = 15  # modify this for trim
    wal = 15

    parameters = DynamicModelParameters(
        u1=u1, u2=u2, u3=u3, w1=w1, w2=w2, w3=w3, uar=uar, ual=ual, war=war, wal=wal
    )

    if delay > 0:
        delayed = ApplyDelay(parameters, delay, PWMCommands(0, 0))
        return delayed
    else:
        return parameters


def get_DB18_uncalibrated(delay: float, trim: float = 0) -> PlatformDynamicsFactory:
    # parameters for autonomous dynamics
    u1 = 5
    u2 = 0
    u3 = 0
    w1 = 4
    w2 = 0
    w3 = 0
    # parameters for forced dynamics
    uar = 1.5
    ual = 1.5
    war = 15 * (1.0 + trim)
    wal = 15 * (1.0 - trim)

    parameters = DynamicModelParameters(
        u1=u1, u2=u2, u3=u3, w1=w1, w2=w2, w3=w3, uar=uar, ual=ual, war=war, wal=wal
    )

    if delay > 0:
        delayed = ApplyDelay(parameters, delay, PWMCommands(0, 0))
        return delayed
    else:
        return parameters


class DynamicModel(GenericKinematicsSE2):
    """
    This represents a dynamical formulation of of a differential-drive vehicle.
    """

    parameters: DynamicModelParameters

    axis_left_rad: float
    axis_right_rad: float
    axis_left_obs_rad: float
    axis_right_obs_rad: float

    @staticmethod
    def _clip_command(value: float) -> float:
        if value < -1.0:
            return -1.0
        if value > 1.0:
            return 1.0
        return value

    @staticmethod
    def _model_acceleration(
        commands: PWMCommands,
        parameters: DynamicModelParameters,
        u: float,
        w: float,
    ) -> Tuple[float, float]:
        motor_right = DynamicModel._clip_command(commands.motor_right)
        motor_left = DynamicModel._clip_command(commands.motor_left)

        longitudinal_accel = (
            -parameters.u1 * u
            - parameters.u2 * w
            + parameters.u3 * w * w
            + parameters.u_alpha_r * motor_right
            + parameters.u_alpha_l * motor_left
        )
        angular_accel = (
            -parameters.w1 * w
            - parameters.w2 * u
            - parameters.w3 * u * w
            + parameters.w_alpha_r * motor_right
            - parameters.w_alpha_l * motor_left
        )

        return float(longitudinal_accel), float(angular_accel)

    @staticmethod
    def _velocity_from_linear_angular(
        longitudinal: float,
        angular: float,
    ) -> se2value:
        velocity = np.zeros((3, 3), dtype=np.float64)
        velocity[0, 1] = -angular
        velocity[1, 0] = angular
        velocity[0, 2] = longitudinal
        return velocity

    @staticmethod
    def _integrate_pose(
        q0: SE2value,
        dt: float,
        longitudinal: float,
        angular: float,
    ) -> SE2value:
        delta_angle = dt * angular
        delta_distance = dt * longitudinal
        if abs(delta_angle) < 1e-8:
            cos_delta = 1.0
            sin_delta = 0.0
            body_tx = delta_distance
            body_ty = 0.0
        else:
            sin_delta = math.sin(delta_angle)
            cos_delta = math.cos(delta_angle)
            scale = delta_distance / delta_angle
            body_tx = sin_delta * scale
            body_ty = (1.0 - cos_delta) * scale

        r00 = float(q0[0, 0])
        r01 = float(q0[0, 1])
        tx0 = float(q0[0, 2])
        r10 = float(q0[1, 0])
        r11 = float(q0[1, 1])
        ty0 = float(q0[1, 2])

        q1 = np.empty((3, 3), dtype=np.float64)
        q1[0, 0] = r00 * cos_delta + r01 * sin_delta
        q1[0, 1] = -r00 * sin_delta + r01 * cos_delta
        q1[0, 2] = tx0 + r00 * body_tx + r01 * body_ty
        q1[1, 0] = r10 * cos_delta + r11 * sin_delta
        q1[1, 1] = -r10 * sin_delta + r11 * cos_delta
        q1[1, 2] = ty0 + r10 * body_tx + r11 * body_ty
        q1[2, 0] = 0.0
        q1[2, 1] = 0.0
        q1[2, 2] = 1.0
        return q1

    def _update_axis_observations(self) -> None:
        resolution = self.parameters.encoder_resolution_rad
        left_ticks = round(self.axis_left_rad / resolution)
        right_ticks = round(self.axis_right_rad / resolution)
        self.axis_left_obs_rad = left_ticks * resolution
        self.axis_right_obs_rad = right_ticks * resolution

    @classmethod
    def _from_state_components(
        cls,
        parameters: DynamicModelParameters,
        q0: SE2value,
        v0: se2value,
        t0: float,
        axis_left_rad: float,
        axis_right_rad: float,
    ) -> "DynamicModel":
        state = cls.__new__(cls)
        state.parameters = parameters
        state.q0 = q0
        state.v0 = v0
        state.t0 = t0
        state.axis_left_rad = axis_left_rad
        state.axis_right_rad = axis_right_rad
        state._update_axis_observations()
        return state

    def __init__(
        self,
        parameters: DynamicModelParameters,
        c0: TSE2value,
        t0: float,
        axis_left_rad: float,
        axis_right_rad: float,
    ):
        self.parameters = parameters
        GenericKinematicsSE2.__init__(self, c0, t0)

        self.axis_left_rad = axis_left_rad
        self.axis_right_rad = axis_right_rad
        self._update_axis_observations()

    @staticmethod
    def model(
        commands: PWMCommands, parameters: DynamicModelParameters, u=None, w=None
    ):
        """Returns the second derivative of x"""
        longitudinal_accel, angular_accel = DynamicModel._model_acceleration(
            commands,
            parameters,
            float(u),
            float(w),
        )
        return np.array(
            [[longitudinal_accel], [angular_accel]],
            dtype=np.float64,
        )

    def integrate(self, dt: float, commands: PWMCommands) -> "DynamicModel":
        key_prefix = "[lib-dynamics]:dynamic-model/integrate"
        with _profile(f"{key_prefix}/extract-prev-state"):
            longit_prev = float(self.v0[0, 2])
            angular_prev = float(self.v0[1, 0])

        with _profile(f"{key_prefix}/predict-accel"):
            longitudinal_accel, angular_accel = self._model_acceleration(
                commands,
                self.parameters,
                u=longit_prev,
                w=angular_prev,
            )

        with _profile(f"{key_prefix}/integrate-velocity"):
            longitudinal = longit_prev + dt * longitudinal_accel
            angular = angular_prev + dt * angular_accel

        with _profile(f"{key_prefix}/compose-commands-se2"):
            next_v0 = self._velocity_from_linear_angular(longitudinal, angular)

        with _profile(f"{key_prefix}/body-pose-integrate"):
            next_q0 = self._integrate_pose(self.q0, dt, longitudinal, angular)

        with _profile(f"{key_prefix}/unpack-integrated-state"):
            t1 = self.t0 + dt

        with _profile(f"{key_prefix}/solve-wheel-velocities"):
            d = self.parameters.wheel_distance
            Rr = self.parameters.wheel_radius_right
            Rl = self.parameters.wheel_radius_left
            half_axle_angular = 0.5 * d * angular
            wR = (longitudinal + half_axle_angular) / Rr
            wL = (longitudinal - half_axle_angular) / Rl

        with _profile(f"{key_prefix}/accumulate-wheel-angles"):
            axis_left_rad = self.axis_left_rad + wL * dt
            axis_right_rad = self.axis_right_rad + wR * dt

        with _profile(f"{key_prefix}/construct-next-state"):
            return self._from_state_components(
                self.parameters,
                next_q0,
                next_v0,
                t1,
                axis_left_rad,
                axis_right_rad,
            )


# TODO: magic numbers in the prototype of this function
def wheel_speed_from_pwm_commands(
    pwm_l: float, pwm_r: float, k: float = 27.0
) -> Tuple[float, float]:
    """
    Returns:
    - omega_l, omega_r      wheels angular speed [rad/s]
    """
    # conversion from duty cycle to motor rotation rate
    omega_l = pwm_l * k
    omega_r = pwm_r * k

    return omega_l, omega_r
