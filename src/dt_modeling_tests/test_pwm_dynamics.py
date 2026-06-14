from typing import cast

import geometry as geo
import numpy as np
from dt_modeling.dynamics.generic_kinematics import GenericKinematicsSE2
from dt_modeling.dynamics.pwm_dynamics import (
    DynamicModel,
    PWMCommands,
    get_DB18_uncalibrated,
)


def _reference_integrate(
    state: DynamicModel,
    dt: float,
    commands: PWMCommands,
) -> DynamicModel:
    linear_prev, angular_prev = geo.linear_angular_from_se2(state.v0)
    longit_prev = linear_prev[0]

    x_dot_dot = state.model(commands, state.parameters, u=longit_prev, w=angular_prev)
    longitudinal = longit_prev + dt * x_dot_dot[0]
    angular = angular_prev + dt * x_dot_dot[1]
    commands_se2 = geo.se2_from_linear_angular([longitudinal[0], 0.0], angular[0])

    kinematics_state = GenericKinematicsSE2.integrate(state, dt, commands_se2)

    d = state.parameters.wheel_distance
    rr = state.parameters.wheel_radius_right
    rl = state.parameters.wheel_radius_left
    matrix = np.array([[rr / d, -rl / d], [rr / 2, rl / 2]])
    anglin = np.array((angular, longitudinal))
    wheel_velocities = np.linalg.inv(matrix) @ anglin
    w_r = float(wheel_velocities[0, 0])
    w_l = float(wheel_velocities[1, 0])

    return DynamicModel(
        state.parameters,
        (kinematics_state.q0, kinematics_state.v0),
        kinematics_state.t0,
        axis_left_rad=state.axis_left_rad + w_l * dt,
        axis_right_rad=state.axis_right_rad + w_r * dt,
    )


def _make_state(trim: float = 0.0) -> DynamicModel:
    factory = get_DB18_uncalibrated(delay=0.0, trim=trim)
    return cast(
        DynamicModel,
        factory.initialize(
            (
                geo.SE2_from_translation_angle([0.2, -0.1], 0.3),
                geo.se2_from_linear_angular([0.4, 0.0], -0.2),
            ),
            1.5,
        ),
    )


def _assert_same_state(actual: DynamicModel, expected: DynamicModel) -> None:
    np.testing.assert_allclose(actual.q0, expected.q0)
    np.testing.assert_allclose(actual.v0, expected.v0)
    np.testing.assert_allclose(actual.t0, expected.t0)
    np.testing.assert_allclose(actual.axis_left_rad, expected.axis_left_rad)
    np.testing.assert_allclose(actual.axis_right_rad, expected.axis_right_rad)
    np.testing.assert_allclose(actual.axis_left_obs_rad, expected.axis_left_obs_rad)
    np.testing.assert_allclose(actual.axis_right_obs_rad, expected.axis_right_obs_rad)


def test_integrate_matches_reference_for_turning_motion() -> None:
    state = _make_state(trim=0.03)
    commands = PWMCommands(0.35, 0.55)
    dt = 0.025

    actual = state.integrate(dt, commands)
    expected = _reference_integrate(state, dt, commands)

    _assert_same_state(actual, expected)


def test_integrate_matches_reference_for_straight_motion() -> None:
    state = _make_state(trim=0.0)
    state.v0 = geo.se2_from_linear_angular([0.0, 0.0], 0.0)
    commands = PWMCommands(0.45, 0.45)
    dt = 0.025

    actual = state.integrate(dt, commands)
    expected = _reference_integrate(state, dt, commands)

    _assert_same_state(actual, expected)
