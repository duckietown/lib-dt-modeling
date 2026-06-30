"""Minimal numpy SE(2) helpers.

These replace the handful of operations the dynamics module used to take from
PyGeometry-z6 (the ``geometry`` package), so that lib-dt-modeling no longer
depends on it.

Representation (matching ``geometry``'s ``SE2value`` / ``se2value``):

* an SE(2) group element is a 3x3 homogeneous transform::

      [[ cos, -sin, x],
       [ sin,  cos, y],
       [   0,    0, 1]]

* an se(2) algebra element is a 3x3 matrix::

      [[ 0, -w, vx],
       [ w,  0, vy],
       [ 0,  0,  0]]
"""
import math
from typing import Tuple

import numpy as np

# Matrix representations are plain 3x3 numpy arrays.
SE2value = np.ndarray
se2value = np.ndarray


def check_SE2(q: SE2value) -> None:
    """Sanity-check that ``q`` looks like an SE(2) element (a 3x3 array)."""
    if not (isinstance(q, np.ndarray) and q.shape == (3, 3)):
        raise ValueError(f"Not a valid SE2 element (expected a 3x3 array): {q!r}")


def check_se2(v: se2value) -> None:
    """Sanity-check that ``v`` looks like an se(2) element (a 3x3 array)."""
    if not (isinstance(v, np.ndarray) and v.shape == (3, 3)):
        raise ValueError(f"Not a valid se2 element (expected a 3x3 array): {v!r}")


def multiply(a: SE2value, b: SE2value) -> SE2value:
    """SE(2) group composition (matrix product)."""
    return a @ b


def exp(x: se2value) -> SE2value:
    """Exponential map se(2) -> SE(2) (``geometry.SE2.group_from_algebra``).

    Closed form for the planar group: rotate by ``w`` and translate by the
    left-Jacobian-mapped linear part.
    """
    w = float(x[1, 0])
    vx = float(x[0, 2])
    vy = float(x[1, 2])
    cos_w = math.cos(w)
    sin_w = math.sin(w)
    if abs(w) < 1e-8:
        # limits as w -> 0: sin(w)/w -> 1, (1 - cos w)/w -> 0
        a = 1.0
        b = 0.0
    else:
        a = sin_w / w
        b = (1.0 - cos_w) / w
    tx = a * vx - b * vy
    ty = b * vx + a * vy
    return np.array(
        [[cos_w, -sin_w, tx],
         [sin_w, cos_w, ty],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def SE2_from_translation_angle(translation, angle: float) -> SE2value:
    """Build an SE(2) element from a translation and a rotation angle."""
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)
    return np.array(
        [[cos_a, -sin_a, translation[0]],
         [sin_a, cos_a, translation[1]],
         [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )


def se2_from_linear_angular(linear, angular: float) -> se2value:
    """Build an se(2) element from a linear velocity and an angular velocity."""
    return np.array(
        [[0.0, -angular, linear[0]],
         [angular, 0.0, linear[1]],
         [0.0, 0.0, 0.0]],
        dtype=np.float64,
    )


def linear_angular_from_se2(v: se2value) -> Tuple[np.ndarray, float]:
    """Extract (linear velocity, angular velocity) from an se(2) element."""
    linear = np.array([v[0, 2], v[1, 2]], dtype=np.float64)
    angular = float(v[1, 0])
    return linear, angular
