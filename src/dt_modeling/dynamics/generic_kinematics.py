# coding=utf-8

from . import se2

# from duckietown_serialization_ds1 import Serializable
from .platform_dynamics import PlatformDynamics, PlatformDynamicsFactory
from .types import TSE2value, se2v


__all__ = ["GenericKinematicsSE2"]


class GenericKinematicsSE2(PlatformDynamicsFactory, PlatformDynamics): #, Serializable):
    """
    Any dynamics on SE(2)

    Commands = velocities in se(2)
    """

    @classmethod
    def initialize(cls, c0: TSE2value, t0: float = 0, seed=None):
        return GenericKinematicsSE2(c0, t0)

    def __init__(self, c0: TSE2value, t0: float):
        # start at q0, v0
        q0, v0 = c0
        se2.check_SE2(q0)
        se2.check_se2(v0)
        self.t0 = t0
        self.v0 = v0
        self.q0 = q0

    def integrate(self, dt: float, commands: se2v) -> "GenericKinematicsSE2":
        """ commands = velocity in body frame """
        # convert to float
        dt = float(dt)
        # the commands must belong to se(2)
        se2.check_se2(commands)
        v = commands
        # suppose we hold v for dt, which pose are we going to?
        diff = se2.exp(dt * v)  # exponential map
        # compute the absolute new pose; applying diff from q0
        q1 = se2.multiply(self.q0, diff)
        # the new configuration
        c1 = q1, v
        # the new time
        t1 = self.t0 + dt
        # return the new state
        return GenericKinematicsSE2(c1, t1)

    def TSE2_from_state(self) -> TSE2value:
        return self.q0, self.v0
