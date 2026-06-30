import dataclasses

import numpy as np


@dataclasses.dataclass
class Pose2DEstimate:
    x: float
    y: float
    theta: float
    # time of the estimate
    time: float

    @property
    def q(self) -> np.ndarray:
        # Yaw-only rotation as a [w, x, y, z] quaternion. Matches the previous
        # transformations.quaternion_from_euler(0, 0, theta): a rotation of
        # theta about z is cos(theta/2) + sin(theta/2) k.
        half = self.theta / 2.0
        return np.array([np.cos(half), 0.0, 0.0, np.sin(half)])

    def copy(self) -> 'Pose2DEstimate':
        return Pose2DEstimate(**dataclasses.asdict(self))

    def __str__(self):
        return f"P(x={self.x}, y={self.y}, theta={self.theta})"
