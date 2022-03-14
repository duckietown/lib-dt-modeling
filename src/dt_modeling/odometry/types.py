import dataclasses

import numpy as np

import transformations as tr


@dataclasses.dataclass
class Pose2DEstimate:
    x: float
    y: float
    theta: float
    # time of the estimate
    time: float

    @property
    def q(self) -> np.ndarray:
        return tr.quaternion_from_euler(0, 0, self.theta)

    def copy(self) -> 'Pose2DEstimate':
        return Pose2DEstimate(**dataclasses.asdict(self))

    def __str__(self):
        return f"P(x={self.x}, y={self.y}, theta={self.theta})"
