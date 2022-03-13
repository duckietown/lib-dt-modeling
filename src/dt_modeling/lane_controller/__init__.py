"""
    lane_controller
    ---------------

    The ``lane_controller`` library contains the implementations for control architectures for
    the motor commands. The basic and default one is :py:class:`LaneController`

    .. autoclass:: dt_modeling.lane_controller.LaneController

"""

from .types import ILaneController
from .lane_controller import PIDLaneController
