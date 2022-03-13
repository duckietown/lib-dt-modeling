import logging
from abc import ABC, abstractmethod
from typing import Tuple


class ILaneController(ABC):

    def __init__(self):
        name = type(self).__name__
        self._logger: logging.Logger = logging.getLogger(name)

    @abstractmethod
    def initialize(self):
        pass

    @abstractmethod
    def update(self, d_hat: float, phi_hat: float, *args, **kwargs):
        pass

    @abstractmethod
    def compute_commands(self) -> Tuple[float, float]:
        pass
