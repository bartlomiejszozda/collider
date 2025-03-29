from abc import ABC, abstractmethod

import numpy as np

from collider.src.Helpers import DenormalizedBbox


class Tracker(ABC):
    @abstractmethod
    def track(self, frame: np.ndarray) -> DenormalizedBbox:
        pass

    @abstractmethod
    def name(self) -> str:
        pass
