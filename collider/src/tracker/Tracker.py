from abc import ABC, abstractmethod
from collider.src.Helpers import DenormalizedBbox

import numpy as np

class Tracker(ABC):
    @abstractmethod
    def track(self, frame: np.ndarray) -> (bool, DenormalizedBbox):
        pass