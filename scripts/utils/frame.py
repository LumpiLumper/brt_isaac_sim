
import numpy as np
from dataclasses import dataclass, field

from utils.cone import Cone

DTYPE = np.dtype([(k, "<f4") for k in
                  ("x", "y", "z", "intensity", "distance", "azimuth", "elevation", "t")])

@dataclass
class Frame:
    data: np.ndarray                      # structured array, dtype=DTYPE
    cones: list[Cone] = field(default_factory=list)
    sensor_height: float = 0.0
    seed: int = 0
    idx: int = 0

    @property
    def xyz(self) -> np.ndarray:
        return np.column_stack([self.data["x"], self.data["y"], self.data["z"]])