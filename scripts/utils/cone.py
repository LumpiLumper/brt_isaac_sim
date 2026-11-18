
from dataclasses import dataclass
from enum import Enum, auto, unique


@unique
class ConeType(Enum):
    BIG_ORANGE = auto()
    SMALL_BLUE = auto()
    SMALL_ORANGE = auto()
    SMALL_YELLOW = auto()


@dataclass(frozen=True)
class Cone:
    type: ConeType
    x: float
    y: float
    z: float