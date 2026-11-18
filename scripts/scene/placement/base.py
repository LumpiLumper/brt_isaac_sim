from typing import Callable, Protocol

import numpy as np

from config import PlacementConfig
from utils.cone import Cone


class PlacementStrategy(Protocol):
    """One way of laying out cones around the sensor.

    The caller owns the rng (seeded per frame), so a strategy must draw all its randomness from it.
    """

    def place(self, rng: np.random.Generator, cfg: PlacementConfig) -> list[Cone]: ...


REGISTRY: dict[str, type[PlacementStrategy]] = {}


def register(name: str) -> Callable[[type], type]:
    def deco(cls):
        if name in REGISTRY:
            raise ValueError(f"placement strategy {name!r} is already registered")
        REGISTRY[name] = cls
        return cls
    return deco


def get_strategy(name: str) -> PlacementStrategy:
    if name not in REGISTRY:
        raise KeyError(f"unknown placement strategy {name!r}; available: {available()}")
    return REGISTRY[name]()


def available() -> list[str]:
    return sorted(REGISTRY)


def far_enough(placed_xy: list[np.ndarray], p: np.ndarray, min_gap: float) -> bool:
    """True if p is at least min_gap away from every already placed point."""
    if not placed_xy:
        return True
    return float(np.min(np.linalg.norm(np.array(placed_xy) - p, axis=1))) >= min_gap
