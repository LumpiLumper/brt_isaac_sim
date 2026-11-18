from scene.placement.base import PlacementStrategy, available, get_strategy, register
from scene.placement import uniform_annulus  # noqa: F401  (registers itself; import new strategies here)

__all__ = ["PlacementStrategy", "available", "get_strategy", "register"]
