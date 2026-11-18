import numpy as np

from config import PlacementConfig
from scene.placement.base import far_enough, register
from utils.cone import Cone


@register("uniform_annulus")
class UniformAnnulus:
    """Cones drawn uniformly over the area between min_radius and radius, kept min_gap apart."""

    MAX_TRIES_PER_CONE = 200

    def place(self, rng: np.random.Generator, cfg: PlacementConfig) -> list[Cone]:
        placed: list[np.ndarray] = []
        cones: list[Cone] = []
        tries = 0
        while len(cones) < cfg.num_cones:
            tries += 1
            if tries > self.MAX_TRIES_PER_CONE * cfg.num_cones:
                raise RuntimeError("cannot fit that many cones: raise radius or lower num_cones/min_gap")
            r = np.sqrt(rng.uniform(cfg.min_radius**2, cfg.radius**2))  # uniform over the annulus' area
            a = rng.uniform(0, 2 * np.pi)
            p = np.array([r * np.cos(a), r * np.sin(a)])
            if not far_enough(placed, p, cfg.min_gap):
                continue
            placed.append(p)
            cone_type = cfg.cone_types[rng.integers(len(cfg.cone_types))]
            cones.append(Cone(cone_type, float(p[0]), float(p[1]), 0.0, float(rng.uniform(0, 360))))
        return cones
