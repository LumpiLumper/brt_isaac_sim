"""The cone USD files: which file belongs to which ConeType, and how big each cone really is."""

from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom

from utils.cone import ConeType

CONE_NAMES = {  # also the label "type" written to the json and the usd file name
    ConeType.SMALL_BLUE: "Small_Cone_Blue",
    ConeType.SMALL_YELLOW: "Small_Cone_Yellow",
    ConeType.SMALL_ORANGE: "Small_Cone_Orange",
    ConeType.BIG_ORANGE: "Big_Orange_Cone",
}

ConeSizes = dict[ConeType, tuple[float, float, float]]


def cone_usd_path(cones_dir: Path, cone_type: ConeType) -> Path:
    return Path(cones_dir) / f"{CONE_NAMES[cone_type]}.usd"


def measure_cone_sizes(cones_dir: Path) -> ConeSizes:
    """Bounding box size (x, y, z in m) of every cone type, measured from the usd geometry itself.

    The label box of a cone is centred half a height above its base, so each usd must have its origin at the centre of
    the base; anything else would put every label off the cone, and that is checked here.
    """
    sizes: ConeSizes = {}
    for cone_type in ConeType:
        path = cone_usd_path(cones_dir, cone_type)
        stage = Usd.Stage.Open(str(path))
        if stage is None:
            raise FileNotFoundError(path)
        root = stage.GetDefaultPrim() or stage.GetPseudoRoot()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
        box = cache.ComputeUntransformedBound(root).ComputeAlignedRange()
        lo, hi = np.array(box.GetMin()), np.array(box.GetMax())
        if abs(lo[2]) > 1e-3 or np.abs(lo[:2] + hi[:2]).max() > 1e-3:
            raise ValueError(f"{path.name}: origin is not the centre of the cone base (bbox {lo} .. {hi})")
        sizes[cone_type] = tuple(float(v) for v in hi - lo)
    return sizes
