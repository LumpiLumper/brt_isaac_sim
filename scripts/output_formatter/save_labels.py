
import json
from pathlib import Path

from utils.cone import ConeType

CONE_LABELS = {
    ConeType.SMALL_BLUE: "Small_Cone_Blue",
    ConeType.SMALL_YELLOW: "Small_Cone_Yellow",
    ConeType.SMALL_ORANGE: "Small_Cone_Orange",
    ConeType.BIG_ORANGE: "Big_Orange_Cone",
}

CONE_SIZES = {
    "Small_Cone_Blue": (0.24, 0.24, 0.325),
    "Small_Cone_Yellow": (0.24, 0.24, 0.325),
    "Small_Cone_Orange": (0.24, 0.24, 0.325),
    "Big_Orange_Cone": (0.38, 0.38, 0.505),
}

def make_label(cone_type, xyz):
    """Box label for one cone given its type and the xyz of its base."""
    if cone_type not in CONE_SIZES:
        raise KeyError(f"unknown cone type {cone_type!r}; known: {sorted(CONE_SIZES)}")
    x, y, z = (float(v) for v in xyz)
    size = CONE_SIZES[cone_type]
    return {"type": cone_type, "center": [x, y, z + size[2] / 2], "size": list(size), "yaw_deg": 0.0}

def save_labels(path, cones):
    boxes = [make_label(CONE_LABELS[c.type], (c.x, c.y, c.z)) for c in cones]
    doc = {"frame": "sensor: x forward, y left, z up, origin at the sensor", "boxes": boxes}
    Path(path).write_text(json.dumps(doc, indent=1))
    return boxes