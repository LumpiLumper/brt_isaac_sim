
import json
from pathlib import Path

from scene.cone_assets import CONE_NAMES, ConeSizes
from utils.cone import Cone
from utils.frame import Frame

FRAME_DESCRIPTION = ("sensor: x forward, y left, z up, origin at the sensor, unrotated "
                     "(box center z is relative to the sensor, yaw_deg is the cone's yaw)")

def make_label(cone: Cone, size, sensor_height: float):
    """Box label for one cone. cone.x/y are relative to the sensor, cone.z is the base height above the ground."""
    return {
        "type": CONE_NAMES[cone.type],
        "center": [float(cone.x), float(cone.y), float(cone.z) + size[2] / 2 - sensor_height],
        "size": [float(v) for v in size],
        "yaw_deg": float(cone.yaw_deg),
    }

def save_labels(path, frame: Frame, cone_sizes: ConeSizes):
    boxes = [make_label(c, cone_sizes[c.type], frame.sensor_height) for c in frame.cones]
    doc = {"seed": frame.seed, "frame_index": frame.idx, "frame": FRAME_DESCRIPTION,
           "sensor_height": frame.sensor_height, "boxes": boxes}
    Path(path).write_text(json.dumps(doc, indent=1))
    return boxes
