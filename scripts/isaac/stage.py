"""The USD scene: a ground slab, the ATX sensor and a /World/Cones group that is re-filled for every frame.

Only pxr is needed here (no running Isaac app), so it is testable on its own. The two operations that have to wait for
the stage to finish loading take a wait_loaded callback instead of touching Isaac themselves.

Cones are children of /World/Cones, which carries the sensor's pose. Cone coordinates are therefore relative to the
sensor (x forward, y left), whatever the sensor's own position and yaw in the world.
"""

from pathlib import Path
from typing import Callable

from pxr import Gf, Usd, UsdGeom

from config import SensorConfig
from scene.cone_assets import cone_usd_path
from utils.cone import Cone

SENSOR_PRIM = "/World/ATX/sensor"  # the OmniLidar prim inside ATX_LiDAR.usd
CONES_ROOT = "/World/Cones"


def _place(prim, x: float, y: float, z: float, yaw_deg: float) -> None:
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddRotateZOp().Set(float(yaw_deg))


def build_scene_file(path: Path, sensor: SensorConfig, radius: float, atx_usd: Path) -> None:
    """Write the scene with an empty /World/Cones. Ground is a flat slab: 1 m thick, top face at z = 0."""
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.GetSizeAttr().Set(1.0)
    gx = UsdGeom.Xformable(ground)
    gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
    gx.AddScaleOp().Set(Gf.Vec3f(4 * radius, 4 * radius, 1))

    atx = stage.DefinePrim("/World/ATX", "Xform")
    atx.GetReferences().AddReference(str(atx_usd))
    _place(atx, sensor.x, sensor.y, sensor.height, sensor.yaw_deg)

    cones = stage.DefinePrim(CONES_ROOT, "Xform")
    _place(cones, sensor.x, sensor.y, 0.0, sensor.yaw_deg)  # on the ground, same heading as the sensor
    stage.GetRootLayer().Save()


def apply_layout(stage: Usd.Stage, cones: list[Cone], cones_dir: Path, wait_loaded: Callable[[], None]) -> None:
    """Replace whatever is under /World/Cones with these cones."""
    root = stage.GetPrimAtPath(CONES_ROOT)
    for child in list(root.GetChildren()):
        stage.RemovePrim(child.GetPath())
    for i, cone in enumerate(cones):
        prim = stage.DefinePrim(f"{CONES_ROOT}/cone_{i:04d}", "Xform")
        prim.GetReferences().AddReference(str(cone_usd_path(cones_dir, cone.type)))
        _place(prim, cone.x, cone.y, cone.z, cone.yaw_deg)
    wait_loaded()
