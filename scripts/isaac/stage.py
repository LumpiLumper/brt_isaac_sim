"""The USD scene: a ground slab, the ATX sensor and a /World/Cones group that is re-filled for every frame.

Only pxr is needed here (no running Isaac app), so it is testable on its own. The two operations that have to wait for
the stage to finish loading take a wait_loaded callback instead of touching Isaac themselves.

Cones are children of /World/Cones, which carries the sensor's pose. Cone coordinates are therefore relative to the
sensor (x forward, y left), whatever the sensor's own position and yaw in the world.
"""

from pathlib import Path
from typing import Callable

from pxr import Gf, Sdf, Usd, UsdGeom, UsdShade

from config import SensorConfig
from scene.cone_assets import cone_usd_path
from utils.cone import Cone

SENSOR_PRIM = "/World/ATX/sensor"  # the OmniLidar prim inside ATX_LiDAR.usd
CONES_ROOT = "/World/Cones"
GROUND_PRIM = "/World/Ground"
GROUND_MATERIAL = "/World/Looks/GroundNonVisual"
NONVISUAL_PREFIX = "omni:simready:nonvisual"  # carb setting /rtx/materialDb/nonVisualMaterialSemantics/prefix, Isaac 6.0.1


def _place(prim, x: float, y: float, z: float, yaw_deg: float) -> None:
    xf = UsdGeom.Xformable(prim)
    xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    xf.AddRotateZOp().Set(float(yaw_deg))


def apply_ground_material(stage: Usd.Stage, base: str, coating: str) -> None:
    """Give the ground a non-visual material: what RTX sensors "feel" of the surface, independent of its looks.

    Call it on the OPEN stage of the running app. Found by experiment (Isaac Sim 6.0.1): the same material authored
    into the scene file is ignored by the renderer, applied to the open stage it is picked up. Without any material
    the ground returns almost nothing at grazing angles, which cut it off beyond ~10 m at a sensor height of 0.4 m.

    This is what Isaac's NonVisualMaterial + apply_visual_materials() write: a Material prim with three string
    attributes (base, coating, attributes) that the ground prim is bound to. "none" for both leaves the ground bare.
    """
    if (base, coating) == ("none", "none"):
        return
    UsdGeom.Scope.Define(stage, "/World/Looks")
    material = UsdShade.Material.Define(stage, GROUND_MATERIAL)
    for name, value in (("base", base), ("coating", coating), ("attributes", "none")):
        material.GetPrim().CreateAttribute(f"{NONVISUAL_PREFIX}:{name}", Sdf.ValueTypeNames.String, custom=True).Set(value)
    UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(GROUND_PRIM)).Bind(material)


def build_scene_file(path: Path, sensor: SensorConfig, radius: float, atx_usd: Path) -> None:
    """Write the scene with an empty /World/Cones. Ground is a flat slab: 1 m thick, top face at z = 0.

    The ground's lidar material is not authored here, see apply_ground_material().
    """
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    ground = UsdGeom.Cube.Define(stage, GROUND_PRIM)
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
