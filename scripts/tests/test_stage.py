import numpy as np
import pytest
from pxr import Gf, Usd, UsdGeom, UsdShade

from config import SensorConfig, resolve_path
from isaac.stage import CONES_ROOT, apply_ground_material, apply_layout, build_scene_file
from scene.cone_assets import CONE_NAMES, cone_usd_path, measure_cone_sizes
from utils.cone import Cone, ConeType

CONES_DIR = resolve_path(__import__("pathlib").Path("assets/cones"))
ATX_USD = resolve_path(SensorConfig().usd_path)


def world_pos(prim):
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return np.array(m.ExtractTranslation())


def world_yaw_deg(prim):
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    return float(np.degrees(np.arctan2(m[0][1], m[0][0])))


@pytest.fixture
def scene(tmp_path):
    def make(sensor=SensorConfig(height=0.9), radius=20.0):
        path = tmp_path / "scene.usd"
        build_scene_file(path, sensor, radius, ATX_USD)
        return Usd.Stage.Open(str(path))
    return make


# --- cone assets -----------------------------------------------------------

def test_every_cone_type_has_a_usd_file():
    for t in ConeType:
        assert cone_usd_path(CONES_DIR, t).is_file(), f"missing {CONE_NAMES[t]}.usd in {CONES_DIR}"


def test_measured_cone_sizes_match_the_geometry():
    sizes = measure_cone_sizes(CONES_DIR)
    assert sizes[ConeType.SMALL_BLUE] == pytest.approx((0.228, 0.228, 0.325), abs=1e-3)
    assert sizes[ConeType.BIG_ORANGE] == pytest.approx((0.285, 0.285, 0.505), abs=1e-3)
    assert sizes[ConeType.SMALL_YELLOW] == sizes[ConeType.SMALL_ORANGE] == sizes[ConeType.SMALL_BLUE]


def test_cone_with_off_centre_origin_is_rejected(tmp_path):
    for t in ConeType:  # a cones dir whose usd origin is not the centre of the base
        stage = Usd.Stage.CreateNew(str(cone_usd_path(tmp_path, t)))
        root = UsdGeom.Xform.Define(stage, f"/{CONE_NAMES[t]}")
        stage.SetDefaultPrim(root.GetPrim())
        cube = UsdGeom.Cube.Define(stage, f"/{CONE_NAMES[t]}/geo")
        cube.GetSizeAttr().Set(0.2)  # centred on the origin: extends below z = 0
        stage.GetRootLayer().Save()
    with pytest.raises(ValueError, match="centre of the cone base"):
        measure_cone_sizes(tmp_path)


# --- scene -----------------------------------------------------------------

def test_scene_has_sensor_at_configured_height_and_pose(scene):
    stage = scene(SensorConfig(height=0.9, x=3.0, y=-2.0, yaw_deg=20.0))
    atx = stage.GetPrimAtPath("/World/ATX")

    assert world_pos(atx) == pytest.approx([3.0, -2.0, 0.9])
    assert world_yaw_deg(atx) == pytest.approx(20.0)
    assert stage.GetPrimAtPath("/World/ATX").GetReferences() is not None
    assert stage.GetDefaultPrim().GetPath() == "/World"


def test_scene_ground_top_is_at_z_zero(scene):
    stage = scene(radius=20.0)  # keep a reference: the prim expires with its stage
    ground = stage.GetPrimAtPath("/World/Ground")
    box = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_]).ComputeWorldBound(ground).ComputeAlignedRange()
    assert box.GetMax()[2] == pytest.approx(0.0)
    assert box.GetMax()[0] == pytest.approx(40.0)  # 4 x radius slab, centred


def test_building_the_scene_twice_overwrites_it(scene, tmp_path):
    scene()
    scene(SensorConfig(height=1.1))  # the second worker of a run rebuilds scene.usd
    stage = Usd.Stage.Open(str(tmp_path / "scene.usd"))
    assert world_pos(stage.GetPrimAtPath("/World/ATX"))[2] == pytest.approx(1.1)


# --- layout ----------------------------------------------------------------

def test_apply_layout_places_cones_relative_to_the_sensor(scene):
    stage = scene(SensorConfig(height=0.9, x=10.0, y=0.0, yaw_deg=90.0))
    waited = []
    cones = [Cone(ConeType.SMALL_BLUE, 2.0, 0.0, 0.0, 15.0), Cone(ConeType.BIG_ORANGE, 0.0, 3.0, 0.0, 0.0)]
    apply_layout(stage, cones, CONES_DIR, lambda: waited.append(1))

    first, second = (stage.GetPrimAtPath(f"{CONES_ROOT}/cone_{i:04d}") for i in range(2))
    assert waited == [1]
    # the group is rotated 90 deg about z, so sensor-frame +x (forward) is world +y
    assert world_pos(first) == pytest.approx([10.0, 2.0, 0.0])
    assert world_pos(second) == pytest.approx([7.0, 0.0, 0.0])
    assert world_yaw_deg(first) == pytest.approx(105.0)  # sensor yaw + cone yaw


def test_apply_layout_replaces_the_previous_layout_and_keeps_the_pose(scene):
    stage = scene(SensorConfig(x=1.0, yaw_deg=45.0))
    apply_layout(stage, [Cone(ConeType.SMALL_BLUE, float(i), 0.0, 0.0) for i in range(5)], CONES_DIR, lambda: None)
    apply_layout(stage, [Cone(ConeType.SMALL_YELLOW, 1.0, 0.0, 0.0)], CONES_DIR, lambda: None)

    root = stage.GetPrimAtPath(CONES_ROOT)
    assert [c.GetName() for c in root.GetChildren()] == ["cone_0000"]
    assert world_yaw_deg(root) == pytest.approx(45.0)
    assert world_pos(root) == pytest.approx([1.0, 0.0, 0.0])


# --- ground material -------------------------------------------------------

def nonvisual(stage, name):
    return stage.GetPrimAtPath("/World/Looks/GroundNonVisual").GetAttribute(f"omni:simready:nonvisual:{name}").Get()


def test_ground_gets_a_bound_nonvisual_material(scene):
    stage = scene()
    apply_ground_material(stage, "asphalt", "paint")

    assert (nonvisual(stage, "base"), nonvisual(stage, "coating"), nonvisual(stage, "attributes")) == ("asphalt", "paint", "none")
    bound = UsdShade.MaterialBindingAPI(stage.GetPrimAtPath("/World/Ground")).ComputeBoundMaterial()[0]
    assert bound.GetPath() == "/World/Looks/GroundNonVisual"


def test_scene_file_has_no_ground_material(scene):
    """Isaac ignores a non-visual material present at load, so the file must not carry one."""
    assert not scene().GetPrimAtPath("/World/Looks")


def test_none_leaves_the_ground_bare(scene):
    stage = scene()
    apply_ground_material(stage, "none", "none")
    assert not stage.GetPrimAtPath("/World/Looks")
