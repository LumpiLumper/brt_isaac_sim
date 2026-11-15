"""Step 4a: build ATX_test_scene.usd, a ground plane plus a wall 80 m ahead, with the ATX referenced at z = 1.5 m.

    /isaac-sim/python.sh assets/ATX_LiDAR/make_test_scene.py

Why this layout: the wall at x = 80 m is beyond the 60 m channels' range and inside the 90 m+ channels', so
the range groups show up as a split in the wall returns. The ground gives a range sweep with elevation.
"""

from pathlib import Path

HERE = Path(__file__).resolve().parent

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Gf, Usd, UsdGeom  # noqa: E402

out = HERE / "ATX_test_scene.usd"
stage = Usd.Stage.CreateNew(str(out))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())


def box(path, center, size):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.GetSizeAttr().Set(1.0)
    xf = UsdGeom.Xformable(cube)
    xf.AddTranslateOp().Set(Gf.Vec3d(*center))
    xf.AddScaleOp().Set(Gf.Vec3f(*size))


box("/World/Ground", (0, 0, -0.5), (1000, 1000, 1))  # top face at z = 0
box("/World/Wall", (80.5, 0, 15), (1, 200, 30))  # face at x = 80

atx = stage.DefinePrim("/World/ATX", "Xform")
atx.GetReferences().AddReference("./ATX_LiDAR.usd")
UsdGeom.Xformable(atx).AddTranslateOp().Set(Gf.Vec3d(0, 0, 1.5))

stage.GetRootLayer().Save()
print(f"[atx] wrote {out}; sensor prim is /World/ATX/sensor")
simulation_app.close()
