"""Step 3: build ATX_LiDAR.usd (an OmniLidar prim) from atx_emitter_config.json.

Run with Isaac Sim's Python (from the repo root, inside the container or via compose):

    /isaac-sim/python.sh assets/ATX_LiDAR/make_atx_usd.py [--window-start -60 --window-end 60]

Output: assets/ATX_LiDAR/ATX_LiDAR.usd with
    /ATX            Xform, default prim (place/move this one)
    /ATX/sensor     OmniLidar, 116 emitters, boresight along +X
"""

import argparse
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent

parser = argparse.ArgumentParser()
parser.add_argument("--config", default=str(HERE / "atx_emitter_config.json"))
parser.add_argument("--out", default=str(HERE / "ATX_LiDAR.usd"))
parser.add_argument("--window-start", type=float, default=None, help="valid azimuth start [deg]; default -FOV/2")
parser.add_argument("--window-end", type=float, default=None, help="valid azimuth end [deg]; default +FOV/2")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

from pxr import Gf, Sdf, Usd, UsdGeom, Vt  # noqa: E402

cfg = json.loads(Path(args.config).read_text())
scan, core, em = cfg["scan"], cfg["core"], cfg["emitterState"]
n = core["numberOfEmitters"]
win = scan["azimuthWindowDeg"]
win_start = win[0] if args.window_start is None else args.window_start
win_end = win[1] if args.window_end is None else args.window_end

stage = Usd.Stage.CreateNew(args.out)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
root = UsdGeom.Xform.Define(stage, "/ATX")
stage.SetDefaultPrim(root.GetPrim())

prim = stage.DefinePrim("/ATX/sensor", "OmniLidar")
for api, inst in (
    ("OmniSensorAPI", None),
    ("OmniSensorGenericLidarCoreAPI", None),
    ("OmniSensorGenericLidarCoreEmitterStateAPI", "s001"),
):
    api_type = Usd.SchemaRegistry.GetAPITypeFromSchemaTypeName(api)
    ok = prim.ApplyAPI(api_type, inst) if inst else prim.ApplyAPI(api_type)
    if not ok:
        raise RuntimeError(f"could not apply {api}")

CORE = "omni:sensor:Core:"
STATE = CORE + "emitterState:s001:"
failures = []


def put(name, value):
    """Set an attribute that the schema already defines; the schema supplies its type."""
    attr = prim.GetAttribute(name)
    if not attr or not attr.IsValid():
        failures.append(name)
        print(f"[atx] MISSING attribute {name}")
        return
    t = str(attr.GetTypeName())
    if t.endswith("[]"):
        value = {"float[]": Vt.FloatArray, "uint[]": Vt.UIntArray}[t](list(value))
    attr.Set(value)


# --- scalars ----------------------------------------------------------------
put(CORE + "scanType", scan["scanType"])
put(CORE + "scanRateBaseHz", int(scan["scanRateBaseHz"]))
put(CORE + "patternFiringRateHz", int(scan["patternFiringRateHz"]))
put(CORE + "stateResolutionStep", 1)
# Isaac Sim 6.0 wants 0..360; a negative start is not read as a wrapped window (-60..60 yields only half of it)
put(CORE + "validStartAzimuthDeg", float(win_start % 360.0))
put(CORE + "validEndAzimuthDeg", float(win_end % 360.0))
put(CORE + "numberOfEmitters", n)
put(CORE + "numberOfChannels", n)
put(CORE + "nearRangeM", core["nearRangeM"])
put(CORE + "farRangeM", core["farRangeM"])
put(CORE + "rangeAccuracyM", core["rangeAccuracyM"])
put(CORE + "waveLengthNm", core["waveLengthNm"])
put(CORE + "maxReturns", core["maxReturns"])
put(CORE + "rangeCount", core["rangeCount"])
put(CORE + "rangesMinM", core["rangesMinM"])
put(CORE + "rangesMaxM", core["rangesMaxM"])

# --- per-emitter arrays -----------------------------------------------------
put(STATE + "channelId", em["channelId"])
put(STATE + "azimuthDeg", em["azimuthDeg"])
put(STATE + "elevationDeg", em["elevationDeg"])
put(STATE + "fireTimeNs", em["fireTimeNs"])
put(STATE + "rangeId", em["rangeId"])
# the schema defaults are sized for another sensor; reset the remaining per-emitter arrays to n zeros
for name in ("distanceCorrectionM", "focalDistM", "focalSlope", "horOffsetM", "vertOffsetM"):
    put(STATE + name, [0.0] * n)

# --- identification ---------------------------------------------------------
for name, value in (
    ("omni:sensor:modelVendor", "Hesai"),
    ("omni:sensor:modelName", "ATX"),
    ("omni:sensor:marketName", "ATX"),
):
    put(name, value)
put("omni:sensor:tickRate", float(scan["scanRateBaseHz"]))

stage.GetRootLayer().Save()

# --- read back ----------------------------------------------------------------
check_stage = Usd.Stage.Open(args.out)
check = check_stage.GetPrimAtPath("/ATX/sensor")
print(f"[atx] wrote {args.out}")
print(f"[atx] prim type {check.GetTypeName()}, schemas {check.GetAppliedSchemas()}")
for name in (CORE + "numberOfEmitters", CORE + "scanType", CORE + "validStartAzimuthDeg", CORE + "validEndAzimuthDeg",
             CORE + "patternFiringRateHz", CORE + "rangeCount"):
    print(f"[atx] {name} = {check.GetAttribute(name).Get()}")
for name in ("channelId", "azimuthDeg", "elevationDeg", "fireTimeNs", "rangeId"):
    print(f"[atx] {name}: {len(check.GetAttribute(STATE + name).Get())} values")
print(f"[atx] missing attributes: {failures or 'none'}")

simulation_app.close()
