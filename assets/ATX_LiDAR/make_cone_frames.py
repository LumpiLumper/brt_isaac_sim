"""Cone-field dataset: flat ground, N random cones within a radius, one ATX lidar sweep, re-scatter, repeat.

    /isaac-sim/python.sh assets/ATX_LiDAR/make_cone_frames.py --frames 20 --cones 60 --seed 0

Per frame f it writes Outputs/cone_frames/frame_{f:04d}.pcd (same PCD layout as validate_atx.py) and
frame_{f:04d}.json (cone type, x, y, yaw in the sensor/world frame, which share the origin). The sensor sits at
(0, 0, --sensor-height) looking along +X; cones are drawn uniformly over the disc between --min-radius and --radius
and kept --min-gap apart.

Sweep capture follows validate_atx.py: warm-up frames are dropped, slices are merged by timestamp into one full
sweep, and polling stops as soon as one is complete (the 6.0.1 annotator can crash in get_data() after ~15-40
polls). After each re-scatter the script lets --settle render steps pass and discards everything read before that,
so a sweep never mixes two cone layouts. annotator.get_data() segfaults at random, so the script restarts itself
in a child process at the first missing frame until all --frames exist (existing frame files are kept).
"""

import argparse
import json
import os
from pathlib import Path

HERE = Path(__file__).resolve().parent
CONES = HERE.parent / "cones"
CONE_TYPES = ["Small_Cone_Blue", "Small_Cone_Yellow", "Small_Cone_Orange", "Big_Orange_Cone"]

parser = argparse.ArgumentParser()
parser.add_argument("--frames", type=int, default=10, help="number of scenes / lidar frames to produce")
parser.add_argument("--cones", type=int, default=50, help="cones per frame")
parser.add_argument("--radius", type=float, default=100.0, help="cones are placed within this radius of the sensor (m)")
parser.add_argument("--min-radius", type=float, default=2.0, help="keep cones out of this radius around the sensor (m)")
parser.add_argument("--min-gap", type=float, default=0.5, help="minimum cone-to-cone distance (m)")
parser.add_argument("--sensor-height", type=float, default=1.5)
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--start-frame", type=int, default=0, help="first frame index (file naming only)")
parser.add_argument("--settle", type=int, default=6, help="render steps to wait after moving cones before reading")
parser.add_argument("--max-steps", type=int, default=120, help="render steps to wait for a complete sweep per frame")
parser.add_argument("--out-dir", default=str(HERE / "Outputs" / "cone_frames"))
parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
args, _ = parser.parse_known_args()

os.umask(0)  # the container user (uid 1234) writes here: keep outputs deletable from the host account

if not args.worker:
    # annotator.get_data() segfaults now and then (Isaac Sim 6.0.1), which no try/except can catch. So run the real work in
    # a child process and restart it at the first missing frame whenever it dies. Each frame's layout is seeded by
    # (seed, frame index), so a restart reproduces exactly what an uninterrupted run would have made.
    import subprocess
    import sys

    out = Path(args.out_dir)
    wanted = range(args.start_frame, args.start_frame + args.frames)
    failures = 0
    while True:
        todo = [i for i in wanted if not (out / f"frame_{i:04d}.json").exists()]
        if not todo:
            break
        if failures >= 5:
            sys.exit(f"[cones] giving up: 5 worker runs in a row produced no frame; {len(todo)} frames missing")
        cmd = [sys.executable, __file__, "--worker"] + [a for a in sys.argv[1:] if a not in ("--worker",)]
        cmd += ["--start-frame", str(todo[0]), "--frames", str(todo[-1] - todo[0] + 1)]
        subprocess.run(cmd)
        left = [i for i in wanted if not (out / f"frame_{i:04d}.json").exists()]
        failures = failures + 1 if len(left) == len(todo) else 0
        if left:
            print(f"[cones] worker stopped, restarting at frame {left[0]} ({len(left)} left)", flush=True)
    print(f"[cones] all {args.frames} frames present in {out}")
    sys.exit(0)

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading, open_stage  # noqa: E402
from pxr import Gf, Usd, UsdGeom  # noqa: E402

cfg = json.loads((HERE / "atx_emitter_config.json").read_text())
win = cfg["scan"]["azimuthWindowDeg"]

# --- scene ----------------------------------------------------------------------
out_dir = Path(args.out_dir)
out_dir.mkdir(parents=True, exist_ok=True)
scene_path = out_dir / "cone_scene.usd"  # built as a file and opened, as validate_atx.py does with its scene
stage = Usd.Stage.CreateNew(str(scene_path))
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
world = UsdGeom.Xform.Define(stage, "/World")
stage.SetDefaultPrim(world.GetPrim())

ground = UsdGeom.Cube.Define(stage, "/World/Ground")  # flat plane: 1 m thick slab with its top face at z = 0
ground.GetSizeAttr().Set(1.0)
gx = UsdGeom.Xformable(ground)
gx.AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
gx.AddScaleOp().Set(Gf.Vec3f(4 * args.radius, 4 * args.radius, 1))

atx = stage.DefinePrim("/World/ATX", "Xform")
atx.GetReferences().AddReference(str(HERE / "ATX_LiDAR.usd"))
UsdGeom.Xformable(atx).AddTranslateOp().Set(Gf.Vec3d(0, 0, args.sensor_height))
stage.DefinePrim("/World/Cones", "Xform")
stage.GetRootLayer().Save()
del stage
open_stage(str(scene_path))
while is_stage_loading():
    simulation_app.update()
stage = omni.usd.get_context().get_stage()


def scatter(rng):
    """Replace all cones with a fresh random layout; returns the label records."""
    stage.RemovePrim("/World/Cones")
    stage.DefinePrim("/World/Cones", "Xform")
    placed, labels, tries = [], [], 0
    while len(placed) < args.cones:
        tries += 1
        if tries > 200 * args.cones:
            raise RuntimeError("cannot fit that many cones: raise --radius or lower --cones/--min-gap")
        r = np.sqrt(rng.uniform(args.min_radius**2, args.radius**2))  # uniform over the annulus' area
        a = rng.uniform(0, 2 * np.pi)
        p = np.array([r * np.cos(a), r * np.sin(a)])
        if placed and np.min(np.linalg.norm(np.array(placed) - p, axis=1)) < args.min_gap:
            continue
        placed.append(p)
        kind = CONE_TYPES[rng.integers(len(CONE_TYPES))]
        yaw = float(rng.uniform(0, 360))
        prim = stage.DefinePrim(f"/World/Cones/cone_{len(placed) - 1:04d}", "Xform")
        prim.GetReferences().AddReference(str(CONES / f"{kind}.usd"))
        xf = UsdGeom.Xformable(prim)
        xf.AddTranslateOp().Set(Gf.Vec3d(float(p[0]), float(p[1]), 0.0))
        xf.AddRotateZOp().Set(yaw)
        labels.append({"type": kind, "x": float(p[0]), "y": float(p[1]), "yaw_deg": yaw})
    while is_stage_loading():
        simulation_app.update()
    return labels


# --- sensor ---------------------------------------------------------------------
render_product = rep.create.render_product("/World/ATX/sensor", [1, 1], name="atx_cone_frames")
annotator = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
annotator.initialize(outputIntensity=True, outputDistance=True, outputAzimuth=True, outputElevation=True,
                     outputTimestamp=True)
annotator.attach([render_product.path])
app = omni.kit.app.get_app()
timeline = omni.timeline.get_timeline_interface()
timeline.play()

WARMUP_STEPS = 9  # pipeline fill after the first play(); buffers hold garbage until then
SWEEP_GAP_S = 0.04  # sweeps are ~0.033 s long and start every 0.1 s
for _ in range(WARMUP_STEPS):
    app.update()


def find_complete_sweep(slices):
    """Indices into the concatenated valid points of a sweep spanning the whole window and followed by the next one."""
    if not slices:
        return None
    all_ts = np.concatenate([s[2] for s in slices]).astype(np.float64) * 1e-9
    all_az = np.concatenate([s[1]["azimuth"] for s in slices])
    order = np.argsort(all_ts)
    groups = np.split(order, np.where(np.diff(all_ts[order]) > SWEEP_GAP_S)[0] + 1)
    for g in groups[:-1]:  # the last group may still be filling
        # a few late points can reach either edge of the window, so min/max alone passes half sweeps: every 10 deg bin needs points
        bins = np.histogram(all_az[g], bins=np.arange(win[0], win[1] + 1, 10.0))[0]
        if all_az[g].min() <= win[0] + 0.5 and all_az[g].max() >= win[1] - 0.5 and bins.min() >= 50:
            return g
    return None


def capture_sweep():
    """Poll until one complete sweep of the current layout is available; None if --max-steps runs out."""
    for _ in range(args.settle):  # old-layout slices may still be in flight
        app.update()
    slices, seen = [], set()
    for _ in range(args.max_steps):
        app.update()
        raw = annotator.get_data()
        if not raw or "data" not in raw:
            continue
        info = raw.get("info", raw)
        pts = np.asarray(raw["data"])
        if pts.size == 0:
            continue
        n_pts = pts.size // 3
        f = {k: np.asarray(info.get(k, [])).reshape(-1) for k in ("intensity", "distance", "azimuth", "elevation", "timestamp")}
        if any(v.size != n_pts for v in f.values()):
            continue
        valid = f["distance"] > 0
        if not valid.any():
            continue
        az = f["azimuth"].astype(np.float32)
        if az[valid].min() < win[0] - 5 or az[valid].max() > win[1] + 5:
            continue
        ts = f["timestamp"].astype(np.uint64)
        key = (n_pts, int(ts.min()), int(ts.max()))
        if key in seen:
            continue
        seen.add(key)
        slices.append((pts.astype(np.float32).reshape(-1, 3)[valid],
                       {k: f[k].astype(np.float32)[valid] for k in ("intensity", "distance", "azimuth", "elevation")},
                       ts[valid]))
        sweep = find_complete_sweep(slices)
        if sweep is not None:
            xyz = np.concatenate([s[0] for s in slices])[sweep]
            cols = {k: np.concatenate([s[1][k] for s in slices])[sweep] for k in slices[0][1]}
            t = np.concatenate([s[2] for s in slices]).astype(np.float64)[sweep] * 1e-9
            return xyz, cols, t - t.min()
    return None


def write_pcd(path, xyz, cols, t):
    names = ["x", "y", "z", "intensity", "distance", "azimuth", "elevation", "t"]
    n = len(xyz)
    rec = np.empty(n, dtype=np.dtype([(k, "<f4") for k in names]))
    for i, k in enumerate("xyz"):
        rec[k] = xyz[:, i]
    for k in ("intensity", "distance", "azimuth", "elevation"):
        rec[k] = cols[k]
    rec["t"] = t
    header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS " + " ".join(names) + "\nSIZE " + " ".join(["4"] * 8) +
              "\nTYPE " + " ".join(["F"] * 8) + "\nCOUNT " + " ".join(["1"] * 8) + f"\nWIDTH {n}\nHEIGHT 1\n"
              f"VIEWPOINT 0 0 {args.sensor_height} 1 0 0 0\nPOINTS {n}\nDATA binary\n")
    path.write_bytes(header.encode() + rec.tobytes())


# --- main loop --------------------------------------------------------------------
# The first sweep after startup came out with about half the points in testing, so burn one before saving anything.
scatter(np.random.default_rng([args.seed, 999999]))
capture_sweep()
done = 0
for i in range(args.frames):
    idx = args.start_frame + i
    if (out_dir / f"frame_{idx:04d}.json").exists():
        continue
    labels = scatter(np.random.default_rng([args.seed, idx]))
    result = capture_sweep()
    if result is None:
        print(f"[cones] frame {idx}: no complete sweep in {args.max_steps} steps, skipped", flush=True)
        continue
    xyz, cols, t = result
    write_pcd(out_dir / f"frame_{idx:04d}.pcd", xyz, cols, t)
    (out_dir / f"frame_{idx:04d}.json").write_text(json.dumps({"seed": args.seed, "cones": labels}, indent=1))
    done += 1
    print(f"[cones] frame {idx}: {len(labels)} cones, {len(xyz)} points -> {out_dir}/frame_{idx:04d}.pcd", flush=True)

timeline.stop()
print(f"[cones] worker wrote {done} frames", flush=True)
simulation_app.close()
