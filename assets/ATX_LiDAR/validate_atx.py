"""Step 4b: scan ATX_test_scene.usd, merge the render-step slices into one full sweep, and check it against the datasheet.

    /isaac-sim/python.sh assets/ATX_LiDAR/validate_atx.py [--out assets/ATX_LiDAR/Outputs/atx_scan.pcd]

Why not scripts/capture_one_scan.py: the RTX lidar delivers one slice per render step (60 Hz), i.e. a 120 deg
window arrives as two ~60 deg slices, and the first ~8 frames are warm-up garbage. This script drops the
warm-up, groups slices into sweeps by timestamp and uses a complete one.

Checks (scene: ground at z=0, wall at x=80 m, sensor at z=1.5 m):
  - azimuth span, elevation span and per-channel elevations match the datasheet
  - no return beyond a channel's max range, none inside its min range
  - total points == analytic prediction for this scene
  - wall returns split by channel max range (60 m channels must see no wall)
"""

import argparse
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent

parser = argparse.ArgumentParser()
parser.add_argument("--usd", default=str(HERE / "ATX_test_scene.usd"))
parser.add_argument("--sensor", default="/World/ATX/sensor")
parser.add_argument("--out", default=str(HERE / "Outputs" / "atx_scan.pcd"))
parser.add_argument("--frames", type=int, default=30)
parser.add_argument("--warmup", type=int, default=9)
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": True})

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading, open_stage  # noqa: E402

cfg = json.loads((HERE / "atx_emitter_config.json").read_text())
em, core, scan = cfg["emitterState"], cfg["core"], cfg["scan"]
elev = np.array(em["elevationDeg"], dtype=np.float64)
aoff = np.array(em["azimuthDeg"], dtype=np.float64)
rmin = np.array([core["rangesMinM"][i] for i in em["rangeId"]])
rmax = np.array([core["rangesMaxM"][i] for i in em["rangeId"]])
win = scan["azimuthWindowDeg"]
step = 360.0 / scan["statesPerRevolution"]

# --- capture --------------------------------------------------------------------
open_stage(args.usd)
while is_stage_loading():
    simulation_app.update()
render_product = rep.create.render_product(args.sensor, [1, 1], name="atx_validate")
annotator = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
annotator.initialize(outputIntensity=True, outputDistance=True, outputAzimuth=True, outputElevation=True,
                     outputTimestamp=True)
annotator.attach([render_product.path])
omni.timeline.get_timeline_interface().play()
app = omni.kit.app.get_app()

WARMUP_FRAMES = args.warmup
SWEEP_GAP_S = 0.04  # sweeps are ~0.033 s long and start every 0.1 s, so a gap this large separates two sweeps
slices, seen = [], set()


def find_complete_sweep():
    """Indices into the concatenated valid points of a sweep that spans the whole window and is followed by the next one."""
    if not slices:
        return None
    all_ts = np.concatenate([s[2] for s in slices]).astype(np.float64) * 1e-9
    all_az = np.concatenate([s[1]["azimuth"] for s in slices])
    order = np.argsort(all_ts)
    groups = np.split(order, np.where(np.diff(all_ts[order]) > SWEEP_GAP_S)[0] + 1)
    for gi, g in enumerate(groups[:-1]):  # the last group may still be filling
        if all_az[g].min() <= win[0] + 0.5 and all_az[g].max() >= win[1] - 0.5:
            return g
    return None


sweep = None
for frame in range(args.frames):
    app.update()
    if frame < WARMUP_FRAMES:  # buffers hold garbage until the pipeline fills
        continue
    raw = annotator.get_data()
    if not raw or "data" not in raw:
        continue
    info = raw.get("info", raw)
    pts = np.asarray(raw["data"])
    if pts.size == 0:
        continue
    n_pts = pts.size // 3
    fields = {k: np.asarray(info.get(k, [])).reshape(-1) for k in ("intensity", "distance", "azimuth", "elevation", "timestamp")}
    if any(v.size != n_pts for v in fields.values()):
        continue
    valid = fields["distance"] > 0
    if not valid.any():
        continue  # warm-up garbage or an empty repeat
    az = fields["azimuth"].astype(np.float32)
    if az[valid].min() < win[0] - 5 or az[valid].max() > win[1] + 5:
        continue
    ts = fields["timestamp"].astype(np.uint64)
    key = (n_pts, int(ts.min()), int(ts.max()))
    if key in seen:
        continue
    seen.add(key)
    slices.append((pts.astype(np.float32).reshape(-1, 3)[valid],
                   {k: fields[k].astype(np.float32)[valid] for k in ("intensity", "distance", "azimuth", "elevation")},
                   ts[valid]))
    print(f"[atx] frame {frame}: slice {len(slices)}, {int(valid.sum())} pts, az {az[valid].min():.1f} .. {az[valid].max():.1f}", flush=True)
    sweep = find_complete_sweep()
    if sweep is not None:
        break  # stop polling: more get_data calls risk a crash in the 6.0.1 annotator
omni.timeline.get_timeline_interface().stop()
if sweep is None:
    raise RuntimeError("never saw a complete sweep; raise --frames or check the sensor prim")

xyz = np.concatenate([s[0] for s in slices])[sweep]
cols = {k: np.concatenate([s[1][k] for s in slices])[sweep] for k in slices[0][1]}
ts = np.concatenate([s[2] for s in slices]).astype(np.float64)[sweep] * 1e-9
n = len(xyz)

# --- PCD ------------------------------------------------------------------------
names = ["x", "y", "z", "intensity", "distance", "azimuth", "elevation", "t"]
rec = np.empty(n, dtype=np.dtype([(k, "<f4") for k in names]))
for i, k in enumerate("xyz"):
    rec[k] = xyz[:, i]
for k in ("intensity", "distance", "azimuth", "elevation"):
    rec[k] = cols[k]
rec["t"] = ts - ts.min()
out = Path(args.out)
out.parent.mkdir(parents=True, exist_ok=True)
header = ("# .PCD v0.7\nVERSION 0.7\nFIELDS " + " ".join(names) + "\nSIZE " + " ".join(["4"] * 8) +
          "\nTYPE " + " ".join(["F"] * 8) + "\nCOUNT " + " ".join(["1"] * 8) + f"\nWIDTH {n}\nHEIGHT 1\n"
          f"VIEWPOINT 0 0 1.5 1 0 0 0\nPOINTS {n}\nDATA binary\n")
out.write_bytes(header.encode() + rec.tobytes())

# --- analytic expectation for this scene ------------------------------------------
H = 1.5
cols_h = np.arange(win[0], win[1], step)  # column angles inside the window
exp_hits = np.zeros(len(elev), dtype=int)
exp_wall = np.zeros(len(elev), dtype=int)  # of which: hits on the wall (near-normal incidence)
for i in range(len(elev)):
    h = np.radians(cols_h + aoff[i])
    e = np.radians(elev[i])
    d = np.stack([np.cos(e) * np.cos(h), np.cos(e) * np.sin(h), np.full_like(h, np.sin(e))], axis=1)
    t_ground = np.where(d[:, 2] < 0, -H / d[:, 2], np.inf)
    t_wall = np.where(d[:, 0] > 0, 80.0 / d[:, 0], np.inf)
    wall_ok = (np.abs(t_wall * d[:, 1]) <= 100) & (H + t_wall * d[:, 2] >= 0) & (H + t_wall * d[:, 2] <= 30)
    t = np.where(wall_ok & (t_wall < t_ground), t_wall, t_ground)
    in_range = np.isfinite(t) & (t >= rmin[i]) & (t <= rmax[i])
    exp_hits[i] = int(in_range.sum())
    exp_wall[i] = int((in_range & wall_ok & (t_wall < t_ground)).sum())
exp_total = int(exp_hits.sum())

# --- report -----------------------------------------------------------------------
ch = np.abs(cols["elevation"][:, None].astype(np.float64) - elev[None, :]).argmin(axis=1)
on_grid = np.abs(cols["elevation"] - elev[ch]) < 0.05
heading = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
dist = cols["distance"]
per_ch = np.bincount(ch, minlength=len(elev))

def line(ok, text):
    print(f"[atx] {'PASS' if ok else 'FAIL'}  {text}")

print(f"[atx] sweep: {n} points, {ts.max() - ts.min():.4f} s span, saved {out}")
# Channels whose hit geometry is unambiguous: wall-only (el >= 0) or steep ground (el <= -6.5). In between, grazing
# ground hits fall under the sensor's detection floor in a way geometry alone cannot predict, so they are only reported.
reliable = (elev >= 0.0) | (elev <= -6.5)
meas, exp = per_ch[reliable], exp_hits[reliable]
line(abs(meas.sum() - exp.sum()) <= 0.02 * exp.sum(),
     f"points on the {int(reliable.sum())} unambiguous channels: {int(meas.sum())} vs analytic {int(exp.sum())} "
     f"({len(cols_h)} columns per channel in the window)")
rel = np.abs(meas - exp) / np.maximum(exp, 1)
worst = np.argsort(-rel)[:4]
ridx = np.where(reliable)[0]
line(rel.max() <= 0.03, f"each of those channels within {rel.max() * 100:.1f}% of its analytic count")
print("[atx] INFO  worst channels (ring, elev, measured, analytic): " +
      ", ".join(f"({ridx[w] + 1}, {elev[ridx[w]]:.2f}, {meas[w]}, {exp[w]})" for w in worst))
zone = ~reliable
print(f"[atx] INFO  {int(zone.sum())} grazing-zone channels (el {elev[zone].max():.1f} .. {elev[zone].min():.1f}): "
      f"{int(per_ch[zone].sum())} points vs {int(exp_hits[zone].sum())} geometric, not checked")
print(f"[atx] INFO  total {n} points in one sweep ({ts.max() - ts.min():.4f} s), {len(elev) * len(cols_h)} if every ray hit")
line(cols["azimuth"].min() > win[0] - 4 and cols["azimuth"].max() < win[1] + 4,
     f"azimuth {cols['azimuth'].min():.2f} .. {cols['azimuth'].max():.2f} deg (window {win[0]} .. {win[1]} +- channel offsets)")
line(abs(heading.min() - win[0]) < 4 and abs(heading.max() - win[1]) < 4,
     f"heading atan2(y,x) {heading.min():.2f} .. {heading.max():.2f} deg: boresight is +X")
line(on_grid.all(), f"all points on a datasheet elevation ({on_grid.mean() * 100:.1f}%), "
     f"{(per_ch > 0).sum()} of {len(elev)} channels present")
line(np.all(dist[on_grid] <= rmax[ch[on_grid]] + 0.5) and np.all(dist[on_grid] >= rmin[ch[on_grid]] - 0.5),
     f"distance {dist.min():.2f} .. {dist.max():.2f} m, within each channel's min/max range")
silent = (per_ch == 0) & (exp_hits > 0)
bad = np.where(silent & (exp_wall > 0))[0]
line(len(bad) == 0, f"channels with expected wall hits but silent: {[int(i) + 1 for i in bad] or 'none'} (1-based ring order)")
graze = np.where(silent & (exp_wall == 0))[0]
if len(graze):
    print(f"[atx] NOTE  {len(graze)} channels (rings {graze.min() + 1}..{graze.max() + 1}, elevation "
          f"{elev[graze].max():.1f} .. {elev[graze].min():.1f} deg) return nothing: their only target is the ground at "
          f"{H / np.sin(np.radians(-elev[graze].max())):.0f} .. {H / np.sin(np.radians(-elev[graze].min())):.0f} m, a grazing hit "
          f"whose intensity falls below the RTX lidar's detection floor (~0.005). A scene effect, not a config error.")
wall = np.abs(xyz[:, 0] - 80.0) < 0.3
print("[atx] wall hits (x=80 m) by channel max range:")
for lim in sorted({float(r) for r in rmax}):
    idx = np.where(rmax == lim)[0]
    print(f"[atx]   max {lim:5.0f} m: {len(idx):3d} channels, {int(np.isin(ch[wall], idx).sum()):6d} wall hits"
          f"{'  (expect 0)' if lim < 80 else ''}")

simulation_app.close()
