"""Capture exactly one full RTX LiDAR scan from a USD stage and save it as PCD.

Isaac Sim 5.1 standalone script. Run with Isaac Sim's own Python:

    <isaac-sim-root>/python.sh capture_one_scan.py \
        --usd OS1_LiDAR_in_Simple_Room.usd \
        --sensor /Root/OS1/sensor \
        --out scan_0000.pcd --headless

Output: binary PCD with fields x y z intensity distance azimuth elevation t
  - points are as delivered by IsaacCreateRTXLidarScanBuffer (sensor frame)
  - t = per-point time offset from first point of the scan [s]
  - VIEWPOINT header = sensor world pose at capture time (tx ty tz qw qx qy qz)
"""

import argparse
import hashlib
import os

parser = argparse.ArgumentParser()
parser.add_argument("--usd", required=True, help="Path to the .usd stage")
parser.add_argument("--sensor", default="/Root/OS1/sensor", help="Prim path of the OmniLidar prim")
parser.add_argument("--out", default="scan_0000.pcd", help="Output .pcd path")
parser.add_argument("--skip", type=int, default=1, help="Number of complete scans to discard before saving (warm-up)")
parser.add_argument("--max-frames", type=int, default=600, help="Give up after this many app updates")
parser.add_argument("--kitti-bin", action="store_true", help="Also write <out>.bin as float32 Nx4 (x y z intensity)")
parser.add_argument("--headless", action="store_true")
args, _ = parser.parse_known_args()

from isaacsim import SimulationApp  # noqa: E402

simulation_app = SimulationApp({"headless": args.headless})

import numpy as np  # noqa: E402
import omni.kit.app  # noqa: E402
import omni.replicator.core as rep  # noqa: E402
import omni.timeline  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.utils.stage import is_stage_loading, open_stage  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402


def unpack(raw):
    """Flatten annotator output into {field: ndarray}. Handles both flat and {'data', 'info'} layouts."""
    if raw is None:
        return {}
    if not isinstance(raw, dict):
        return {"data": np.asarray(raw)}
    out = {}
    info = raw.get("info")
    if isinstance(info, dict):
        out.update(info)
    for k, v in raw.items():
        if k != "info":
            out[k] = v
    return out


def scan_is_clean(fields, n_bins=36, max_ratio=1.4):
    """False if the buffer mixes two sweeps (some azimuth sectors ~2x denser than the rest).

    A buffer read mid-update holds the end of one sweep plus the start of the next, which shows up
    as a doubled sector next to an empty one. Only the density of occupied sectors is compared, so
    limited-FOV (solid-state) sensors pass. Returns True when there is no azimuth data to check.
    """
    az = np.asarray(fields.get("azimuth", []), dtype=np.float32).reshape(-1)
    if az.size == 0:
        return True
    counts, _ = np.histogram(az, bins=n_bins, range=(az.min(), az.max() + 1e-6))
    counts = counts[counts > 0]
    return counts.max() <= max_ratio * np.median(counts)


def write_pcd_binary(path, columns, viewpoint):
    """columns: ordered list of (name, 1-D float32 array). viewpoint: (tx,ty,tz,qw,qx,qy,qz)."""
    n = len(columns[0][1])
    dtype = np.dtype([(name, "<f4") for name, _ in columns])
    rec = np.empty(n, dtype=dtype)
    for name, arr in columns:
        rec[name] = arr
    names = [c[0] for c in columns]
    header = (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION 0.7\n"
        f"FIELDS {' '.join(names)}\n"
        f"SIZE {' '.join(['4'] * len(names))}\n"
        f"TYPE {' '.join(['F'] * len(names))}\n"
        f"COUNT {' '.join(['1'] * len(names))}\n"
        f"WIDTH {n}\n"
        "HEIGHT 1\n"
        f"VIEWPOINT {' '.join(f'{v:.9g}' for v in viewpoint)}\n"
        f"POINTS {n}\n"
        "DATA binary\n"
    )
    with open(path, "wb") as f:
        f.write(header.encode("ascii"))
        f.write(rec.tobytes())


# --- open stage ------------------------------------------------------------
usd_path = os.path.abspath(args.usd)
open_stage(usd_path)
while is_stage_loading():
    simulation_app.update()

stage = omni.usd.get_context().get_stage()
sensor_prim = stage.GetPrimAtPath(args.sensor)
if not sensor_prim.IsValid():
    raise RuntimeError(f"Sensor prim {args.sensor} not found in {usd_path}")
print(f"[capture] sensor prim: {args.sensor} (type {sensor_prim.GetTypeName()})")

# --- render product + annotator -------------------------------------------
render_product = rep.create.render_product(args.sensor, [1, 1], name="lidar_capture")
annotator = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
annotator.initialize(
    outputIntensity=True,
    outputDistance=True,
    outputAzimuth=True,
    outputElevation=True,
    outputTimestamp=True,
)
annotator.attach([render_product.path])

# --- run until one complete (non-warm-up) scan arrives ----------------------
app = omni.kit.app.get_app()
timeline = omni.timeline.get_timeline_interface()
timeline.play()

last_digest = None
scans_seen = 0
fields = None
for frame in range(args.max_frames):
    app.update()
    d = unpack(annotator.get_data())
    pts = np.asarray(d.get("data", []), dtype=np.float32)
    if pts.size == 0:
        continue
    digest = hashlib.md5(pts.tobytes()).digest()
    if digest == last_digest:  # same scan returned again, wait for the next one
        continue
    last_digest = digest
    if not scan_is_clean(d):
        print(f"[capture] frame {frame}: buffer mixes two sweeps, waiting for the next one")
        continue
    scans_seen += 1
    print(f"[capture] frame {frame}: complete scan #{scans_seen} with {pts.size // 3} points")
    if scans_seen > args.skip:
        fields = d
        break

timeline.stop()

if fields is None:
    simulation_app.close()
    raise RuntimeError(
        f"No complete scan after {args.max_frames} frames. "
        "Check that the sensor prim is an OmniLidar and that the timeline actually plays."
    )

# --- assemble columns ------------------------------------------------------
xyz = np.asarray(fields["data"], dtype=np.float32).reshape(-1, 3)
n = xyz.shape[0]
columns = [("x", xyz[:, 0]), ("y", xyz[:, 1]), ("z", xyz[:, 2])]
for key in ("intensity", "distance", "azimuth", "elevation"):
    arr = np.asarray(fields.get(key, []), dtype=np.float32).reshape(-1)
    if arr.size == n:
        columns.append((key, arr))
ts = np.asarray(fields.get("timestamp", []), dtype=np.uint64).reshape(-1)
if ts.size == n:
    columns.append(("t", ((ts - ts.min()).astype(np.float64) * 1e-9).astype(np.float32)))

# sensor world pose -> PCD VIEWPOINT
m = UsdGeom.Xformable(sensor_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
t = m.ExtractTranslation()
q = m.ExtractRotationQuat()
viewpoint = (t[0], t[1], t[2], q.GetReal(), *q.GetImaginary())

out_path = os.path.abspath(args.out)
write_pcd_binary(out_path, columns, viewpoint)
print(f"[capture] wrote {n} points, fields {[c[0] for c in columns]} -> {out_path}")
print(f"[capture] z range {xyz[:, 2].min():.3f} .. {xyz[:, 2].max():.3f} (sanity check for frame)")

if args.kitti_bin:
    inten = dict(columns).get("intensity", np.zeros(n, dtype=np.float32))
    bin_path = os.path.splitext(out_path)[0] + ".bin"
    np.column_stack([xyz, inten]).astype(np.float32).tofile(bin_path)
    print(f"[capture] wrote KITTI-style float32 Nx4 -> {bin_path}")

simulation_app.close()
