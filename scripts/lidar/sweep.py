"""Assemble one complete lidar sweep out of the partial slices the RTX scan-buffer annotator returns.

Pure numpy: feed it the dicts from annotator.get_data(), get a Frame back. The scan geometry (azimuth window, sweep
length) is sensor specific and arrives as a SweepSpec; see lidar/atx.py for the ATX one.
"""

from dataclasses import dataclass

import numpy as np

from utils.cone import Cone
from utils.frame import DTYPE, Frame

COLUMNS = ("intensity", "distance", "azimuth", "elevation")


@dataclass(frozen=True)
class SweepSpec:
    azimuth_window: tuple[float, float]  # deg, boresight at 0
    gap_s: float                         # a time gap larger than this separates two sweeps
    bin_deg: float = 10.0                # every azimuth bin of this width needs points ...
    min_points_per_bin: int = 50         # ... at least this many, or the sweep is called incomplete
    edge_tol_deg: float = 0.5            # the sweep must reach both window edges within this
    slack_deg: float = 5.0               # slices with points further outside the window are garbage


@dataclass
class Slice:
    xyz: np.ndarray                  # (n, 3) float32, valid points only
    cols: dict[str, np.ndarray]      # COLUMNS -> (n,) float32
    ts: np.ndarray                   # (n,) uint64 ns
    key: tuple                       # identifies the raw read, to drop repeats


@dataclass
class Sweep:
    xyz: np.ndarray
    cols: dict[str, np.ndarray]
    t: np.ndarray                    # seconds since the sweep's first point


def parse_slice(raw, spec: SweepSpec) -> Slice | None:
    """Validate one annotator.get_data() result; None if it is empty, inconsistent or outside the scan window."""
    if not raw or "data" not in raw:
        return None
    info = raw.get("info", raw)
    pts = np.asarray(raw["data"])
    if pts.size == 0:
        return None
    n = pts.size // 3
    f = {k: np.asarray(info.get(k, [])).reshape(-1) for k in (*COLUMNS, "timestamp")}
    if any(v.size != n for v in f.values()):
        return None
    valid = f["distance"] > 0
    if not valid.any():
        return None
    az = f["azimuth"].astype(np.float32)
    lo, hi = spec.azimuth_window
    if az[valid].min() < lo - spec.slack_deg or az[valid].max() > hi + spec.slack_deg:
        return None
    ts = f["timestamp"].astype(np.uint64)
    return Slice(
        xyz=pts.astype(np.float32).reshape(-1, 3)[valid],
        cols={k: f[k].astype(np.float32)[valid] for k in COLUMNS},
        ts=ts[valid],
        key=(n, int(ts.min()), int(ts.max())),
    )


def find_complete_sweep(slices: list[Slice], spec: SweepSpec) -> np.ndarray | None:
    """Indices into the concatenated slices of a sweep that spans the whole window and is followed by the next one."""
    if not slices:
        return None
    ts = np.concatenate([s.ts for s in slices]).astype(np.float64) * 1e-9
    az = np.concatenate([s.cols["azimuth"] for s in slices])
    order = np.argsort(ts)
    groups = np.split(order, np.where(np.diff(ts[order]) > spec.gap_s)[0] + 1)
    lo, hi = spec.azimuth_window
    edges = np.arange(lo, hi + 1, spec.bin_deg)
    for g in groups[:-1]:  # the last group may still be filling
        # a few late points can reach either edge, so min/max alone passes half sweeps: every bin needs points
        bins = np.histogram(az[g], bins=edges)[0]
        if az[g].min() <= lo + spec.edge_tol_deg and az[g].max() >= hi - spec.edge_tol_deg \
                and bins.min() >= spec.min_points_per_bin:
            return g
    return None


class SweepAssembler:
    """Collects slices for one frame; add() returns the Sweep as soon as a complete one is available."""

    def __init__(self, spec: SweepSpec):
        self.spec = spec
        self.slices: list[Slice] = []
        self._seen: set[tuple] = set()

    def add(self, raw) -> Sweep | None:
        s = parse_slice(raw, self.spec)
        if s is None or s.key in self._seen:
            return None
        self._seen.add(s.key)
        self.slices.append(s)
        g = find_complete_sweep(self.slices, self.spec)
        if g is None:
            return None
        xyz = np.concatenate([s.xyz for s in self.slices])[g]
        cols = {k: np.concatenate([s.cols[k] for s in self.slices])[g] for k in COLUMNS}
        t = np.concatenate([s.ts for s in self.slices]).astype(np.float64)[g] * 1e-9
        return Sweep(xyz, cols, t - t.min())


def to_frame(sweep: Sweep, cones: list[Cone], sensor_height: float, seed: int = 0, idx: int = 0) -> Frame:
    data = np.empty(len(sweep.xyz), dtype=DTYPE)
    for i, k in enumerate("xyz"):
        data[k] = sweep.xyz[:, i]
    for k in COLUMNS:
        data[k] = sweep.cols[k]
    data["t"] = sweep.t
    return Frame(data=data, cones=cones, sensor_height=sensor_height, seed=seed, idx=idx)
