"""Scan geometry of the ATX sensor, read from its emitter config (assets/ATX_LiDAR/atx_emitter_config.json)."""

import json
from pathlib import Path

from lidar.sweep import SweepSpec

GAP_MARGIN = 1.2  # slices of one sweep span ~1 sweep length; 1.2x of it separates two sweeps


def load_sweep_spec(emitter_config_path: Path) -> SweepSpec:
    scan = json.loads(Path(emitter_config_path).read_text())["scan"]
    lo, hi = scan["azimuthWindowDeg"]
    sweep_s = (hi - lo) / 360.0 / scan["scanRateBaseHz"]  # 120 deg at 10 Hz -> 0.0333 s
    return SweepSpec(azimuth_window=(float(lo), float(hi)), gap_s=GAP_MARGIN * sweep_s)
