import json

import numpy as np
import pytest

from config import REPO_ROOT, SensorConfig, resolve_path
from lidar.atx import load_sweep_spec
from lidar.sweep import SweepAssembler, SweepSpec, parse_slice, to_frame
from utils.cone import Cone, ConeType
from utils.frame import DTYPE

SPEC = SweepSpec(azimuth_window=(-60.0, 60.0), gap_s=0.04)


def raw_slice(az_lo, az_hi, t0_s, n=400, dur_s=0.01, distance=5.0):
    """What annotator.get_data() returns: n points spread over an azimuth range and a short time span."""
    az = np.linspace(az_lo, az_hi, n, dtype=np.float32)
    ts = (t0_s + np.linspace(0, dur_s, n)) * 1e9
    xyz = np.column_stack([np.cos(np.radians(az)), np.sin(np.radians(az)), np.zeros(n)]) * distance
    return {"data": xyz.astype(np.float32), "info": {
        "intensity": np.ones(n, np.float32), "distance": np.full(n, distance, np.float32),
        "azimuth": az, "elevation": np.zeros(n, np.float32), "timestamp": ts.astype(np.uint64)}}


def full_sweep(t0_s):
    """Three slices that together cover the whole -60..60 window, 100 points per 10 deg bin."""
    return [raw_slice(-60, -20, t0_s), raw_slice(-20, 20, t0_s + 0.011), raw_slice(20, 60, t0_s + 0.022)]


def feed(raws, spec=SPEC):
    asm = SweepAssembler(spec)
    out = None
    for r in raws:
        out = asm.add(r) or out
    return out


# --- parse_slice -----------------------------------------------------------

def test_parse_slice_keeps_valid_points_only():
    raw = raw_slice(-10, 10, 0.0, n=100)
    raw["info"]["distance"][:30] = 0  # misses
    s = parse_slice(raw, SPEC)
    assert len(s.xyz) == 70 and all(len(v) == 70 for v in s.cols.values()) and len(s.ts) == 70


@pytest.mark.parametrize("make_raw", [
    lambda: None,
    lambda: {},
    lambda: {"data": np.zeros((0, 3))},
    lambda: {**raw_slice(-10, 10, 0, n=50), "info": {**raw_slice(-10, 10, 0, n=50)["info"], "distance": np.ones(7)}},
    lambda: {**raw_slice(-10, 10, 0, n=50), "info": {**raw_slice(-10, 10, 0, n=50)["info"], "distance": np.zeros(50)}},
    lambda: raw_slice(-90, -80, 0.0),  # outside the window
])
def test_parse_slice_rejects_garbage(make_raw):
    assert parse_slice(make_raw(), SPEC) is None


# --- sweep assembly --------------------------------------------------------

def test_full_sweep_followed_by_next_is_accepted():
    sweep = feed(full_sweep(0.0) + [raw_slice(-60, -50, 0.1)])  # next sweep starts 0.1 s later
    assert sweep is not None
    assert len(sweep.xyz) == 1200  # only the first sweep's points, not the following slice
    assert sweep.t.min() == 0.0 and 0.02 < sweep.t.max() < 0.04


def test_sweep_is_not_reported_before_the_next_one_starts():
    assert feed(full_sweep(0.0)) is None  # the last group may still be filling


def test_half_sweep_is_rejected():
    assert feed([raw_slice(-60, 0, 0.0), raw_slice(-60, -50, 0.1)]) is None


def test_hole_in_one_azimuth_bin_is_rejected():
    holed = [raw_slice(-60, -20, 0.0), raw_slice(-20, 20, 0.011), raw_slice(30, 60, 0.022)]
    assert feed(holed + [raw_slice(-60, -50, 0.1)]) is None


def test_sparse_bin_is_rejected():
    thin = [raw_slice(-60, -20, 0.0), raw_slice(-20, 20, 0.011, n=40), raw_slice(20, 60, 0.022)]  # 10 per bin
    assert feed(thin + [raw_slice(-60, -50, 0.1)]) is None


def test_repeated_reads_are_ignored():
    r = raw_slice(-60, 60, 0.0)
    asm = SweepAssembler(SPEC)
    asm.add(r)
    asm.add(r)
    assert len(asm.slices) == 1


def test_slices_from_a_stale_sweep_do_not_leak_into_the_result():
    stale = [raw_slice(-60, -30, -0.1)]  # leftover of the previous layout, a whole period earlier
    sweep = feed(stale + full_sweep(0.0) + [raw_slice(-60, -50, 0.1)])
    assert sweep is not None and len(sweep.xyz) == 1200


def test_bin_size_and_point_floor_come_from_the_spec():
    strict = SweepSpec(azimuth_window=(-60.0, 60.0), gap_s=0.04, min_points_per_bin=500)
    assert feed(full_sweep(0.0) + [raw_slice(-60, -50, 0.1)], strict) is None


# --- to_frame --------------------------------------------------------------

def test_to_frame_builds_structured_frame():
    sweep = feed(full_sweep(0.0) + [raw_slice(-60, -50, 0.1)])
    cones = [Cone(ConeType.SMALL_BLUE, 1.0, 2.0, 0.0)]
    frame = to_frame(sweep, cones, sensor_height=0.4, seed=7, idx=3)

    assert frame.data.dtype == DTYPE and len(frame.data) == 1200
    np.testing.assert_array_equal(frame.xyz, sweep.xyz)
    np.testing.assert_array_equal(frame.data["distance"], sweep.cols["distance"])
    np.testing.assert_array_equal(frame.data["t"], sweep.t.astype(np.float32))
    assert (frame.cones, frame.sensor_height, frame.seed, frame.idx) == (cones, 0.4, 7, 3)


# --- ATX spec --------------------------------------------------------------

def test_atx_spec_is_derived_from_the_emitter_config(tmp_path):
    f = tmp_path / "emitter.json"
    f.write_text(json.dumps({"scan": {"azimuthWindowDeg": [-60.0, 60.0], "scanRateBaseHz": 10.0}}))
    spec = load_sweep_spec(f)
    assert spec.azimuth_window == (-60.0, 60.0)
    assert spec.gap_s == pytest.approx(0.04)  # the value validate_atx.py used to hardcode


def test_real_atx_emitter_config_loads():
    path = resolve_path(SensorConfig().emitter_config_path)
    assert path.is_file(), f"{path} not found (repo root {REPO_ROOT})"
    spec = load_sweep_spec(path)
    assert spec.azimuth_window == (-60.0, 60.0)
