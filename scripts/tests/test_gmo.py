import numpy as np
import pytest

from lidar.gmo import gmo_to_raw
from lidar.sweep import SweepAssembler, SweepSpec
from utils.frame import DTYPE

SPEC = SweepSpec(azimuth_window=(-60.0, 60.0), gap_s=0.04)


def one(az, el, r, intensity=0.5, toff=0, ts=0):
    return gmo_to_raw([az], [el], [r], [intensity], [toff], ts)


def test_spherical_to_cartesian_axes():
    assert one(0, 0, 5)["data"][0] == pytest.approx([5, 0, 0], abs=1e-5)     # boresight is +x
    assert one(90, 0, 5)["data"][0] == pytest.approx([0, 5, 0], abs=1e-5)    # positive azimuth is to the left (+y)
    assert one(0, 30, 5)["data"][0] == pytest.approx([5 * np.cos(np.radians(30)), 0, 2.5], abs=1e-5)  # up is +z
    assert one(-60, -9, 10)["data"][0] == pytest.approx(
        [10 * np.cos(np.radians(9)) * 0.5, -10 * np.cos(np.radians(9)) * np.sin(np.radians(60)), -10 * np.sin(np.radians(9))], abs=1e-5)


def test_matches_the_pcd_convention_of_the_old_annotator():
    """xyz = d * (cos el cos az, cos el sin az, sin el), as verified on a PCD written by the old pipeline."""
    rng = np.random.default_rng(0)
    az, el, d = rng.uniform(-60, 60, 500), rng.uniform(-9, 0.1, 500), rng.uniform(2, 50, 500)
    raw = gmo_to_raw(az, el, d, np.ones(500), np.zeros(500), 0)
    a, e = np.radians(az), np.radians(el)
    expected = np.column_stack([d * np.cos(e) * np.cos(a), d * np.cos(e) * np.sin(a), d * np.sin(e)])
    np.testing.assert_allclose(raw["data"], expected, atol=1e-4)


def test_info_fields_and_dtypes():
    raw = gmo_to_raw([10.0, 20.0], [-1.0, -2.0], [5.0, 6.0], [0.1, 0.2], [100, 200], 1_000_000)
    info = raw["info"]

    assert raw["data"].shape == (2, 3) and raw["data"].dtype == np.float32
    np.testing.assert_array_equal(info["distance"], np.float32([5.0, 6.0]))
    np.testing.assert_array_equal(info["azimuth"], np.float32([10.0, 20.0]))
    np.testing.assert_array_equal(info["elevation"], np.float32([-1.0, -2.0]))
    np.testing.assert_array_equal(info["intensity"], np.float32([0.1, 0.2]))
    assert info["timestamp"].dtype == np.uint64
    np.testing.assert_array_equal(info["timestamp"], [1_000_100, 1_000_200])  # write timestamp + per-point offset


def test_empty_write_is_none():
    assert gmo_to_raw([], [], [], [], [], 0) is None


def test_inputs_are_copied():
    az = np.array([10.0, 20.0])
    raw = gmo_to_raw(az, [0.0, 0.0], [5.0, 5.0], [1.0, 1.0], [0, 0], 0)
    az[:] = 999.0  # the writer's buffer is only valid inside the callback
    np.testing.assert_array_equal(raw["info"]["azimuth"], np.float32([10.0, 20.0]))


# --- the writes of a real scan, as seen in the probe: lead-in, two big slices, tail ----------------------------

FULL_STEP = (14_000, 16_665_000)     # point times inside a 16.67 ms render step, as seen in the probe
START_OF_STEP = (14_000, 815_000)    # the tail of a scan sits at the start of its step ...
END_OF_STEP = (16_548_000, 16_663_000)  # ... and the lead-in of the next scan at the end of its step


def write(ts_ns, az_lo, az_hi, n, toff=FULL_STEP):
    """One render step: n points spread over an azimuth range and over a span of the step's 16.67 ms."""
    az = np.linspace(az_lo, az_hi, n)
    return gmo_to_raw(az, np.full(n, -3.0), np.full(n, 10.0), np.full(n, 0.01), np.linspace(*toff, n), ts_ns)


def test_writes_of_one_scan_assemble_into_a_sweep():
    step = 16_666_667
    scan = [write(0 * step, 59.5, 60.0, 40, END_OF_STEP),  # lead-in: a few columns at the edge
            write(1 * step, 0.0, 60.0, 22_000),       # first half of the window
            write(2 * step, -60.0, 0.0, 22_000),      # second half
            write(3 * step, -60.0, -57.0, 300, START_OF_STEP)]  # tail
    quiet = [None, None]                              # steps where the lidar returns nothing
    next_scan_lead_in = write(6 * step, 59.5, 60.0, 40, END_OF_STEP)

    asm = SweepAssembler(SPEC)
    sweeps = [asm.add(r) for r in scan + quiet + [next_scan_lead_in]]
    sweep = next(s for s in sweeps if s is not None)

    assert len(sweep.xyz) == 40 + 22_000 + 22_000 + 300
    assert sweep.t.min() == 0.0 and 0.03 < sweep.t.max() < 0.04   # ~34 ms, as with the old annotator
    assert sweeps[:5] == [None] * 5                               # only reported once the next scan has started


def test_warmup_garbage_is_rejected():
    garbage = gmo_to_raw(np.linspace(-180, 180, 1000), np.zeros(1000), np.full(1000, 5.0), np.ones(1000), np.zeros(1000), 0)
    assert SweepAssembler(SPEC).add(garbage) is None


def test_frame_dtype_is_unchanged():
    assert DTYPE.names == ("x", "y", "z", "intensity", "distance", "azimuth", "elevation", "t")
