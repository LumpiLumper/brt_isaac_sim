import json

import numpy as np
import pytest

from utils.cone import Cone, ConeType
from utils.frame import DTYPE, Frame
from save_as_pcd import write_pcd
from save_labels import CONE_LABELS, CONE_SIZES, make_label, save_labels
from save_frame import save_frame


def make_data(n=3):
    data = np.zeros(n, dtype=DTYPE)
    for i, name in enumerate(DTYPE.names):
        data[name] = np.arange(n) + i * 10
    return data


def split_pcd(raw: bytes):
    header, _, body = raw.partition(b"DATA binary\n")
    return header.decode().splitlines(), body


# --- write_pcd -------------------------------------------------------------

def test_pcd_header(tmp_path):
    out = tmp_path / "a.pcd"
    write_pcd(out, Frame(make_data(3), sensor_height=1.5))
    lines, _ = split_pcd(out.read_bytes())
    k = len(DTYPE.names)

    assert lines[0] == "# .PCD v0.7"
    assert "FIELDS " + " ".join(DTYPE.names) in lines
    assert "SIZE " + " ".join(["4"] * k) in lines
    assert "TYPE " + " ".join(["F"] * k) in lines
    assert "COUNT " + " ".join(["1"] * k) in lines
    assert "WIDTH 3" in lines
    assert "HEIGHT 1" in lines
    assert "POINTS 3" in lines
    assert "VIEWPOINT 0 0 1.5 1 0 0 0" in lines


def test_pcd_body_round_trips(tmp_path):
    data = make_data(5)
    out = tmp_path / "a.pcd"
    write_pcd(out, Frame(data))
    _, body = split_pcd(out.read_bytes())

    assert len(body) == 5 * DTYPE.itemsize
    np.testing.assert_array_equal(np.frombuffer(body, dtype=DTYPE), data)


def test_pcd_empty_frame(tmp_path):
    out = tmp_path / "a.pcd"
    write_pcd(out, Frame(np.zeros(0, dtype=DTYPE)))
    lines, body = split_pcd(out.read_bytes())

    assert "POINTS 0" in lines
    assert body == b""


def test_pcd_casts_other_dtypes_to_float32(tmp_path):
    data = make_data(2).astype([(k, "<f8") for k in DTYPE.names])
    out = tmp_path / "a.pcd"
    write_pcd(out, Frame(data))
    _, body = split_pcd(out.read_bytes())

    assert len(body) == 2 * DTYPE.itemsize


# --- make_label / save_labels ---------------------------------------------

def test_make_label_centers_box_on_cone_height():
    label = make_label("Small_Cone_Blue", (1, 2, 0.5))
    h = CONE_SIZES["Small_Cone_Blue"][2]

    assert label["type"] == "Small_Cone_Blue"
    assert label["center"] == pytest.approx([1, 2, 0.5 + h / 2])
    assert label["size"] == list(CONE_SIZES["Small_Cone_Blue"])
    assert label["yaw_deg"] == 0.0


def test_make_label_accepts_numpy_xyz_and_returns_plain_floats():
    label = make_label("Big_Orange_Cone", np.array([1.0, 2.0, 3.0], dtype=np.float32))
    json.dumps(label)  # numpy scalars would raise TypeError
    assert all(type(v) is float for v in label["center"])


def test_make_label_unknown_type():
    with pytest.raises(KeyError, match="unknown cone type"):
        make_label("Purple_Cone", (0, 0, 0))


def test_save_labels_writes_json(tmp_path):
    out = tmp_path / "labels.json"
    boxes = save_labels(out, [Cone(ConeType.SMALL_BLUE, 0, 0, 0), Cone(ConeType.BIG_ORANGE, 1, 1, 0)])
    doc = json.loads(out.read_text())

    assert doc["boxes"] == boxes
    assert [b["type"] for b in doc["boxes"]] == ["Small_Cone_Blue", "Big_Orange_Cone"]
    assert "frame" in doc


def test_save_labels_no_cones(tmp_path):
    out = tmp_path / "labels.json"
    save_labels(out, [])
    assert json.loads(out.read_text())["boxes"] == []


# --- save_frame ------------------------------------------------------------

def test_save_frame_writes_pcd_and_labels_to_separate_files(tmp_path):
    frame = Frame(make_data(2), [Cone(ConeType.SMALL_BLUE, 1.0, 2.0, 0.0)])
    save_frame(frame, tmp_path / "scan")

    assert {p.name for p in tmp_path.iterdir()} == {"scan.pcd", "scan.json"}
    boxes = json.loads((tmp_path / "scan.json").read_text())["boxes"]
    assert [b["type"] for b in boxes] == ["Small_Cone_Blue"]


def test_every_cone_type_has_label_and_size():
    for cone_type in ConeType:
        assert CONE_LABELS[cone_type] in CONE_SIZES
