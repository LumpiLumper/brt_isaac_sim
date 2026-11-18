import json
from dataclasses import replace
from datetime import datetime

from config import parse_config, to_dict
from runner.metadata import METADATA_FILE, make_run_dir, write_metadata
from utils.cone import ConeType

NOW = datetime(2026, 11, 18, 14, 30, 5)


def cfg_for(tmp_path, *extra):
    return parse_config(["--out-root", str(tmp_path / "Outputs"), *extra])


def test_run_dir_is_named_after_date_and_time(tmp_path):
    run_dir = make_run_dir(cfg_for(tmp_path), now=NOW)
    assert run_dir == tmp_path / "Outputs" / "2026-11-18_14-30-05"
    assert run_dir.is_dir()


def test_runs_started_in_the_same_second_get_separate_folders(tmp_path):
    cfg = cfg_for(tmp_path)
    a, b = make_run_dir(cfg, now=NOW), make_run_dir(cfg, now=NOW)
    assert a != b and a.is_dir() and b.is_dir()


def test_explicit_run_dir_is_reused(tmp_path):
    existing = tmp_path / "Outputs" / "earlier"
    cfg = cfg_for(tmp_path, "--run-dir", str(existing))
    assert make_run_dir(cfg) == existing
    assert make_run_dir(cfg) == existing  # resuming twice must not fail or create siblings
    assert [p.name for p in existing.parent.iterdir()] == ["earlier"]


def test_metadata_contents(tmp_path):
    cfg = cfg_for(tmp_path, "--sensor-height", "0.9", "--seed", "4")
    path = write_metadata(make_run_dir(cfg, now=NOW), cfg, argv=["run.py", "--seed", "4"], now=NOW)
    meta = json.loads(path.read_text())

    assert path.name == METADATA_FILE
    assert meta["created"] == "2026-11-18T14:30:05"
    assert meta["argv"] == ["run.py", "--seed", "4"]
    assert meta["config"]["sensor"]["height"] == 0.9
    assert meta["config"]["seed"] == 4
    assert "run_dir" not in meta["config"]
    assert "commit" in (meta["git"] or {"commit": None})


def test_metadata_reproduces_the_config(tmp_path):
    cfg = cfg_for(tmp_path, "--sensor-height", "0.9", "--cones", "12", "--placement-param", "rows=3")
    cfg = replace(cfg, placement=replace(cfg.placement, cone_types=(ConeType.SMALL_BLUE, ConeType.BIG_ORANGE)))
    path = write_metadata(make_run_dir(cfg, now=NOW), cfg)

    replayed = parse_config(["--config", str(path)])
    assert replayed == cfg  # same settings, and run_dir is unset, so a replay starts a new folder
    assert make_run_dir(replayed, now=datetime(2026, 12, 1)) != path.parent


def test_resumed_run_keeps_original_metadata(tmp_path):
    cfg = cfg_for(tmp_path)
    run_dir = make_run_dir(cfg, now=NOW)
    first = write_metadata(run_dir, cfg, argv=["first"], now=NOW).read_text()
    write_metadata(run_dir, cfg, argv=["second"], now=datetime(2027, 1, 1))
    assert (run_dir / METADATA_FILE).read_text() == first


def test_to_dict_is_plain_json():
    json.dumps(to_dict(parse_config([])))
