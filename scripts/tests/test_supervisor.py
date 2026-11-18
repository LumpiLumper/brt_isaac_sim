import json
from datetime import datetime

import numpy as np

from config import parse_cli, parse_config
from runner.frames import frame_done, frame_stem, missing_frames
from runner.metadata import METADATA_FILE
from runner.supervisor import RUN_LOG_FILE, supervise, worker_args
from utils.cone import Cone, ConeType
from utils.frame import DTYPE, Frame
from output_formatter.save_frame import save_frame
from scene.cone_assets import ConeSizes

NOW = datetime(2026, 11, 18, 14, 30, 5)
PREFIX = ["python", "worker.py"]
SIZES = {t: (0.2, 0.2, 0.3) for t in ConeType}


def cfg_for(tmp_path, *extra):
    return parse_config(["--out-root", str(tmp_path / "Outputs"), *extra])


def write_frame(run_dir, idx):
    (run_dir / f"{frame_stem(idx)}.pcd").write_bytes(b"x")
    (run_dir / f"{frame_stem(idx)}.json").write_text("{}")


class FakeWorker:
    """Stands in for the Isaac worker: handles the frames it is told to, but dies after `per_run` of them."""

    def __init__(self, per_run, deaths=None):
        self.per_run, self.deaths, self.calls = per_run, deaths, []

    def __call__(self, cmd):
        assert cmd[:2] == PREFIX
        cfg, is_worker = parse_cli(cmd[2:])
        assert is_worker
        self.calls.append(cfg)
        made = 0
        for idx in range(cfg.start_frame, cfg.start_frame + cfg.frames):
            if frame_done(cfg.run_dir, idx):
                continue
            if made == self.per_run:
                return -11  # segfault
            write_frame(cfg.run_dir, idx)
            made += 1
        return 0


def test_worker_that_never_dies_needs_one_run(tmp_path):
    worker = FakeWorker(per_run=100)
    res = supervise(cfg_for(tmp_path, "--frames", "5"), PREFIX, worker, now=NOW, log=lambda m: None)
    assert res.ok and len(res.workers) == 1 and res.restarts == 0


def test_dying_worker_is_restarted_until_all_frames_exist(tmp_path):
    worker = FakeWorker(per_run=3)
    res = supervise(cfg_for(tmp_path, "--frames", "10"), PREFIX, worker, now=NOW, log=lambda m: None)

    assert res.ok and res.missing == []
    assert [w.produced for w in res.workers] == [3, 3, 3, 1]
    assert res.restarts == 3
    assert [c.start_frame for c in worker.calls] == [0, 3, 6, 9]  # each restart resumes at the first missing frame


def test_all_workers_share_one_run_folder_and_metadata(tmp_path):
    worker = FakeWorker(per_run=2)
    res = supervise(cfg_for(tmp_path, "--frames", "6"), PREFIX, worker, now=NOW, log=lambda m: None)

    assert {c.run_dir for c in worker.calls} == {res.run_dir}
    assert [p.name for p in (tmp_path / "Outputs").iterdir()] == [res.run_dir.name]
    assert (res.run_dir / METADATA_FILE).is_file()


def test_gives_up_after_idle_restarts(tmp_path):
    worker = FakeWorker(per_run=0)  # dies before producing anything
    logs = []
    res = supervise(cfg_for(tmp_path, "--frames", "4"), PREFIX, worker, max_idle_restarts=3, now=NOW, log=logs.append)

    assert not res.ok and res.missing == [0, 1, 2, 3]
    assert len(res.workers) == 3
    assert any("giving up" in m for m in logs)


def test_idle_counter_resets_after_progress(tmp_path):
    outcomes = iter([0, 0, 2, 0, 0, 2])  # frames produced by successive runs

    def flaky(cmd):
        cfg, _ = parse_cli(cmd[2:])
        todo = [i for i in range(cfg.start_frame, cfg.start_frame + cfg.frames) if not frame_done(cfg.run_dir, i)]
        for idx in todo[:next(outcomes)]:
            write_frame(cfg.run_dir, idx)
        return -11

    res = supervise(cfg_for(tmp_path, "--frames", "4"), PREFIX, flaky, max_idle_restarts=3, now=NOW, log=lambda m: None)
    assert res.ok  # two idle runs, then progress, twice: never three idle runs in a row


def test_existing_frames_are_skipped_on_resume(tmp_path):
    cfg = cfg_for(tmp_path, "--frames", "5", "--run-dir", str(tmp_path / "Outputs" / "earlier"))
    (tmp_path / "Outputs" / "earlier").mkdir(parents=True)
    write_frame(cfg.run_dir, 0)
    write_frame(cfg.run_dir, 1)

    worker = FakeWorker(per_run=100)
    res = supervise(cfg, PREFIX, worker, now=NOW, log=lambda m: None)
    assert res.ok and res.workers[0].produced == 3
    assert worker.calls[0].start_frame == 2


def test_half_written_frame_is_redone(tmp_path):
    cfg = cfg_for(tmp_path, "--frames", "2", "--run-dir", str(tmp_path / "run"))
    (tmp_path / "run").mkdir()
    (tmp_path / "run" / "frame_0000.pcd").write_bytes(b"x")  # json missing: the worker died mid-frame
    assert missing_frames(cfg.run_dir, cfg) == [0, 1]


def test_run_log_records_workers(tmp_path):
    res = supervise(cfg_for(tmp_path, "--frames", "5"), PREFIX, FakeWorker(per_run=2), now=NOW, log=lambda m: None)
    log = json.loads((res.run_dir / RUN_LOG_FILE).read_text())

    assert log["complete"] is True and log["restarts"] == 2 and log["missing"] == []
    assert [w["produced"] for w in log["workers"]] == [2, 2, 1]


def test_worker_args_rebuild_the_run_config(tmp_path):
    cfg = cfg_for(tmp_path, "--frames", "20", "--sensor-height", "0.9", "--seed", "4")
    res = supervise(cfg, PREFIX, FakeWorker(per_run=100), now=NOW, log=lambda m: None)

    replayed, is_worker = parse_cli(worker_args(res.run_dir, 7, 5))
    assert is_worker and replayed.run_dir == res.run_dir
    assert (replayed.start_frame, replayed.frames) == (7, 5)
    assert (replayed.sensor.height, replayed.seed) == (0.9, 4)


# --- save_frame is atomic ------------------------------------------------------

def test_save_frame_leaves_no_temp_files(tmp_path):
    frame = Frame(np.zeros(2, dtype=DTYPE), [Cone(ConeType.SMALL_BLUE, 1.0, 2.0, 0.0)])
    save_frame(frame, tmp_path / "frame_0000", SIZES)
    assert sorted(p.name for p in tmp_path.iterdir()) == ["frame_0000.json", "frame_0000.pcd"]


def test_failed_write_does_not_mark_the_frame_done(tmp_path, monkeypatch):
    import output_formatter.save_frame as module
    monkeypatch.setattr(module, "save_labels", lambda path, frame, cone_sizes: (_ for _ in ()).throw(RuntimeError("boom")))
    frame = Frame(np.zeros(2, dtype=DTYPE), [])
    try:
        module.save_frame(frame, tmp_path / "frame_0000", SIZES)
    except RuntimeError:
        pass
    assert not frame_done(tmp_path, 0)  # pcd exists, json does not
    assert not (tmp_path / "frame_0000.json").exists()
