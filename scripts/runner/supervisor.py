"""Run the capture worker in a child process and restart it whenever it dies.

Isaac Sim 6.0.1's annotator.get_data() segfaults now and then, which no try/except can catch. The supervisor owns the
run folder, starts a worker at the first missing frame, and starts another when it exits with frames still missing.
Each frame's layout is seeded by (seed, frame index), so a restart reproduces what an uninterrupted run would make.
"""

import json
import subprocess
import time
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Callable

from config import RunConfig
from runner.frames import missing_frames
from runner.metadata import METADATA_FILE, make_run_dir, write_metadata

RUN_LOG_FILE = "run_log.json"
MAX_IDLE_RESTARTS = 5  # worker runs in a row that produced no frame before giving up


@dataclass
class WorkerRun:
    first_frame: int
    produced: int
    seconds: float
    exit_code: int


@dataclass
class SupervisorResult:
    run_dir: Path
    workers: list[WorkerRun] = field(default_factory=list)
    missing: list[int] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.missing

    @property
    def restarts(self) -> int:
        return max(len(self.workers) - 1, 0)


def worker_args(run_dir: Path, first: int, count: int) -> list[str]:
    """Arguments that make a child rebuild the exact run config and handle frames first .. first+count-1."""
    return ["--worker", "--config", str(run_dir / METADATA_FILE), "--run-dir", str(run_dir),
            "--start-frame", str(first), "--frames", str(count)]


def subprocess_worker(cmd: list[str]) -> int:
    return subprocess.run(cmd).returncode


def supervise(cfg: RunConfig, command: list[str], run_worker: Callable[[list[str]], int] = subprocess_worker,
              max_idle_restarts: int = MAX_IDLE_RESTARTS, argv: list[str] | None = None,
              now: datetime | None = None, log: Callable[[str], None] = print) -> SupervisorResult:
    """command is the worker prefix, e.g. [sys.executable, "make_cone_frames.py"]; run_worker is injectable for tests."""
    run_dir = make_run_dir(cfg, now=now)
    write_metadata(run_dir, cfg, argv=argv, now=now)
    result = SupervisorResult(run_dir)
    idle = 0
    while True:
        todo = missing_frames(run_dir, cfg)
        if not todo:
            break
        if idle >= max_idle_restarts:
            log(f"[supervisor] giving up: {idle} worker runs in a row produced no frame; {len(todo)} frames missing")
            break
        started = time.monotonic()
        code = run_worker(command + worker_args(run_dir, todo[0], todo[-1] - todo[0] + 1))
        left = missing_frames(run_dir, cfg)
        run = WorkerRun(todo[0], len(todo) - len(left), time.monotonic() - started, code)
        result.workers.append(run)
        idle = idle + 1 if run.produced == 0 else 0
        if left:
            log(f"[supervisor] worker stopped (exit {code}) after {run.produced} frames, "
                f"restarting at frame {left[0]} ({len(left)} left)")
    result.missing = missing_frames(run_dir, cfg)
    (run_dir / RUN_LOG_FILE).write_text(json.dumps({
        "complete": result.ok, "missing": result.missing, "restarts": result.restarts,
        "workers": [vars(w) for w in result.workers]}, indent=1))
    if result.ok:
        log(f"[supervisor] all {cfg.frames} frames present in {run_dir}")
    return result
