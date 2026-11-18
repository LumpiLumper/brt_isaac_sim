"""Per-run output folder and the metadata file that lets a run be reproduced."""

import json
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from config import RunConfig, to_dict

METADATA_FILE = "metadata.json"
TIMESTAMP_FORMAT = "%Y-%m-%d_%H-%M-%S"


def make_run_dir(cfg: RunConfig, now: datetime | None = None) -> Path:
    """The folder this run writes into: cfg.run_dir when resuming, else a new <out_root>/<date_time> one."""
    if cfg.run_dir is not None:
        cfg.run_dir.mkdir(parents=True, exist_ok=True)
        return cfg.run_dir
    stamp = (now or datetime.now()).strftime(TIMESTAMP_FORMAT)
    cfg.out_root.mkdir(parents=True, exist_ok=True)
    for n in range(100):  # two runs started within the same second must not share a folder
        run_dir = cfg.out_root / (stamp if n == 0 else f"{stamp}_{n}")
        try:
            run_dir.mkdir()
            return run_dir
        except FileExistsError:
            continue
    raise RuntimeError(f"could not create a unique run folder for {stamp} in {cfg.out_root}")


def _git_state(cwd: Path) -> dict | None:
    try:
        run = lambda *a: subprocess.run(["git", *a], cwd=cwd, capture_output=True, text=True, check=True).stdout.strip()
        return {"commit": run("rev-parse", "HEAD"), "dirty": bool(run("status", "--porcelain"))}
    except (OSError, subprocess.CalledProcessError):
        return None


def write_metadata(run_dir: Path, cfg: RunConfig, argv: list[str] | None = None, now: datetime | None = None) -> Path:
    """Write metadata.json once per run; a resumed run keeps the original file."""
    path = run_dir / METADATA_FILE
    if path.exists():
        return path
    meta = {
        "created": (now or datetime.now()).isoformat(timespec="seconds"),
        "argv": sys.argv if argv is None else argv,
        "python": sys.version.split()[0],
        "git": _git_state(Path(__file__).resolve().parent),
        "config": to_dict(cfg),
    }
    path.write_text(json.dumps(meta, indent=1))
    return path
