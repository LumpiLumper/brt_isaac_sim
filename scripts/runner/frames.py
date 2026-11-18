"""Which frame files a run folder holds. save_frame() writes the json last, so it marks a finished frame."""

from pathlib import Path

from config import RunConfig


def frame_stem(idx: int) -> str:
    return f"frame_{idx:04d}"


def frame_done(run_dir: Path, idx: int) -> bool:
    return all((run_dir / f"{frame_stem(idx)}{ext}").exists() for ext in (".pcd", ".json"))


def wanted_frames(cfg: RunConfig) -> range:
    return range(cfg.start_frame, cfg.start_frame + cfg.frames)


def missing_frames(run_dir: Path, cfg: RunConfig) -> list[int]:
    return [i for i in wanted_frames(cfg) if not frame_done(run_dir, i)]
