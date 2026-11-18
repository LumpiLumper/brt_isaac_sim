"""Run configuration: defaults < --config TOML file < command-line flags."""

import argparse
import dataclasses
import json
import tomllib
from dataclasses import dataclass, field
from pathlib import Path

from utils.cone import ConeType

REPO_ROOT = Path(__file__).resolve().parent.parent


def resolve_path(p: Path) -> Path:
    """Config paths are repo-relative so saved configs replay on another machine; absolute paths pass through."""
    return p if p.is_absolute() else REPO_ROOT / p


@dataclass(frozen=True)
class SensorConfig:
    model: str = "ATX"
    usd_path: Path = Path("assets/ATX_LiDAR/ATX_LiDAR.usd")
    emitter_config_path: Path = Path("assets/ATX_LiDAR/atx_emitter_config.json")
    height: float = 0.4
    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0

    def __post_init__(self):
        if self.height <= 0:
            raise ValueError(f"sensor height must be positive, got {self.height}")


@dataclass(frozen=True)
class PlacementConfig:
    algorithm: str = "uniform_annulus"
    num_cones: int = 50
    radius: float = 50.0
    min_radius: float = 2.0
    min_gap: float = 0.5
    cone_types: tuple[ConeType, ...] = tuple(ConeType)
    params: dict = field(default_factory=dict)  # algorithm-specific extras

    def __post_init__(self):
        if self.num_cones < 0:
            raise ValueError(f"num_cones must be >= 0, got {self.num_cones}")
        if not 0 <= self.min_radius < self.radius:
            raise ValueError(f"need 0 <= min_radius < radius, got {self.min_radius} and {self.radius}")
        if self.min_gap < 0:
            raise ValueError(f"min_gap must be >= 0, got {self.min_gap}")
        if not self.cone_types:
            raise ValueError("cone_types must not be empty")


@dataclass(frozen=True)
class CaptureConfig:
    settle: int = 6       # render steps to wait after moving cones before reading
    max_steps: int = 120  # render steps to wait for a complete sweep per frame
    warmup_steps: int = 9  # render steps after the first play(), while the pipeline fills and buffers hold garbage


@dataclass(frozen=True)
class RunConfig:
    sensor: SensorConfig = field(default_factory=SensorConfig)
    placement: PlacementConfig = field(default_factory=PlacementConfig)
    capture: CaptureConfig = field(default_factory=CaptureConfig)
    frames: int = 10
    seed: int = 0
    start_frame: int = 0
    out_root: Path = Path("Outputs")  # each run gets its own timestamped folder in here
    run_dir: Path | None = None
    cones_dir: Path = Path("assets/cones")  # the cone usd files (repo-relative, see resolve_path)


def to_dict(cfg: RunConfig) -> dict:
    """JSON-serializable form of the config that from_dict() reads back. run_dir is left out: it belongs to
    one particular run, so replaying a saved config starts a new folder."""
    def plain(v):
        if isinstance(v, Path):
            return str(v)
        if isinstance(v, ConeType):
            return v.name
        if isinstance(v, dict):
            return {k: plain(x) for k, x in v.items()}
        if isinstance(v, (tuple, list)):
            return [plain(x) for x in v]
        return v

    d = plain(dataclasses.asdict(cfg))
    d.pop("run_dir")
    return d


def _section(cls, values: dict):
    """Build a config dataclass from a dict, rejecting unknown keys and converting special types."""
    values = dict(values)
    known = {f.name for f in dataclasses.fields(cls)}
    unknown = set(values) - known
    if unknown:
        raise ValueError(f"unknown {cls.__name__} keys: {sorted(unknown)}; known: {sorted(known)}")
    for f in dataclasses.fields(cls):
        if f.type is Path and f.name in values:
            values[f.name] = Path(values[f.name])
    if "cone_types" in values:
        values["cone_types"] = tuple(ConeType[n] if isinstance(n, str) else n for n in values["cone_types"])
    return cls(**values)


def from_dict(d: dict) -> RunConfig:
    d = dict(d)
    sections = {"sensor": SensorConfig, "placement": PlacementConfig, "capture": CaptureConfig}
    kwargs = {name: _section(cls, d.pop(name, {})) for name, cls in sections.items()}
    for key in ("out_root", "run_dir", "cones_dir"):
        if d.get(key) is not None:
            d[key] = Path(d[key])
    top = {f.name for f in dataclasses.fields(RunConfig)} - set(sections)
    unknown = set(d) - top
    if unknown:
        raise ValueError(f"unknown config keys: {sorted(unknown)}; known: {sorted(top | set(sections))}")
    return RunConfig(**kwargs, **d)


def _parse_param(item: str):
    key, sep, raw = item.partition("=")
    if not sep:
        raise argparse.ArgumentTypeError(f"expected KEY=VALUE, got {item!r}")
    for cast in (int, float):
        try:
            return key, cast(raw)
        except ValueError:
            pass
    return key, raw


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser()
    p.add_argument("--config", type=Path,
                   help="TOML file, or the metadata.json of an earlier run to reproduce it; command-line flags override it")
    p.add_argument("--frames", type=int)
    p.add_argument("--seed", type=int)
    p.add_argument("--start-frame", type=int)
    p.add_argument("--out-root", type=Path, help="folder that holds the timestamped run folders")
    p.add_argument("--run-dir", type=Path, help="reuse this run folder (resume) instead of creating a new one")
    p.add_argument("--cones-dir", type=Path, help="folder with the cone usd files")
    p.add_argument("--sensor-height", type=float)
    p.add_argument("--placement", help="placement algorithm name")
    p.add_argument("--cones", type=int, help="cones per frame")
    p.add_argument("--radius", type=float)
    p.add_argument("--min-radius", type=float)
    p.add_argument("--min-gap", type=float)
    p.add_argument("--placement-param", action="append", default=[], type=_parse_param, metavar="KEY=VALUE")
    p.add_argument("--settle", type=int)
    p.add_argument("--max-steps", type=int)
    p.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)  # set by the supervisor on its children
    return p


# CLI flag (argparse dest) -> (section, config field); section None means top level.
_FLAGS = {
    "frames": (None, "frames"), "seed": (None, "seed"), "start_frame": (None, "start_frame"),
    "out_root": (None, "out_root"), "run_dir": (None, "run_dir"), "cones_dir": (None, "cones_dir"),
    "sensor_height": ("sensor", "height"),
    "placement": ("placement", "algorithm"), "cones": ("placement", "num_cones"),
    "radius": ("placement", "radius"), "min_radius": ("placement", "min_radius"),
    "min_gap": ("placement", "min_gap"),
    "settle": ("capture", "settle"), "max_steps": ("capture", "max_steps"),
}


def load_config_file(path: Path) -> dict:
    if path.suffix == ".json":
        d = json.loads(path.read_text())
        return d.get("config", d)  # metadata.json nests the config; a bare config dict also works
    return tomllib.loads(path.read_text())


def parse_config(argv: list[str] | None = None) -> RunConfig:
    return parse_cli(argv)[0]


def parse_cli(argv: list[str] | None = None) -> tuple[RunConfig, bool]:
    """The config plus whether this process is a supervisor-launched worker."""
    ns = build_parser().parse_args(argv)
    d = load_config_file(ns.config) if ns.config else {}
    for dest, (section, name) in _FLAGS.items():
        value = getattr(ns, dest)
        if value is None:
            continue
        target = d if section is None else d.setdefault(section, {})
        target[name] = value
    if ns.placement_param:
        d.setdefault("placement", {}).setdefault("params", {}).update(dict(ns.placement_param))
    return from_dict(d), ns.worker
