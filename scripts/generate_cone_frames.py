"""Cone-field dataset: flat ground, random cones around the ATX sensor, one lidar sweep per layout, repeat.

    docker compose run --rm sim /isaac-sim/python.sh scripts/generate_cone_frames.py --frames 20 --cones 60

Same job as make_cone_frames.py, built from the modules in scripts/ (config, placement strategies, sweep assembly,
output formatter, supervisor). Every run gets its own Outputs/<date>_<time>/ with metadata.json (the full config, so
the run can be replayed with --config) and run_log.json, then frame_NNNN.pcd/.json per frame. Without --worker this
process is the supervisor: it starts the Isaac worker and restarts it at the first missing frame when it crashes
(annotator.get_data() segfaults at random in Isaac Sim 6.0.1).
"""

import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))  # the modules (config, runner, isaac, ...) sit next to this file

from config import parse_cli, resolve_path  # noqa: E402


def run_worker(cfg):
    if cfg.run_dir is None:
        sys.exit("--worker needs --run-dir; start the script without --worker to get a supervisor")
    os.umask(0)  # the container user (uid 1234) writes here: keep outputs deletable from the host account

    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"headless": True})

    import numpy as np
    import omni.usd
    from isaacsim.core.utils.stage import is_stage_loading, open_stage

    from isaac.sensor import LidarCapture
    from isaac.stage import SENSOR_PRIM, apply_ground_material, apply_layout, build_scene_file
    from lidar.atx import load_sweep_spec
    from lidar.sweep import to_frame
    from output_formatter.save_frame import save_frame
    from runner.frames import frame_done, frame_stem, wanted_frames
    from scene.cone_assets import measure_cone_sizes
    from scene.placement import get_strategy

    def wait_loaded():
        while is_stage_loading():
            simulation_app.update()

    run_dir = cfg.run_dir
    cones_dir = resolve_path(cfg.cones_dir)
    strategy = get_strategy(cfg.placement.algorithm)
    spec = load_sweep_spec(resolve_path(cfg.sensor.emitter_config_path))
    cone_sizes = measure_cone_sizes(cones_dir)

    scene_path = run_dir / "scene.usd"  # built as a file and opened, as validate_atx.py does with its scene
    build_scene_file(scene_path, cfg.sensor, cfg.placement.radius, resolve_path(cfg.sensor.usd_path))
    open_stage(str(scene_path))
    wait_loaded()
    stage = omni.usd.get_context().get_stage()
    apply_ground_material(stage, cfg.ground_base, cfg.ground_coating)  # on the open stage: see its docstring

    capture = LidarCapture(SENSOR_PRIM, spec, cfg.capture)
    capture.start()

    # The first sweep after startup came out with about half the points in testing, so burn one before saving anything.
    apply_layout(stage, strategy.place(np.random.default_rng([cfg.seed, 999999]), cfg.placement), cones_dir, wait_loaded)
    capture.capture_sweep()

    done = 0
    for idx in wanted_frames(cfg):
        if frame_done(run_dir, idx):
            continue
        cones = strategy.place(np.random.default_rng([cfg.seed, idx]), cfg.placement)  # (seed, idx): restarts repeat it
        apply_layout(stage, cones, cones_dir, wait_loaded)
        sweep = capture.capture_sweep()
        if sweep is None:
            print(f"[worker] frame {idx}: no complete sweep in {cfg.capture.max_steps} steps, skipped", flush=True)
            continue
        frame = to_frame(sweep, cones, cfg.sensor.height, seed=cfg.seed, idx=idx)
        save_frame(frame, run_dir / frame_stem(idx), cone_sizes)
        done += 1
        print(f"[worker] frame {idx}: {len(cones)} cones, {len(frame.data)} points", flush=True)

    capture.stop()
    print(f"[worker] wrote {done} frames", flush=True)
    simulation_app.close()


def main():
    cfg, is_worker = parse_cli()
    if is_worker:
        run_worker(cfg)
        return
    from runner.supervisor import supervise
    result = supervise(cfg, [sys.executable, __file__], argv=sys.argv)
    sys.exit(0 if result.ok else 1)


if __name__ == "__main__":
    main()
