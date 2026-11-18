# Report notes: ATX lidar simulation and sweep assembly

Findings from the design discussion, structured for use in the report. Each claim is marked:
**[verified]** checked against the datasheet, the docs or the code, **[observed]** seen in our own runs (per the
README and script comments, not re-measured here), **[hypothesis]** plausible but not tested.

## 1. The real sensor (Hesai ATX)

Source: `assets/ATX_LiDAR/ATX_User_Manual_A07-en-260520.pdf`, section 1.4 and Appendix A.1. **[verified]**

| Property | Value |
|---|---|
| Scanning method | 1-D rotating mirror (hybrid solid-state); the housing does not spin |
| Channels | 116, unevenly distributed in elevation |
| FOV (H x V) | 120 deg x 18.3 deg (-12.4 to +5.9 deg) |
| Horizontal resolution | 0.08 deg at 10 Hz |
| Frame rate | 10 Hz |
| Range | 0.5 to 198 m instrumented, 3 cm precision (1 sigma) |
| Returns | single return (last/strongest) |
| Points per frame | 120 / 0.08 = 1500 per channel, x 116 = 174 000 |

Behaviours that matter for simulation:
- The mirror alternates direction between frames: even frames sweep increasing azimuth, odd frames decreasing
  (parity flag in the packet tail).
- The mirror sweeps the 120 deg over the whole 0.1 s frame.
- Each mirror surface covers slightly more than 120 deg.
- Azimuth is measured from the Y-axis (manual, Figure 3).

## 2. How the sensor is modelled in Isaac Sim

Isaac Sim 6.0.1 has no mirror model, so the ATX is built as a **rotary lidar with the output limited to a
120 deg azimuth window**. **[verified]** (`assets/ATX_LiDAR/make_atx_config.py`, `make_atx_usd.py`, README)

- 116 emitters at the datasheet elevations, ordered by descending elevation (RingID order).
- 4500 firing states per 360 deg revolution (0.08 deg each), 10 Hz, 45 000 Hz firing rate.
- Window -60..60 deg around the +X boresight, 1500 columns per channel.
- Per-emitter horizontal offset taken from the even-frame value of the manual.
- Ranges in 6 range groups from the per-channel table.
- 52 super-resolution channels have no range in the manual and borrow the range of the nearest normal channel.
- `omni:sensor:tickRate` equals `scanRateBaseHz` (10 Hz). **[verified]** (`make_atx_usd.py:109`)

### Where the model differs from the real sensor (limitations to state)

1. **Timing within a frame.** The simulated head sweeps 360 deg per 0.1 s, so the 120 deg window is covered in the
   first third of each frame (about 0.033 s). The real mirror takes the whole 0.1 s. Per-point timestamps (`t` in
   the PCD) are about three times too compressed. This matters only for motion deskewing or moving platforms; it does
   not matter for static cone scenes.
2. **No even/odd alternation.** The simulation only reproduces the even-frame sweep direction.
3. **Horizontal FOV** is exactly 120 deg, not slightly more.
4. **Borrowed ranges** for the 52 super-resolution channels.
5. **Azimuth convention.** The manual uses the Y-axis as 0 deg; the sensor prim uses +X as the boresight.
6. Grazing ground hits (1-4 deg below the horizon from 1.5 m) fall under the simulator's detection floor and are
   dropped. This is a property of the test scene, not of the sensor model. **[observed]** (README)

Point geometry (elevations, resolution, window, range, points per frame) matches the datasheet.

## 3. Why sweeps are assembled in our own code

### What Isaac Sim documents
Sources: Isaac Sim docs for `IsaacCreateRTXLidarScanBuffer` (5.1, 6.0.1) and the 6.0 ROS 2 migration guide.
**[verified]** (documentation only; behaviour not re-tested)

- The scan buffer is meant to return one full scan. With `omni:sensor:Core:accumulateOutputs` on (default), the lidar
  accumulates data over several frames until a scan is complete.
- For a rotary lidar a "full scan" is a **360 deg rotation**. The first scan can be partial if the first beam is not
  at the minimum azimuth.
- Accumulation only works when `omni:sensor:tickRate` equals `omni:sensor:Core:scanRateBaseHz`; otherwise partial
  scans are produced every frame.
- In 6.0 the ROS 2 RTX lidar helper's `fullScan` option is deprecated; publish rate follows `tickRate`. The
  scan-buffer extension itself is deprecated since 6.0.0 in favour of `isaacsim.sensors.experimental.rtx`.
- A forum report describes timestamp arrays about half the size of the point arrays from this annotator, with no
  confirmed answer on how to detect a full scan reliably.

### What we observed
**[observed]** (README, `validate_atx.py` and `make_cone_frames.py` comments)

- `get_data()` returns partial slices, one per render step (60 Hz). One ATX sweep arrives as two ~60 deg slices plus
  a few late points.
- The buffers hold garbage during a warm-up after the first `play()`; the first sweep after startup is about half full.
- Slices from the previous cone layout keep arriving after the cones are re-scattered.
- The 6.0.1 annotator crashes nondeterministically in `get_data()` after about 15-40 polls.

### Conclusion for the report
The documented setup is in place (tick rate matches the scan rate, accumulation at its default), yet the annotator
still returned partial slices. We could not find documentation that explains this. So we assemble each sweep
ourselves from the timestamped slices and check it for completeness before saving a frame.

Suggested wording: *"The Isaac Sim 6.0.1 scan buffer is documented to accumulate full scans, but with our 120 deg
ATX configuration it returned partial slices, a partly filled first sweep and stale data after scene changes, and
its `get_data()` crashed intermittently. We therefore assemble the sweeps from timestamped slices and verify each
one for completeness."* This describes what was observed and does not claim Isaac cannot do it.

### Hypotheses (untested)
1. The simulator treats the ATX as a 360 deg revolution of which only 1/3 produces output, so the scan buffer's
   "complete" logic (a whole revolution) never matches the ATX's 120 deg sweep. **[hypothesis]**
2. The behaviour is a bug in the deprecated 6.0.1 extension (the intermittent crash points that way). **[hypothesis]**

Quick checks that would settle it:
- Read `accumulateOutputs` from the ATX prim.
- Try the experimental extension `isaacsim.sensors.experimental.rtx` instead of the deprecated one.
- Run the ROS 2 lidar publisher on the ATX and inspect what it publishes.

If any of these gives clean 120 deg scans, the sweep assembly can shrink to a validation check.

## 4. How the sweep assembly works

Code: `scripts/lidar/sweep.py`, `scripts/lidar/atx.py`, tested in `scripts/tests/test_sweep.py`. **[verified]**

1. Each `get_data()` result is validated (non-empty, field sizes consistent, at least one valid point, azimuth inside
   the window plus 5 deg slack) and reduced to its valid points.
2. Repeated reads are dropped (key: point count and min/max timestamp).
3. All slices are sorted by timestamp and split into groups wherever the gap exceeds `gap_s`.
4. A group is accepted as a complete sweep when it reaches both window edges (within 0.5 deg) **and** every 10 deg
   azimuth bin holds at least 50 points. Min/max alone is not enough: a few late points can reach either edge of a
   half sweep.
5. The last group is never accepted, because it may still be filling. A sweep is therefore only reported once the
   next one has started.
6. Points are stacked into a structured array (x, y, z, intensity, distance, azimuth, elevation, t), with `t` in
   seconds from the sweep's first point.

Sensor-specific parameters come from `atx_emitter_config.json`: the azimuth window, and `gap_s = 1.2 x sweep length`
where the sweep length is 120/360 / 10 Hz = 0.033 s, giving 0.04 s.

Why each check exists: every one counters a failure seen in the data (half sweeps, stale layouts, garbage warm-up
reads). After each re-scatter the capture also waits `settle` render steps and discards earlier reads, so a frame
never mixes two cone layouts.

## 5. Related design decisions

- **Config:** one frozen `RunConfig` (sensor, placement, capture) built from defaults, then a TOML file or a previous
  run's `metadata.json`, then CLI flags. Unknown keys are errors.
- **Reproducibility:** each run gets `Outputs/<date>_<time>/` with `metadata.json` (config, argv, Python version, git
  commit and dirty flag). Placement is seeded per `(seed, frame index)`, so a replay produces the same layouts.
  Crash-restart workers reuse the run folder via `--run-dir`.
- **Placement algorithms** are pluggable through a registry (`@register("name")`); `uniform_annulus` is the only one
  so far.
- **Sensor modelling kept explicit** in the config (`model`, `usd_path`, `emitter_config_path`), so a run records
  which sensor it used.

## 6. Open items

- Fix the `SensorConfig.usd_path` default: it says `atx_lidar.usd`, the file is `ATX_LiDAR.usd`.
- Decide whether to run the three checks in section 3 before relying on the custom assembly.
- Cone box sizes come from two sources (`CONE_SIZES` and the USD bounding box in the old script); pick one.
- Per-frame JSON still lacks the sensor pose and seed that the old script wrote.
- Compare one real frame from the refactored pipeline with one from the current script (step 4 of the plan).

## Sources

- Hesai ATX User Manual A07 (`assets/ATX_LiDAR/ATX_User_Manual_A07-en-260520.pdf`), sections 1.4 and 3.1, Appendix A.1
- [Isaac Create RTX Lidar Scan Buffer, 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.rtx/docs/ogn/OgnIsaacCreateRTXLidarScanBuffer.html)
- [Isaac Create RTX Lidar Scan Buffer, 6.0.1](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/py/source/deprecated/isaacsim.sensors.rtx/docs/ogn/OgnIsaacCreateRTXLidarScanBuffer.html)
- [ROS 2 Sensor Graph Migration, 6.0](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/ros2_sensor_graph_migration.html)
- [ROS2 RTX Lidar Helper, 6.0.0](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.ros2.nodes/docs/ogn/OgnROS2RtxLidarHelper.html)
- [Forum: RTX Lidar Scan Buffer timestamp count is half of the point cloud size](https://forums.developer.nvidia.com/t/rtx-lidar-scan-buffer-timestamp-count-is-half-of-the-point-cloud-size/354993)
- Repo files: `assets/ATX_LiDAR/README.md`, `make_atx_config.py`, `make_atx_usd.py`, `validate_atx.py`, `make_cone_frames.py`
