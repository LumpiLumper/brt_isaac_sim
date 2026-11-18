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

## 6. The `get_data()` crash and how it was fixed

**Symptom.** With the first version of the capture (polling `annotator.get_data()` of `IsaacCreateRTXLidarScanBuffer`
after each `app.update()`), the Isaac worker segfaulted (exit 139) after a handful of frames. In a 100-frame run the
supervisor needed 17 workers for 20 frames (frames per worker: 1, 0, 0, 5, 0, 3, 0, 2, 2, 0, 6, 1, 0, 0, 0, 0, 0) and
gave up after 5 workers in a row that produced nothing. The README's "crashes after about 15-40 frames" was far too
optimistic. **[observed]**

**Measured baseline.** 32 single workers, capped at 40 frames each: every one crashed, 142 frames in total, about
4.4 frames per worker. Healthy capture itself is fast (about 0.27 s per frame); start-up and crashes dominated the
time. **[observed]**

**Diagnosis.**
1. The crash backtrace (breakpad) always showed a numpy `array.copy()` called from Python, but its frame names are
   marked low-confidence and we call `.copy()` nowhere in our code.
2. `faulthandler` did not help: Kit's crash handler takes the signal first.
3. A line tracer (`sys.settrace` on our code and Isaac's annotator modules, writing the last executed line into an
   `mmap` file, which survives a segfault) gave the same answer in 8 of 8 crashes:
   `omni/replicator/core/scripts/utils/annotator_utils.py:519`, in `_get_data_from_ptr`, which is
   `return array.copy(), False`. **[observed]**
4. Reading that code: Isaac wraps the lidar node's raw output pointer in a zero-copy array
   (`_reshape_output_ptr`) and then copies it. Pointer and element count are read separately, so when the node swaps
   its buffer in between, the copy reads freed memory. This race is the most likely reading of the evidence; the
   race itself was not observed directly. **[hypothesis, consistent with all 8 traces]**

**What did not help** (8 interleaved workers per setting, same benchmark):

| Setting | Mean frames before crash |
|---|---|
| baseline | 4.4 (32 workers) |
| `enable_motion_bvh` on (Isaac logs "multi-tick is enabled but motion BVH is not active. This is not supported.") | 2.4 |
| `get_data(do_array_copy=True)` | 5.5, i.e. no effect |

`do_array_copy` could not matter: in `AnnotatorCache.get_data` the effective `do_copy` is `kwargs.get("do_copy") or
params.get("do_array_copy")`, and the annotator-level default is already `True`, so the failing copy ran either way.

**Fix.** Do not poll the annotator. Attach a Replicator **writer** with the `GenericModelOutput` annotator through
the experimental API (`isaacsim.sensors.experimental.rtx`: `Lidar` wraps the existing ATX OmniLidar prim,
`LidarSensor.attach_writer`) and copy what is needed *inside* the writer's `write()` callback, where the buffers are
guaranteed valid. Isaac's own deprecation warning for the scan buffer says the same: "Use the GenericModelOutput
annotator directly to access full-scan RTX Lidar data." Code: `scripts/isaac/sensor.py`, `scripts/lidar/gmo.py`.

**Evidence for the fix.** Probe (our ATX scene, one worker, 600 render steps of about 100 scans): 5 of 5 workers
finished with exit 0 and zero fatal log lines, against 0 of 32 for polling. **[observed]**
Real pipeline (`scripts/generate_cone_frames.py --frames 100 --cones 50 --seed 0`, scene re-scattered every frame):
**one worker produced all 100 frames, 0 restarts, 65.7 s in total** (start-up included), exit 0. The same command with
the polling capture produced 20 frames from 17 workers in 405 s and then gave up. Frame quality matches the old
pipeline: 44 858 to 47 365 points per frame (mean 45 406; the old runs gave about 45 000), sweep duration
(`t` max) 0.0341 to 0.0343 s, 97 distinct point counts among 100 frames (so the frames are not repeats of one
scan). **[observed]**

**What the GMO data looks like** (probe, **[observed]**), needed to understand `lidar/gmo.py`:
- Coordinates are spherical: `x` azimuth in degrees (positive to the left), `y` elevation in degrees, `z` range in
  metres, `scalar` intensity. Cartesian xyz = r (cos el cos az, cos el sin az, sin el), which reproduces the xyz of
  a PCD written by the old pipeline to 4e-6. Intensity has the same scale (about 0.005 to 2.5).
- One write per render step (60 Hz). A 10 Hz scan arrives as: a lead-in of about 40 points, two slices of about
  22 000 points (about 60 deg each), a tail of about 300 points, then steps with no returns. Together about 44 900
  points, as before.
- Absolute point time = write `timestampNs` + per-point `timeOffsetNs`. One sweep spans about 34 ms, exactly what the
  old annotator reported. A new scan starts about 65 ms after the previous tail, so the 0.04 s gap rule still
  separates sweeps.
- The first writes after start-up are garbage (azimuth outside the window); the existing azimuth check rejects them.
- The `scanComplete` flag does not mark the end of a window sweep in any way we could use (it is 0 only on the second
  big slice), so the timestamp/bin completeness check in `lidar/sweep.py` stays. The assembler itself was reused
  unchanged: `gmo_to_raw` produces the same slice dict the annotator used to.

**Related pitfall found on the way.** Isaac puts a `cv2` folder containing a `config.py` on `sys.path` after
`SimulationApp` starts, so a later `import config` can resolve to the wrong file. Our worker works only because
`config` is imported before the app starts. The generic top-level names `config` and `utils` are a risk; rename them
if this ever breaks.

## 7. Missing far ground returns, and the ground material

**Symptom.** In a point-cloud viewer the ground showed only as rings out to about 10 to 12 m; beyond that only the
cones were visible. Frame data agreed (sensor height 0.4 m): 40 884 ground points within 6 m, 3 297 between 6 and
12 m, none between 12 and 30 m. **[observed]**

**It is not a geometric or range limit.** The ATX has about 20 channels between -2 and 0 deg, which would hit flat
ground at 11 to 229 m (`height / tan(elevation)`), and every channel range group reaches at least 60 m. The rays hit;
the returns were not delivered. Ground intensity fell from 0.0082 (0 to 6 m) to 0.0056 (6 to 12 m), toward the lowest
intensity seen anywhere (0.0052), which is consistent with a detection floor on weak grazing returns. **[observed;
the threshold itself is a hypothesis, no intensity or sensitivity attribute exists on the ATX prim]**

**Cause: the ground had no non-visual material.** RTX sensors read a surface's strength from its *non-visual
material* (`isaacsim.core.experimental.materials.NonVisualMaterial`: a base such as asphalt or concrete, a coating,
and attributes), independent of how it looks. Our ground slab had none. Empty scene, ATX at 0.4 m, ground material
varied (one fresh Isaac process each): **[observed]**

| Ground material | Max ground distance | Points per frame | Points beyond 12 m |
|---|---|---|---|
| none (previous behaviour) | 9.1 m | 44 715 | 0 |
| asphalt, concrete, dirt, steel, aluminum+paint | 114.7 m | about 96 500 | about 26 000 |
| retroreflective attribute (unrealistic upper bound) | 114.7 m | 96 557 | 26 147 |
| `calibration_lambertion`, `emissive` | 9.1 m | 44 715 | 0 (no effect) |

With any real material the cutoff disappears: 114.7 m is just the edge of the ground slab (the lowest channel with
ground returns, -0.2 deg, would hit at 114.6 m). The material changes the intensity values only modestly (mean
0.0232 concrete, 0.0239 asphalt, 0.0265 dirt, 0.0290 steel).

**Pitfall: the material must be applied to the OPEN stage.** Authored into the scene file (as plain USD attributes
plus a material binding) it has no effect: 44 711 points and 9.1 m, identical to no material. Applied the same way
after the stage is open and the app is running, with plain `pxr` or with Isaac's own API, it works (96 557 points,
114.7 m). Why the renderer ignores a material present at load was not investigated. Code:
`apply_ground_material()` in `scripts/isaac/stage.py`, called by the worker after `open_stage`.

**Decision.** Default `ground_base = "asphalt"` (cone tracks are normally on asphalt), `ground_coating = "none"`,
settable with `--ground-base` / `--ground-coating`; `--ground-base none` reproduces the old, nearly invisible ground.
Both values are stored in `metadata.json`, so a run says which ground it used. Real pipeline check (`--frames 10
--cones 50`): about 96 600 points per frame, ground out to 114.7 m, sweep duration unchanged (0.0341 s). **[observed]**

**Consequences and caveats.**
- Frames have about twice as many points (a PCD grows from about 1.4 MB to about 2.7 MB), so a 1000-frame dataset is
  about 2.7 GB of PCD.
- Datasets made before this change (the 100-frame run of section 6) have the cut-off ground and no ground material in
  their metadata.
- The simulated ground is now a full-range surface at every distance. Whether a real ATX at 0.4 m height sees the
  ground that far was not checked against real data; asphalt is a plausible choice, not a calibrated one.

## 8. Open items

- Fix the `SensorConfig.usd_path` default: it says `atx_lidar.usd`, the file is `ATX_LiDAR.usd`. (Done in code.)
- Cone box sizes: now taken from the cone USD geometry (the old hand-typed 0.38 m big-cone width was wrong; the
  geometry is 0.285 m). (Done.)
- The three checks in section 3 are superseded by section 6; the sweep assembly is still needed (the GMO data is
  still delivered per render step).
- Rename the top-level modules `config` and `utils` to something Isaac cannot shadow.
- A new section 4 caveat: the assembler now consumes GMO slices (via `lidar/gmo.py`) instead of `get_data()` results;
  steps 1-6 of its logic are unchanged.

## Sources

- Hesai ATX User Manual A07 (`assets/ATX_LiDAR/ATX_User_Manual_A07-en-260520.pdf`), sections 1.4 and 3.1, Appendix A.1
- [Isaac Create RTX Lidar Scan Buffer, 5.1](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/py/source/extensions/isaacsim.sensors.rtx/docs/ogn/OgnIsaacCreateRTXLidarScanBuffer.html)
- [Isaac Create RTX Lidar Scan Buffer, 6.0.1](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/py/source/deprecated/isaacsim.sensors.rtx/docs/ogn/OgnIsaacCreateRTXLidarScanBuffer.html)
- [ROS 2 Sensor Graph Migration, 6.0](https://docs.isaacsim.omniverse.nvidia.com/latest/migration_guides/isaac_sim_6_0/ros2_sensor_graph_migration.html)
- [ROS2 RTX Lidar Helper, 6.0.0](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/py/source/extensions/isaacsim.ros2.nodes/docs/ogn/OgnROS2RtxLidarHelper.html)
- [Forum: RTX Lidar Scan Buffer timestamp count is half of the point cloud size](https://forums.developer.nvidia.com/t/rtx-lidar-scan-buffer-timestamp-count-is-half-of-the-point-cloud-size/354993)
- Repo files: `assets/ATX_LiDAR/README.md`, `make_atx_config.py`, `make_atx_usd.py`, `validate_atx.py`, `make_cone_frames.py`
