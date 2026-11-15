# Hesai ATX RTX lidar for Isaac Sim 6.0

Everything needed to rebuild `ATX_LiDAR.usd` from the datasheet (`ATX_User_Manual_A07-en-260520.pdf`, Appendix A.1).

| Step | File | Output |
|---|---|---|
| 1 | channel table pulled from the manual | `atx_channels.json` (116 channels) |
| 2 | `make_atx_config.py` (plain Python) | `atx_emitter_config.json` |
| 3 | `make_atx_usd.py` (Isaac Sim Python) | `ATX_LiDAR.usd` |
| 4 | `make_test_scene.py`, `validate_atx.py` | `ATX_test_scene.usd`, `Outputs/atx_scan.pcd` |

Run from the repo root, inside the container (`docker compose run --rm sim ...`, see `DOCKER.md`):

```bash
python3 assets/ATX_LiDAR/make_atx_config.py
docker compose run --rm sim /isaac-sim/python.sh assets/ATX_LiDAR/make_atx_usd.py
docker compose run --rm sim /isaac-sim/python.sh assets/ATX_LiDAR/make_test_scene.py
docker compose run --rm sim /isaac-sim/python.sh assets/ATX_LiDAR/validate_atx.py
```

## Using the sensor

Reference `ATX_LiDAR.usd` (default prim `/ATX`) and place it. The `OmniLidar` prim is `/ATX/sensor`; the boresight is +X.

## How the ATX is modelled

- 116 emitters, one per channel, ordered by descending elevation (+5.92 to -12.41 deg, RingID order).
- Rotary scan at 10 Hz, 4500 firing states per revolution (0.08 deg), output limited to a 120 deg azimuth window,
  so 1500 columns per channel and 174 000 points per frame if every ray hits.
- Per-emitter horizontal offset = even-frame value from the manual (the sim cannot alternate even/odd frames).
- Ranges 0.5 to 198 m in 6 range groups from the per-channel table. The 52 super-resolution channels have no range in
  the manual and borrow the range of the nearest normal channel.
- 905 nm, 3 cm range accuracy, single return.

## Things to know

- The azimuth window must be written in the 0..360 convention (`300..60`). `-60..60` is not read as a wrapped window and
  yields only half of it. `make_atx_usd.py` does the conversion.
- The sim sweeps a full 360 deg per 0.1 s, so the 120 deg window is swept in the first third of each frame. Per-point
  timestamps therefore differ from the real mirror, which sweeps the 120 deg over the whole frame.
- The RTX lidar delivers one slice per render step (60 Hz), so one ATX sweep arrives as two ~60 deg slices plus a
  few late points. `validate_atx.py` merges them by timestamp. It also stops reading as soon as it has one sweep: the
  6.0.1 annotator crashes nondeterministically in `get_data()` after about 15-40 frames. Re-run if that happens.
- Grazing ground hits (here 1-4 deg below the horizon from 1.5 m height) fall below the sim's detection floor and are
  dropped. That is a property of the test scene, not of the sensor config.
- `scripts/capture_one_scan.py` does not suit this sensor: it can pick up warm-up frames and half sweeps.
- The `.usd` files are written by the container user (uid 1234), so they are read-only for your host user.
