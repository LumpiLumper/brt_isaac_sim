"""Step 2: turn atx_channels.json (from the ATX manual, Appendix A.1) into an RTX-lidar emitter config.

Plain Python, no Isaac Sim needed:

    python3 make_atx_config.py            # writes atx_emitter_config.json next to this file

Modelling choices (the manual describes a 1-D rotating mirror, Isaac Sim has no mirror model):
  - ROTARY scan with the output restricted to a 120 deg azimuth window. One full revolution is split
    into 4500 firing states (360 / 0.08 deg) at 10 Hz, so the window holds 1500 columns per channel.
  - One emitter per channel, ordered by descending vertical angle (= RingID order, 0-based).
  - Horizontal offset per emitter = even-frame value (the sim cannot alternate even/odd frames).
  - Super-resolution channels have no range in the manual; they take the range of the nearest
    normal channel by vertical angle.
"""

import json
from pathlib import Path

HERE = Path(__file__).resolve().parent

# datasheet, section 1.4
H_FOV_DEG = 120.0
H_RES_DEG = 0.08
FRAME_RATE_HZ = 10.0
NEAR_RANGE_M = 0.5
FAR_RANGE_M = 198.0
RANGE_PRECISION_M = 0.03  # 1 sigma
WAVELENGTH_NM = 905.0

STATES_PER_REV = round(360.0 / H_RES_DEG)  # 4500
FIRING_RATE_HZ = STATES_PER_REV * FRAME_RATE_HZ  # 45000


def load_channels():
    ch = json.loads((HERE / "atx_channels.json").read_text())
    ch.sort(key=lambda c: -c["elev"])  # RingID order
    normal = [c for c in ch if c["max_m"] is not None]
    for c in ch:
        if c["max_m"] is None:
            ref = min(normal, key=lambda n: abs(n["elev"] - c["elev"]))
            c["min_m"], c["max_m"] = ref["min_m"], ref["max_m"]
            c["range_from_channel"] = ref["channel"]
    return ch


def main():
    ch = load_channels()
    n = len(ch)

    # distinct (min, max) pairs -> range groups, per-emitter rangeId (0-based)
    pairs = sorted({(c["min_m"], c["max_m"]) for c in ch})
    range_id = [pairs.index((c["min_m"], c["max_m"])) for c in ch]

    period_ns = 1e9 / FIRING_RATE_HZ
    spacing_ns = period_ns / n
    fire_time_ns = [round(spacing_ns * (i + 0.5)) for i in range(n)]

    cfg = {
        "source": "ATX_User_Manual_A07-en-260520.pdf, Appendix A.1",
        "scan": {
            "scanType": "ROTARY",
            "scanRateBaseHz": FRAME_RATE_HZ,
            "patternFiringRateHz": FIRING_RATE_HZ,
            "reportRateBaseHz": FIRING_RATE_HZ,
            "statesPerRevolution": STATES_PER_REV,
            "azimuthWindowDeg": [-H_FOV_DEG / 2, H_FOV_DEG / 2],
            "azimuthWindowNote": "boresight = sensor +X; step 3 must convert to validStart/EndAzimuthDeg",
            "expectedPointsPerFrame": round(H_FOV_DEG / H_RES_DEG) * n,
        },
        "core": {
            "numberOfEmitters": n,
            "numberOfChannels": n,
            "nearRangeM": NEAR_RANGE_M,
            "farRangeM": FAR_RANGE_M,
            "rangeAccuracyM": RANGE_PRECISION_M,
            "waveLengthNm": WAVELENGTH_NM,
            "maxReturns": 1,
            "rangeCount": len(pairs),
            "rangesMinM": [p[0] for p in pairs],
            "rangesMaxM": [p[1] for p in pairs],
        },
        "emitterState": {
            "channelId": list(range(1, n + 1)),
            "ringId": list(range(n)),
            "azimuthDeg": [c["az_even"] for c in ch],
            "elevationDeg": [c["elev"] for c in ch],
            "fireTimeNs": fire_time_ns,
            "rangeId": range_id,
            "atxChannelNo": [c["channel"] for c in ch],
        },
    }

    out = HERE / "atx_emitter_config.json"
    out.write_text(json.dumps(cfg, indent=1))

    el = cfg["emitterState"]["elevationDeg"]
    print(f"wrote {out}")
    print(f"emitters: {n}, elevation {max(el)} .. {min(el)} deg")
    print(f"states/rev {STATES_PER_REV}, firing rate {FIRING_RATE_HZ:.0f} Hz, fire spacing {spacing_ns:.0f} ns")
    print(f"range groups (min,max m): {pairs}")
    print(f"expected points/frame in window: {cfg['scan']['expectedPointsPerFrame']}")
    print(f"super-res channels given borrowed range: {sum('range_from_channel' in c for c in ch)}")


if __name__ == "__main__":
    main()
