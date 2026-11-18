"""Turn one GenericModelOutput (GMO) write of an RTX lidar into the slice dict that lidar/sweep.py consumes.

A GMO write holds the returns of one render step. For our lidar the coordinates are spherical: x is the azimuth in
degrees (positive to the left of the boresight), y the elevation in degrees (positive up), z the range in metres, and
`scalar` the intensity. Per-point times are offsets from the write's own timestamp.
"""

import numpy as np


def gmo_to_raw(azimuth_deg, elevation_deg, range_m, intensity, time_offset_ns, timestamp_ns) -> dict | None:
    """Slice dict in the layout annotator.get_data() used to return: {"data": (n, 3) xyz, "info": {...}}.

    None for an empty write. The inputs are copied, so the buffers they came from may be released afterwards.
    """
    n = len(range_m)
    if n == 0:
        return None
    az = np.radians(np.asarray(azimuth_deg, dtype=np.float64))
    el = np.radians(np.asarray(elevation_deg, dtype=np.float64))
    r = np.asarray(range_m, dtype=np.float64)
    xyz = np.column_stack([r * np.cos(el) * np.cos(az), r * np.cos(el) * np.sin(az), r * np.sin(el)])
    return {
        "data": xyz.astype(np.float32),
        "info": {
            "intensity": np.array(intensity, dtype=np.float32),
            "distance": r.astype(np.float32),
            "azimuth": np.array(azimuth_deg, dtype=np.float32),
            "elevation": np.array(elevation_deg, dtype=np.float32),
            "timestamp": (np.asarray(time_offset_ns, dtype=np.int64) + int(timestamp_ns)).astype(np.uint64),
        },
    }
