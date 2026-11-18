"""The RTX lidar capture loop. Needs a running Isaac Sim app, so import it only after SimulationApp has started.

Data is taken from GenericModelOutput inside a Replicator writer callback, NOT by polling
annotator.get_data() of IsaacCreateRTXLidarScanBuffer. The polling path segfaulted within a few frames on every
worker we measured: Isaac wraps the lidar node's raw output pointer in a zero-copy array and copies it, and that copy
reads freed memory when the node swaps its buffer in between (annotator_utils.py `_get_data_from_ptr`, `array.copy()`).
A writer callback runs at a point where the buffers are guaranteed valid, and we copy what we need inside it.
See notes.md ("get_data() crash") for the measurements.
"""

from collections import deque

import numpy as np
import omni.kit.app
import omni.replicator.core as rep
import omni.timeline
from isaacsim.sensors.experimental.rtx import Lidar, LidarSensor, parse_generic_model_output_data
from omni.replicator.core import Writer

from config import CaptureConfig
from lidar.gmo import gmo_to_raw
from lidar.sweep import Sweep, SweepAssembler, SweepSpec

class GmoCollector(Writer):
    """Collects one slice per render step; what write() copies out stays valid after the callback returns."""

    def __init__(self):
        self.data_structure = "renderProduct"
        self.annotators = [rep.annotators.get("GenericModelOutput")]
        self.slices: deque = deque(maxlen=256)

    def write(self, data) -> None:
        for render_product in data.get("renderProducts", {}).values():
            raw = render_product.get("GenericModelOutput")
            if isinstance(raw, dict):
                raw = raw.get("data")
            gmo = parse_generic_model_output_data(raw)
            n = gmo.numElements
            if n == 0:
                continue
            # spherical coordinates: x = azimuth [deg], y = elevation [deg], z = range [m]
            self.slices.append(gmo_to_raw(gmo.x[:n], gmo.y[:n], gmo.z[:n], gmo.scalar[:n],
                                          gmo.timeOffsetNs[:n], gmo.timestampNs))


rep.WriterRegistry.register(GmoCollector)


class LidarCapture:
    def __init__(self, sensor_prim: str, spec: SweepSpec, capture: CaptureConfig):
        self.spec, self.cfg = spec, capture
        # wraps the existing OmniLidar prim of the ATX; accumulate_outputs=None leaves the value authored on it alone
        self.sensor = LidarSensor(Lidar(sensor_prim, accumulate_outputs=None, aux_output_level="BASIC"), annotators=[])
        self.writer = self.sensor.attach_writer(GmoCollector.__name__)  # the registry key is the class name
        self.app = omni.kit.app.get_app()
        self.timeline = omni.timeline.get_timeline_interface()

    def start(self) -> None:
        self.timeline.play()
        for _ in range(self.cfg.warmup_steps):
            self.app.update()

    def stop(self) -> None:
        self.timeline.stop()

    def capture_sweep(self) -> Sweep | None:
        """One complete sweep of the current cone layout, or None if max_steps render steps pass without one.

        Slices from the previous layout may still be in flight, so settle steps pass first and everything collected
        until then is dropped.
        """
        for _ in range(self.cfg.settle):
            self.app.update()
        self.writer.slices.clear()
        assembler = SweepAssembler(self.spec)
        for _ in range(self.cfg.max_steps):
            self.app.update()
            while self.writer.slices:
                sweep = assembler.add(self.writer.slices.popleft())
                if sweep is not None:
                    return sweep
        return None
