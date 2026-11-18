"""The RTX lidar capture loop. Needs a running Isaac Sim app, so import it only after SimulationApp has started."""

import omni.kit.app
import omni.replicator.core as rep
import omni.timeline

from config import CaptureConfig
from lidar.sweep import Sweep, SweepAssembler, SweepSpec


class LidarCapture:
    def __init__(self, sensor_prim: str, spec: SweepSpec, capture: CaptureConfig):
        self.spec, self.cfg = spec, capture
        self.render_product = rep.create.render_product(sensor_prim, [1, 1], name="atx_cone_frames")
        self.annotator = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
        self.annotator.initialize(outputIntensity=True, outputDistance=True, outputAzimuth=True,
                                  outputElevation=True, outputTimestamp=True)
        self.annotator.attach([self.render_product.path])
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

        Slices from the previous layout may still be in flight, so settle steps pass first and nothing read before
        them is kept. Polling stops the moment a sweep is complete: every get_data() call risks the 6.0.1 crash.
        """
        for _ in range(self.cfg.settle):
            self.app.update()
        assembler = SweepAssembler(self.spec)
        for _ in range(self.cfg.max_steps):
            self.app.update()
            sweep = assembler.add(self.annotator.get_data())
            if sweep is not None:
                return sweep
        return None
