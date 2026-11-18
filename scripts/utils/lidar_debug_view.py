from omni.kit.scripting import BehaviorScript
import omni.replicator.core as rep
import omni.usd
from pxr import Usd


def find_lidars(root_prim):
    """All OmniLidar prims at or below root_prim (references/instances included)."""
    return [
        p for p in Usd.PrimRange(root_prim, Usd.TraverseInstanceProxies())
        if p.GetTypeName() == "OmniLidar" and p.IsActive()
    ]


class LidarDebugDraw(BehaviorScript):
    """Draw the point cloud of every RTX lidar under the prim this script is attached to.

    Attach it to a lidar's Xform to debug just that sensor, or to /World (or any
    ancestor) to debug all lidars below it.
    """

    def on_init(self):
        self._writer = None
        self._rps = []

    def on_play(self):
        stage = omni.usd.get_context().get_stage()
        lidars = find_lidars(stage.GetPrimAtPath(str(self.prim_path)))
        if not lidars:
            print(f"[lidar_debug] no OmniLidar found under {self.prim_path}")
            return

        for lidar in lidars:
            path = lidar.GetPath().pathString
            print(f"[lidar_debug] drawing {path}")
            self._rps.append(rep.create.render_product(path, [1, 1], name=lidar.GetName()))

        self._writer = rep.writers.get("RtxLidarDebugDrawPointCloud")
        self._writer.attach(self._rps)

    def on_stop(self):
        if self._writer:
            self._writer.detach()
            self._writer = None
        for rp in self._rps:
            rp.destroy()
        self._rps = []

    def on_update(self, current_time: float, delta_time: float):
        pass

    def on_destroy(self):
        self.on_stop()
