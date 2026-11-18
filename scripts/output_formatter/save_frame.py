
import os
from pathlib import Path

from output_formatter.save_as_pcd import write_pcd
from output_formatter.save_labels import save_labels
from scene.cone_assets import ConeSizes
from utils.frame import Frame

def _atomic(final: Path, write):
    """Write via a temp file and rename, so a crash never leaves a half-written file under the final name."""
    tmp = final.with_name(final.name + ".tmp")
    write(tmp)
    os.replace(tmp, final)

def save_frame(frame: Frame, log_path: Path, cone_sizes: ConeSizes):
    """Writes <log_path>.pcd, then <log_path>.json. The json goes last: its presence means the frame is complete."""
    log_path = Path(log_path)
    _atomic(log_path.with_suffix(".pcd"), lambda p: write_pcd(path=p, frame=frame))
    _atomic(log_path.with_suffix(".json"), lambda p: save_labels(path=p, frame=frame, cone_sizes=cone_sizes))
