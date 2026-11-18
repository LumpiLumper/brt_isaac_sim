
from pathlib import Path

from utils.frame import Frame
from save_as_pcd import write_pcd
from save_labels import save_labels

def save_frame(frame: Frame, log_path: Path):
    log_path = Path(log_path)
    write_pcd(path=log_path.with_suffix(".pcd"), frame=frame)
    save_labels(path=log_path.with_suffix(".json"), cones=frame.cones)