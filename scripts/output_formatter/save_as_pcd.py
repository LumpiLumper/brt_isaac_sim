
from utils.frame import Frame, DTYPE

def write_pcd(path, frame: Frame):
    names = DTYPE.names
    n = len(frame.data)
    k = len(names)
    header = (
        "# .PCD v0.7\nVERSION 0.7\n"
        f"FIELDS {' '.join(names)}\n"
        f"SIZE {' '.join(['4'] * k)}\n"
        f"TYPE {' '.join(['F'] * k)}\n"
        f"COUNT {' '.join(['1'] * k)}\n"
        f"WIDTH {n}\nHEIGHT 1\n"
        f"VIEWPOINT 0 0 {frame.sensor_height} 1 0 0 0\n"
        f"POINTS {n}\nDATA binary\n"
    )
    path.write_bytes(header.encode() + frame.data.astype(DTYPE).tobytes())