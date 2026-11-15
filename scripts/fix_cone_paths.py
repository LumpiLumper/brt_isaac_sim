"""Rewrite absolute /home/brt-01/... asset references in assets/cones/*.usd to relative paths."""
import glob
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})

from pxr import Sdf

OLD = "/home/brt-01"


def fix(path):
    return "./" + path.split("/")[-1] if path.startswith(OLD) else path


def visit(spec):
    for lst in (spec.referenceList, spec.payloadList):
        for field in ("prependedItems", "appendedItems", "explicitItems", "addedItems"):
            items = getattr(lst, field)
            new = []
            for it in items:
                cls = type(it)
                new.append(cls(fix(it.assetPath), it.primPath) if it.assetPath else it)
            if list(items) != new:
                setattr(lst, field, new)
    for child in spec.nameChildren:
        visit(child)


for f in sorted(glob.glob("assets/cones/*.usd")):
    layer = Sdf.Layer.FindOrOpen(f)
    for prim in layer.rootPrims:
        visit(prim)
    layer.Save()
    print("fixed", f)

app.close()
