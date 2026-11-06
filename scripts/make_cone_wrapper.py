"""
Build a wrapper USD around a Blender cone export:
  - references the raw geometry file
  - adds RTX non-visual (LiDAR/radar) labels to the cone materials as overrides

Re-exporting from Blender only touches the *_geo.usd file; labels survive.

Run in Isaac Sim's Script Editor, or:  ./python.sh make_cone_wrapper.py
Before running: rename the Blender export to Small_Cone_Blue_geo.usd.
"""

import sys, os
from pxr import Sdf, Usd, UsdGeom

# material name -> (base, coating, [attributes])
# Valid tokens: see the "Supported values" table in the Isaac Sim docs,
# isaacsim.core.experimental.materials (Non-Visual Materials).
# Values below are placeholders until you have measured intensity data.
LABELS_BLUE = {
    "Cone_Blue": ("plastic", "none", ["none"]),  # verify "plastic" is a listed base token
    "Cone_White": ("plastic", "paint", ["retroreflective"]),  # verify "paint" coating token
}

LABELS_ORANGE = {
    "Cone_Orange": ("plastic", "none", ["none"]),
    "Cone_White": ("plastic", "paint", ["retroreflective"]),
}

LABELS_YELLOW = {
    "Cone_Yellow": ("plastic", "none", ["none"]),
    "Cone_Black": ("plastic", "paint", ["retroreflective"]),
}


def wrap_cone(geo_path):
    """
    Wrap a cone USD around a Blender export, adding non-visual labels.
    :param geo_path: path to the Blender-exported USD file (e.g. Small_Cone_Blue_geo.usd)
    :return: path to the wrapper USD file (e.g. Small_Cone_Blue.usd)
    """
    geo_path = os.path.abspath(os.path.expanduser(geo_path))
    name = os.path.splitext(os.path.basename(geo_path))[0].removesuffix("_geo")
    root = f"/{name}"
    out_path = os.path.join(os.path.dirname(geo_path), f"{name}.usd")

    labels = {
        
    }

    stage = Usd.Stage.CreateNew(out_path)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)

    root = UsdGeom.Xform.Define(stage, root).GetPrim()
    root.GetReferences().AddReference(geo_path)  # uses the geo file's defaultPrim
    stage.SetDefaultPrim(root)

    labels = LABELS_BLUE if "Blue" in name else LABELS_ORANGE if "Orange" in name else LABELS_YELLOW

    for mat, (base, coating, attrs) in labels.items():
        p = stage.OverridePrim(f"{root.GetPath()}/_materials/{mat}")
        p.CreateAttribute("omni:simready:nonvisual:base", Sdf.ValueTypeNames.Token, custom=True).Set(base)
        p.CreateAttribute("omni:simready:nonvisual:coating", Sdf.ValueTypeNames.Token, custom=True).Set(coating)
        p.CreateAttribute("omni:simready:nonvisual:attributes", Sdf.ValueTypeNames.TokenArray, custom=True).Set(attrs)

    # Sanity check: confirm the overrides land on real prims from the geo file
    for mat in labels:
        p = stage.GetPrimAtPath(f"{root.GetPath()}/_materials/{mat}")
        print(mat, "type:", p.GetTypeName() or "MISSING (path mismatch)",
            "| base:", p.GetAttribute("omni:simready:nonvisual:base").Get())
    print("Wrote", out_path)

    stage.GetRootLayer().Save()
    return out_path

def main():
    geo_path = sys.argv[1] if len(sys.argv) > 1 else input("Path to geo file:").strip()

    wrap_cone(geo_path)


if __name__ == "__main__":
    main()