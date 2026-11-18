import sys
from pathlib import Path

SCRIPTS = Path(__file__).resolve().parent.parent

# output_formatter modules import each other as top-level modules
# (`from save_as_pcd import ...`) and use `utils.*` as a package.
for p in (SCRIPTS, SCRIPTS / "output_formatter"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))
