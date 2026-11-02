"""Make `bhl_arm_sdk` importable when pytest runs from outside this package
(repo root, colcon test); from inside it `python3 -m pytest` already works."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
