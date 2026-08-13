"""The packages under test live in core/, not here, so put core/ on the path
however pytest was invoked -- `pytest` from the repo root works as well as
`python3 -m pytest` from core/."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
