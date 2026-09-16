"""Mission events for the web dashboard (web/server.py), one JSON line each on stdout.

Silent unless MISSION_EVENTS is set, so command-line runs print exactly as before."""

import json
import os

PREFIX = "[EVENT] "


def emit(kind, **data):
    if not os.environ.get("MISSION_EVENTS"):
        return
    data["kind"] = kind
    # Convert NumPy values to JSON and flush so the dashboard updates immediately.
    print(PREFIX + json.dumps(data, default=lambda value: value.tolist()), flush=True)
