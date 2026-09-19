"""Read a Nav2 map off disk, for the tools that work without a running stack."""

import numpy as np
import yaml


def read_map(yaml_path):
    """The PGM, its resolution and its map-frame origin, rows running north.

    Rows are flipped so row 0 is the lowest y, which is the order every caller
    here indexes in; Nav2's own PGM has row 0 at the highest y."""
    import cv2

    meta = yaml.safe_load(yaml_path.read_text())
    if meta.get("mode", "trinary") != "trinary" or meta.get("negate", 0):
        raise ValueError(f"{yaml_path} is not a plain trinary map")
    image = cv2.imread(str(yaml_path.parent / meta["image"]), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise ValueError(f"cannot read the map image named in {yaml_path}")
    return np.flipud(image), float(meta["resolution"]), np.array(meta["origin"][:2], float)


def to_cells(points, resolution, origin):
    """Map-frame xy to (column, row) in a north-up image."""
    return (np.asarray(points, float) - origin) / resolution
