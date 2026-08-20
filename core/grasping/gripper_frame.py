"""The one place that knows how GraspGenX's grasp frame relates to the
robot's hand.

GraspGenX works in a canonical grasp frame: +Z approach, +X closing. The HSR
closes along Y instead (hand_palm_link's fingers sit at y = +/-0.0245), so the
two differ by a quarter turn about the shared approach axis. The gripper's
config.json records that as `base_rotation`, written when the gripper was
registered.

Naming follows the rest of the pipeline: `canonical_from_palm` maps a point
in hand_palm_link coordinates into canonical ones, so poses compose as

    world_from_palm = world_from_canonical @ canonical_from_palm

graspgenx_client applies this once, on the way in. Everything downstream of
it -- grasps.yaml, MoveIt, the IK solver -- speaks hand_palm_link only.
"""

import json
from functools import lru_cache
from pathlib import Path

import numpy as np

GRIPPERS_DIR = Path(__file__).resolve().parents[2] / "docker" / "graspgenx" / "x_grippers"


@lru_cache(maxsize=None)
def canonical_from_palm(gripper):
    """(4, 4) mapping hand_palm_link coordinates into GraspGenX's frame."""
    config = json.loads((GRIPPERS_DIR / gripper / "config.json").read_text())
    return np.asarray(config["base_rotation"], dtype=np.float64)


def palm_from_canonical(gripper):
    """The inverse. Needed to draw vis_mesh.obj, which ships in the canonical
    frame, at a pose that is now expressed in hand_palm_link."""
    return np.linalg.inv(canonical_from_palm(gripper))
