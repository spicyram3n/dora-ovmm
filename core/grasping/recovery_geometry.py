"""Pure geometry shared by bounded grasp recovery and its tests."""
import numpy as np
from scipy.spatial.transform import Rotation
from .can_alignment import align_side_grasp


def execution_grasps(data, registration, profile):
    result = []
    g = data['target_geometry']
    for grasp in data['grasps']:
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_quat([grasp['orientation'][k] for k in 'xyzw']).as_matrix()
        pose[:3, 3] = [grasp['position'][k] for k in 'xyz']
        try:
            pose, _, _ = align_side_grasp(pose, [g['center_x'], g['center_y']],
                                          g['radius'], profile, level=True)
        except ValueError:
            continue
        result.append((pose @ np.asarray(registration['base_rotation']), grasp['score']))
    return result


def planar_tracking_delta(actual, goal, base):
    """Recover only small planar tracking errors, never change arm geometry."""
    actual, goal, base = np.asarray(actual), np.asarray(goal), np.asarray(base)
    if not all(np.isfinite(v).all() for v in (actual, goal, base)):
        raise ValueError('Nonfinite tracking pose')
    rotation = goal[:3, :3] @ actual[:3, :3].T
    rv = Rotation.from_matrix(rotation).as_rotvec()
    translation = goal[:3, 3] - rotation @ actual[:3, 3]
    desired = rotation @ np.r_[base[:2], 0.] + translation
    delta = np.r_[desired[:2] - base[:2], rv[2]]
    if (np.linalg.norm(rv[:2]) > .002 or abs(translation[2]) > .002
            or abs(delta[2]) > .05 or np.linalg.norm(delta[:2]) > .01):
        raise ValueError('Tracking discrepancy exceeds bounded planar recovery')
    return delta
