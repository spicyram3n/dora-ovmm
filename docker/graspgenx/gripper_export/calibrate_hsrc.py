"""Rebuild HSRC registration from free-motion FK and opposing pad meshes.

Run with GraspGenX's .venv Python inside its container. A one-time backup
preserves the previous registration. No network or training is required.
"""
import json
from pathlib import Path
import shutil

import numpy as np
import trimesh
import yourdfpy


ROOT = Path('/opt/graspgenx/assets/x_grippers/hsrc_hand')


def geometry(robot, rotation, motor):
    robot.update_cfg({'hand_motor_joint': motor,
                      'hand_l_spring_proximal_joint': 0.0,
                      'hand_r_spring_proximal_joint': 0.0})
    meshes = {}
    for name, source in robot.scene.geometry.items():
        mesh = source.copy()
        mesh.apply_transform(rotation @ robot.scene.graph.get(name)[0])
        meshes[name] = mesh
    # The DAE contains separate rubber pad and finger-body geometries.
    # Identify the thin facing pad within each finger, not its outer body.
    pads = []
    for side in ('l', 'r'):
        candidates = [m for n, m in meshes.items() if n.startswith(f'{side}_distal.dae')]
        pads.append(min(candidates, key=lambda m: m.extents[0]))
    left, right = sorted(pads, key=lambda m: m.centroid[0])
    lower = np.maximum(left.bounds[0], right.bounds[0])
    upper = np.minimum(left.bounds[1], right.bounds[1])
    lower[0] = left.bounds[1, 0]
    upper[0] = right.bounds[0, 0]
    assert np.all(upper > lower), (motor, lower, upper)
    return trimesh.util.concatenate(list(meshes.values())), upper - lower, (lower + upper) / 2


def main():
    config = json.loads((ROOT / 'config.json').read_text())
    robot = yourdfpy.URDF.load(str(ROOT / 'gripper.urdf'))
    rotation = np.asarray(config['base_rotation'])
    opened, extents, center = geometry(robot, rotation, 1.10)
    _, half_extents, half_center = geometry(robot, rotation, 0.55)
    assert center[2] > 0 and half_center[2] > 0
    assert 0.10 < extents[0] < 0.14
    assert 0.06 < half_extents[0] < 0.09
    # Preserve original files before changing assets used by the server.
    backup = ROOT / 'before_fk_calibration'
    if not backup.exists():
        backup.mkdir()
        for name in ('config.json', 'vis_mesh.obj', 'coll_mesh.obj'):
            shutil.copy2(ROOT / name, backup / name)
    config['open'] = {'hand_motor_joint': 1.10,
                      'hand_l_spring_proximal_joint': 0.0,
                      'hand_r_spring_proximal_joint': 0.0}
    config['close'] = {'hand_motor_joint': 0.0,
                       'hand_l_spring_proximal_joint': 0.0,
                       'hand_r_spring_proximal_joint': 0.0}
    config['sweep_volume'] = {'extents': extents.tolist(), 'offset': center.tolist(),
                              'extents2': half_extents.tolist(), 'offset2': half_center.tolist()}
    config['fingertip'] = center.tolist()
    config['standoff'] = [0.0, float(extents[2] / 2)]
    config['bbox'] = opened.bounds.tolist()
    (ROOT / 'config.json').write_text(json.dumps(config, indent=4) + '\n')
    opened.export(str(ROOT / 'vis_mesh.obj'))
    opened.export(str(ROOT / 'coll_mesh.obj'))
    print(json.dumps({'open_pad_volume': config['sweep_volume'], 'bbox': config['bbox']}, indent=2))


if __name__ == '__main__':
    main()
