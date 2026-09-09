"""Offline scene inference, exporting one point set per detected instance."""
import argparse
import json
import sys
from pathlib import Path

import numpy as np


def validate_scene(scene):
    clouds = list(scene.glob('*.ply'))
    if len(clouds) != 1:
        raise ValueError('Scene must contain exactly one coloured PLY in metres')
    poses = list((scene / 'poses').glob('*.txt'))
    if not poses:
        raise ValueError('No camera poses found')
    intrinsic = np.loadtxt(scene / 'intrinsics.txt')
    if intrinsic.shape != (4, 4) or not np.isfinite(intrinsic).all():
        raise ValueError('intrinsics.txt must contain a finite 4x4 matrix')
    from PIL import Image
    resolution = None
    for i in range(len(poses)):
        pose = np.loadtxt(scene / 'poses' / f'{i}.txt')
        if (pose.shape != (4, 4) or not np.isfinite(pose).all()
                or not np.allclose(pose[3], [0, 0, 0, 1])
                or not np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-4)
                or not np.isclose(np.linalg.det(pose[:3, :3]), 1, atol=1e-4)):
            raise ValueError(f'Pose {i} must be a rigid camera-to-scene transform')
        with Image.open(scene / 'color' / f'{i}.jpg') as rgb, Image.open(scene / 'depth' / f'{i}.png') as depth:
            if rgb.size != depth.size or (resolution and rgb.size != resolution):
                raise ValueError('Use aligned RGB/depth images with one shared resolution')
            resolution = rgb.size
    return clouds[0]


def export_instances(output, points, masks, classes, scores, prompts, source_frame, threshold):
    if masks.shape != (len(points), len(scores)) or len(classes) != len(scores):
        raise ValueError('Prediction masks do not match point cloud / scores')
    if not np.isfinite(points).all() or not np.isfinite(scores).all():
        raise ValueError('Nonfinite points or scores')
    output.mkdir(parents=True, exist_ok=False)
    instances = []
    for i, (class_id, score) in enumerate(zip(classes, scores)):
        class_id = int(class_id)
        if score < threshold or not 0 <= class_id < len(prompts):
            continue
        selected = points[masks[:, i].astype(bool)]
        if not len(selected):
            continue
        filename = f'instance_{i:04d}.npy'
        np.save(output / filename, selected.astype(np.float32))
        instances.append(dict(name=f'instance_{i:04d}', label=prompts[class_id],
                              confidence=float(score), points=filename))
    (output / 'instances.json').write_text(json.dumps(dict(
        source_frame=source_frame, units='m', instances=instances), indent=2))
    return len(instances)


def main():
    if sys.argv[1:] == ['--download-checkpoints']:
        import gdown
        import shutil
        import tempfile
        import zipfile
        destination = Path('pretrained/checkpoints')
        destination.mkdir(parents=True, exist_ok=True)
        names = ['scannet200_val.ckpt',
                 'yolo_world_v2_x_obj365v1_goldg_cc3mlite_pretrain_1280ft-14996a36.pth']
        if all((destination / name).is_file() for name in names):
            print('Checkpoints already present')
            return
        with tempfile.TemporaryDirectory() as temp:
            archive = str(Path(temp) / 'checkpoints.zip')
            gdown.download(id='1FneLaYaClWDO51L9lIvlTQbheh5SfOFD', output=archive)
            with zipfile.ZipFile(archive) as bundle:
                for name in names:
                    member, = [p for p in bundle.namelist() if Path(p).name == name]
                    with bundle.open(member) as src, (destination / (name + '.partial')).open('wb') as dst:
                        shutil.copyfileobj(src, dst)
                    (destination / (name + '.partial')).replace(destination / name)
        print('Checkpoints downloaded')
        return
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--labels', nargs='+', required=True)
    parser.add_argument('--source-frame', required=True)
    parser.add_argument('--depth-scale', type=float, default=1000.0,
                        help='Raw depth units per metre (1000 for millimetres)')
    parser.add_argument('--confidence', type=float, default=0.1)
    parser.add_argument('--check-only', action='store_true')
    args = parser.parse_args()
    if not np.isfinite(args.depth_scale) or args.depth_scale <= 0:
        parser.error('--depth-scale must be positive and finite')
    if not 0 <= args.confidence <= 1:
        parser.error('--confidence must lie in [0, 1]')
    cloud = validate_scene(args.scene)
    if args.output.exists():
        parser.error('--output must be a new directory')
    if args.check_only:
        print('Scene layout and camera matrices passed validation')
        return
    import torch
    from utils import OpenYolo3D
    from models.Mask3D.mask3d import load_mesh_or_pc
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA GPU is required')
    pc = load_mesh_or_pc(str(cloud), datatype='point cloud')
    if not len(pc.points) or not pc.has_colors():
        raise ValueError('PLY must be nonempty and contain RGB vertex colours')
    for name in ['scannet200_val.ckpt', 'yolo_world_v2_x_obj365v1_goldg_cc3mlite_pretrain_1280ft-14996a36.pth']:
        if not (Path('pretrained/checkpoints') / name).is_file():
            raise FileNotFoundError('Run run_openyolo3d.sh --download-checkpoints first')
    model = OpenYolo3D('pretrained/config.yaml')
    with torch.inference_mode():
        prediction = model.predict(str(args.scene), depth_scale=args.depth_scale, text=args.labels)
    masks, classes, scores = prediction[args.scene.name]
    def array(value):
        return value.detach().cpu().numpy() if torch.is_tensor(value) else np.asarray(value)
    points = np.asarray(load_mesh_or_pc(str(cloud), datatype='point cloud').points)
    count = export_instances(args.output, points, array(masks), array(classes), array(scores),
                             args.labels, args.source_frame, args.confidence)
    print(f'Exported {count} instances to {args.output / "instances.json"}')


if __name__ == '__main__':
    main()
