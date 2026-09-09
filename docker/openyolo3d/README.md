# OpenYOLO3D scene instance extraction

For the supplied `data_hrl` scan, use the [checked preparation and run instructions](HRL.md).

Separate GPU container, following the build/run script convention used by SAM3
and GraspGenX. This is an offline scene job: it writes separate labelled point
sets for our existing scene graph builder. It does not provide a Zenoh service.

Status: wrapper/export/graph handoff checked with synthetic data; Docker build,
checkpoint download and GPU inference have **not** been verified in this workspace
(Docker is unavailable and NVIDIA driver access fails).

## Run on the GPU host

Requires Docker with NVIDIA Container Toolkit. This reproduces upstream's old
PyTorch 1.12.1 / CUDA 11.3 stack and compiles MinkowskiEngine and PointNet2.
The initial build is substantial. New GPU generations may require a newer stack
and source changes; setting a CUDA architecture alone does not ensure support.

Python 3.10 is compiled into an isolated environment against the base image's
libraries. Do not add `conda install python=3.10` to this image: re-solving its
legacy Conda environment against current channels failed with glibc 2.27 versus
2.28 requirements. After updating the Dockerfile, rerun the normal script;
there is no need to clear Docker's build cache or re-prepare the scan.

The legacy dependency set also requires PyYAML below 6. The Dockerfile installs
PyYAML 5.4.1 first using Cython 0.29.37 and disabled build isolation, avoiding
the `cython_sources` failure caused by Cython 3. Keep the PyYAML 5.4.1 pin in
both the Dockerfile and requirements; installing 6.0.1 later does not fix the
earlier build and conflicts with the old dependency requirement.
See the [PyYAML maintainer explanation](https://github.com/yaml/pyyaml/issues/736).

From the repository root:

```bash
bash docker/openyolo3d/run_openyolo3d.sh --download-checkpoints
bash docker/openyolo3d/run_openyolo3d.sh \
  --scene /data/room1 --output /output/room1 \
  --source-frame room1_scan --depth-scale 1000 \
  --labels chair table sofa cabinet cup bottle
```

Place the prepared scene at `docker/openyolo3d/data/room1/`:

```text
room1/
  scene.ply          # exactly one PLY; fused XYZ in metres, RGB vertex colours
  intrinsics.txt     # 4x4 camera intrinsic matrix
  color/0.jpg        # contiguous 0..N-1 names, JPEG RGB
  depth/0.png        # lossless depth, aligned with RGB
  poses/0.txt        # 4x4 camera-to-scene transform, translation in metres
  ...
```

Use aligned colour/depth at the same resolution, with intrinsics for that
resolution. All frames share the same intrinsics. Poses use the optical camera
convention (x right, y down, z forward). Poses and PLY must share one scene frame.
Raw depth divided by `--depth-scale` gives metres; use 1000 for millimetre PNGs.
The wrapper checks layout, resolution and rigid pose matrices. It cannot verify
the physical accuracy of calibration, depth scale or registration.

RGB-D images alone are insufficient: reconstruct/register the scan first to
obtain the fused PLY and camera poses. This wrapper does not perform SLAM or fusion.
Choose representative views to control memory; upstream projects scene points
into multiple frames on the GPU. Labels are your candidate vocabulary, and
multiple instances of the same label are retained.

To use another data directory, export `OPENYOLO3D_DATA=/absolute/path/to/scenes`.
Likewise `OPENYOLO3D_OUTPUT` overrides the output directory. Add `--check-only` to
validate input layout before loading the model. The output subdirectory must be
new for every run, preventing stale segments from earlier predictions.

Checkpoints and Hugging Face caches persist outside the container. The download
command uses upstream's Google Drive archive and extracts only the two required
weights. If Drive is unavailable, obtain them using the
[upstream installation guide](https://github.com/aminebdj/OpenYOLO3D/blob/main/docs/Installation.md)
and place them in `docker/openyolo3d/checkpoints/`:

- `scannet200_val.ckpt`
- `yolo_world_v2_x_obj365v1_goldg_cc3mlite_pretrain_1280ft-14996a36.pth`

YOLO-World may download its text encoder on first inference.

## Existing scene graph handoff

The output is `instances.json` plus one `instance_NNNN.npy` per retained mask.
Each array contains XYZ points in metres in the declared scan frame. Separate
instances retain separate names, including when labels match. Upstream masks may
overlap; these are separate proposals, not a guaranteed disjoint point partition.
Names are local to a run, not persistent tracking IDs. Scores are upstream
ranking scores, not calibrated probabilities.

Supply a measured registration JSON containing `source_frame: "room1_scan"`
and `map_from_source` (a rigid 4x4 transform), then run in the main workspace:

```bash
PYTHONPATH=core python -m scene_graph.openyolo3d \
  --instances docker/openyolo3d/output/room1/instances.json \
  --transform /absolute/path/to/room1_to_map.json \
  --output outputs/scene_graph/room1.json
```

This uses `scene_graph.Instance` and `graph.build`; registration is applied before
computing bounds and relations. Frame mismatches fail explicitly.

## Model choice, researched September 2026

OpenYOLO3D is a reasonable speed-oriented offline baseline. Its authors report
24.7 ScanNet200 validation mAP and approximately 22 seconds per scene, with up to
16x speedup over the compared method. These are benchmark claims, not a live
camera frame rate or measurements on this robot. Reconstruction and data
preparation are additional work. [Paper](https://arxiv.org/abs/2406.02548)

Its masks come from Mask3D; YOLO-World labels those proposals. Missing or merged
Mask3D proposals limit object recovery even if the detector recognizes the class.
There is no evidence here that it beats SAM3 at per-image segmentation: 3D scene
AP and 2D mask AP measure different tasks. Also, our existing `sam3_client.detect`
selects only the highest-scoring mask, so that API cannot currently return all
objects for scene construction.

Alternatives worth considering:

| Option | Fit and tradeoff |
| --- | --- |
| [YOLOE-26 segmentation](https://docs.ultralytics.com/models/yoloe/) | My first candidate for fast **per-frame** open-vocabulary masks. Lift masks with measured depth and associate instances across views. It does not provide globally consistent 3D instances itself; benchmark its masks on your data rather than comparing 2D AP against OpenYOLO3D's 3D AP. |
| [Details Matter, ICCV 2025](https://arxiv.org/html/2507.23134v1) | Accuracy-focused 3D alternative reporting stronger ScanNet200 results, combining image/point proposals and Alpha-CLIP classification. Its appendix reports about 597 seconds per Replica scene on RTX 4090: not a speed upgrade. Check evaluation protocol (Top-1 versus Top-K) before comparing AP. This is a research candidate; an installable official release was not verified. |
| [BoxOVIS](https://arxiv.org/html/2512.19088v1) | Extends the efficient box-based approach to recover rare objects using RGB-D proposals; reports improved results with under one minute per scene. However, its [official repository](https://github.com/ndkhanh360/BoxOVIS) currently only promises a future code release, so it is not an install-now alternative. |

For a reconstructed room, start with OpenYOLO3D and inspect missed/merged small
objects. For live robot RGB-D, benchmark YOLOE-26 alongside SAM3 returning all
masks. Neither choice requires replacing your scene graph construction code.
