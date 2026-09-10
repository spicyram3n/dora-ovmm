# OpenYOLO3D

Extract labelled 3D objects from a prepared RGB-D scan for the scene graph. This runs offline in a GPU container.

**Using `data_hrl`?** Follow [HRL.md](HRL.md).

**Validation:** wrapper/export/graph handoff checked with synthetic data. Docker build, downloads, and GPU inference have not been verified in this workspace.

## 1. Prepare the scan

Requires Docker with NVIDIA Container Toolkit. The legacy PyTorch 1.12.1 / CUDA 11.3 stack may need changes for newer GPUs.

From the repository root, place data under `docker/openyolo3d/data/room1/`:

```text
room1/
  scene.ply       # exactly one PLY: XYZ in metres, RGB vertex colours
  intrinsics.txt  # 4×4 camera intrinsic matrix
  color/0.jpg     # RGB frames numbered 0..N-1
  depth/0.png     # aligned lossless depth, same numbering
  poses/0.txt     # 4×4 camera-to-scene poses, same numbering
```

- RGB and depth must share resolution and intrinsics.
- Poses use optical camera axes: x right, y down, z forward. Pose translations and PLY coordinates use metres in the same scene frame.
- Raw depth divided by `--depth-scale` gives metres; use `1000` for millimetre depth.
- Reconstruct/register the scan first. This tool does not do SLAM or fusion.

## 2. Download and run

Run on the **GPU host**, from the repository root:

```bash
bash docker/openyolo3d/run_openyolo3d.sh --download-checkpoints
bash docker/openyolo3d/run_openyolo3d.sh \
  --scene /data/room1 --output /output/room1 \
  --source-frame room1_scan --depth-scale 1000 \
  --labels chair table sofa cabinet cup bottle
```

Add `--check-only` to validate inputs without loading the model. Use a **new output subdirectory for each run**. Choose representative views to limit GPU memory use.

| Setting | Purpose |
| --- | --- |
| `OPENYOLO3D_DATA=/absolute/path` | Override the host input directory |
| `OPENYOLO3D_OUTPUT=/absolute/path` | Override the host output directory |
| `OPENYOLO3D_REBUILD=1` | Rebuild after changing the image or app |

Example rebuild for the HRL setup:

```bash
OPENYOLO3D_REBUILD=1 bash docker/openyolo3d/run_hrl.sh --check-only
```

Images, checkpoints, and Hugging Face caches are reused. The text encoder may download on first inference.

## 3. Build the scene graph

Output: `instances.json` and one `instance_NNNN.npy` per mask, containing XYZ points in metres.

Create a measured registration JSON with `source_frame: "room1_scan"` and a rigid 4×4 `map_from_source`. Then run in the **main workspace**, from its root:

```bash
PYTHONPATH=core python -m scene_graph.openyolo3d \
  --instances docker/openyolo3d/output/room1/instances.json \
  --transform /absolute/path/to/room1_to_map.json \
  --output outputs/scene_graph/room1.json
```

Check alignment against the map. Input validation cannot verify calibration or registration accuracy. Masks can overlap; instance names are local to each run, and scores are ranking scores rather than calibrated probabilities.

## Fix common problems

| Problem | Fix |
| --- | --- |
| Google Drive download fails | Follow the [upstream installation guide](https://github.com/aminebdj/OpenYOLO3D/blob/main/docs/Installation.md) and put the two weights below in `checkpoints/` |
| `ConfigParsingError` about lazy inheritance | Rebuild; the image forces `lazy_import=False` for the pinned config |
| PyYAML `cython_sources` build failure | Keep PyYAML `5.4.1`, Cython `0.29.37`, and the Dockerfile's disabled build isolation |
| Conda/glibc conflict | Keep the isolated Python build; do not replace it with `conda install python=3.10` |

Required weights:

```text
scannet200_val.ckpt
yolo_world_v2_x_obj365v1_goldg_cc3mlite_pretrain_1280ft-14996a36.pth
```

The model uses Mask3D proposals and YOLO-World labels. Missing or merged proposals can miss objects; inspect results before navigation.
