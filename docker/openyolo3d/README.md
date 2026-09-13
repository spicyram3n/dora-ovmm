# OpenYOLO3D

Turns a prepared RGB-D scan into labelled 3D objects for the scene graph. Runs offline in a GPU container.

- **Using `data_hrl`?** Follow [HRL.md](HRL.md) instead.
- **Tested so far:** the wrapper, export and graph handoff, with synthetic data. The Docker build, downloads and GPU inference are **not** verified here.

## At a glance

| # | Where | Command | Why |
| --- | --- | --- | --- |
| 1 | Anywhere | Put the scan in `docker/openyolo3d/data/room1/` | The container reads it from there |
| 2 | GPU host, repo root | `bash docker/openyolo3d/run_openyolo3d.sh --download-checkpoints` | Get the model weights (once) |
| 3 | GPU host, repo root | `bash docker/openyolo3d/run_openyolo3d.sh --scene ... --output ...` | Detect and label objects |
| 4 | Dev container, repo root | `python -m core.scene_graph.openyolo3d ...` | Turn them into a scene graph |

Needs Docker with the NVIDIA Container Toolkit. The legacy PyTorch 1.12.1 / CUDA 11.3 stack may need changes on newer GPUs.

---

## 1. Prepare the scan

```text
docker/openyolo3d/data/room1/
  scene.ply       # exactly one PLY: XYZ in metres, RGB vertex colours
  intrinsics.txt  # 4×4 camera intrinsics
  color/0.jpg     # RGB frames 0..N-1
  depth/0.png     # aligned lossless depth, same numbering
  poses/0.txt     # 4×4 camera-to-scene poses, same numbering
```

| Rule | Why |
| --- | --- |
| RGB and depth share resolution and intrinsics | Depth is read at RGB pixels |
| Poses use optical axes: x right, y down, z forward | That is what the model expects |
| Poses and PLY in metres, same scene frame | Objects must land where the PLY is |
| Raw depth ÷ `--depth-scale` = metres | Use `1000` for millimetre depth |
| Reconstruct and register the scan first | This tool does no SLAM or fusion |

## 2. Download and run

On the **GPU host**, from the repository root:

```bash
bash docker/openyolo3d/run_openyolo3d.sh --download-checkpoints
bash docker/openyolo3d/run_openyolo3d.sh \
  --scene /data/room1 --output /output/room1 \
  --source-frame room1_scan --depth-scale 1000
```

| Flag | Use it to |
| --- | --- |
| `--scene /data/room1` | Choose the scan (container path of `data/room1`) |
| `--output /output/room1` | Choose the output folder. **Use a new one per run.** |
| `--source-frame room1_scan` | Name the scan's frame; the registration must use the same name |
| `--depth-scale 1000` | Convert raw depth to metres |
| `--labels ...` | Choose the classes to look for. Default: the 198 [ScanNet200 classes](../../config/scene_graph/scannet200.yaml) |
| `--check-only` | Validate the inputs without loading the model |

| Environment variable | Use it to |
| --- | --- |
| `OPENYOLO3D_DATA=/absolute/path` | Change the host input folder |
| `OPENYOLO3D_OUTPUT=/absolute/path` | Change the host output folder |
| `OPENYOLO3D_REBUILD=1` | Rebuild after changing the image or app |

Example rebuild for the HRL setup: `OPENYOLO3D_REBUILD=1 bash docker/openyolo3d/run_hrl.sh --check-only`

- **Views:** choose representative views to limit GPU memory use.
- **Caches:** images, checkpoints and Hugging Face caches are reused. The text encoder may download on first inference.

## 3. Build the scene graph

The run writes `instances.json` and one `instance_NNNN.npy` per mask (XYZ points in metres).

1. Write a registration JSON: `source_frame: "room1_scan"` plus a measured rigid 4×4 `map_from_source`.
2. In the **dev container**, from the repository root:

```bash
python -m core.scene_graph.openyolo3d \
  --instances docker/openyolo3d/output/room1/instances.json \
  --transform /absolute/path/to/room1_to_map.json \
  --output outputs/scene_graph/room1.json
```

3. Check the alignment against the map. Input checks cannot prove calibration or registration.

- **Masks can overlap.** Instance names are local to each run.
- **Scores rank, they don't measure:** scores are ranking scores, not probabilities.
- **Next:** [preview the viewpoints](../../core/navigation/README.md#real-room-from-a-scan) before driving.

## Fix common problems

| Problem | Fix |
| --- | --- |
| Google Drive download fails | Follow the [upstream guide](https://github.com/aminebdj/OpenYOLO3D/blob/main/docs/Installation.md) and put the two weights below in `checkpoints/` |
| `ConfigParsingError` about lazy inheritance | Rebuild; the image forces `lazy_import=False` for the pinned config |
| PyYAML `cython_sources` build failure | Keep PyYAML `5.4.1`, Cython `0.29.37` and the Dockerfile's disabled build isolation |
| Conda/glibc conflict | Keep the isolated Python build; do not use `conda install python=3.10` |

**Required weights:**

```text
scannet200_val.ckpt
yolo_world_v2_x_obj365v1_goldg_cc3mlite_pretrain_1280ft-14996a36.pth
```

The model uses Mask3D proposals and YOLO-World labels. Missing or merged proposals miss objects, so check the results before navigating.
