# Run the HRL scan

The original `data_hrl` is nearly ready, but should not be passed directly to
OpenYOLO3D. Inspection found:

- 395 matching RGB/depth/numbered pose files, IDs 0–394 without gaps.
- Saved RGB and depth images are 640×480; sampled depth images are uint16.
- `scene.ply` has 897,702 coloured points, with room-sized coordinate bounds.
- `pose/` must become `poses/`. Its two extra transform files must be excluded:
  upstream counts every file in this folder as a frame.
- `intrinsic/intrinsic_color.txt` must become `intrinsics.txt`, **with its first
  two rows divided by 3** for these saved images. The corrected focal lengths
  are 535.0395 and principal point is (320.5739, 239.7726).

The intrinsic resize factor and depth scale were checked geometrically, rather
than inferred only from filenames: projecting every eighth PLY point into frames
0, 50, 100, 200, 300 and 394 using the numbered poses and depth / 1000 gave median
absolute depth differences of approximately 1–3 mm, with 71–99% of valid projected
samples within 5 cm. Uncorrected intrinsics gave roughly 9–46 cm median errors.
These samples strongly support camera-to-scan poses, metre cloud coordinates,
and millimetre depth. They do not establish perfect alignment for every pixel.
Extra ground/ICP transform compositions tested much worse: do not apply them
again to the numbered poses or use them as a robot-map registration.

## 1. Prepare input (skip if data/hrl already exists)

From the repository root:

```bash
python3 docker/openyolo3d/prepare_hrl.py --stride 10
```

This creates `docker/openyolo3d/data/hrl/` with corrected intrinsics, `scene.ply`,
and `color/`, `depth/`, `poses/`. It selects every tenth frame (40 total), renumbers
the matched triplets to 0–39 and saves original IDs in `preparation.json`.
Original files are not edited. Prepared images/cloud/poses are hardlinks where
possible; treat them as read-only. An existing output directory is rejected.

40 views are a memory-conscious starting point, not a guarantee of full object
coverage. For more views, prepare a separate directory and use its scene name:

```bash
python docker/openyolo3d/prepare_hrl.py --stride 5 \
  --output docker/openyolo3d/data/hrl_dense
# Then use --scene /data/hrl_dense --output /output/hrl_dense for inference.
```

Using all 395 views can consume substantial GPU memory because upstream builds
point-to-frame projection arrays. No changes to the cloud point ordering are made.

## 2. Segment on the Docker GPU host

```bash
bash docker/openyolo3d/run_openyolo3d.sh --download-checkpoints
bash docker/openyolo3d/run_hrl.sh --check-only
bash docker/openyolo3d/run_hrl.sh
```

`run_hrl.sh` supplies a starting vocabulary of furniture and household objects.
Open vocabulary means you can choose those category names; it does not mean the
model invents names for every unknown object. Customize the script's labels or
override them, for example `bash docker/openyolo3d/run_hrl.sh --labels chair table cup`.
For a second run, use `--output /output/hrl_run2` to avoid overwriting results.

Output: `docker/openyolo3d/output/hrl/instances.json` and one XYZ `.npy` per
retained instance. Same-class objects remain separate. Masks can overlap, and
instance names are local to the run. `predictions.txt` is not an input to this
wrapper; it was not present in the on-disk folder during inspection.

Docker and NVIDIA access are unavailable in this workspace, so actual model
inference, checkpoint download, and the image build still need host verification.

## 3. Build with our existing scene graph code

Our graph builder stores points in the robot's `map` frame. First obtain the
rigid transform mapping **this HRL scan** into that map. Save it as
`config/map/hrl_to_map.json`, with keys `source_frame` (value `hrl_scan`) and
`map_from_source` (your measured 4×4 matrix, translation in metres).

Do not reuse `world_to_map.json`: that is for the Gazebo world. The scan's two
auxiliary transform files have not been established as robot-map registration.
Do not substitute identity unless scan and target map coordinates truly coincide.

Once segmentation and registration are available:

```bash
python -m core.scene_graph.openyolo3d \
  --instances docker/openyolo3d/output/hrl/instances.json \
  --transform config/map/hrl_to_map.json \
  --output outputs/scene_graph/hrl.json
```

This loads each point set as `Instance`, runs our existing `graph.build`, and
saves the graph. Bounds, centroids, furniture/object classification and geometric
relations come from our existing code. Room assignment is a separate optional
reasoner step. No graph has been generated yet: predicted instances and a verified
HRL-to-map registration are still missing.

## Inspect test results in Rerun (no map or ROS needed)

`rerun-sdk` is baked into the devcontainer image, so no separate viewer
environment is needed. From the repository root, in the devcontainer:

```bash
python3 -m core.scene_graph.visualize \
  --instances docker/openyolo3d/output/hrl/instances.json \
  --output outputs/scene_graph/hrl_preview
```

This opens the native Rerun desktop viewer with colored point clouds, labelled
bounding boxes and object-to-furniture arrows. Toggle entities under
`scene/instances` and `scene/relations` in the viewer to inspect individual objects.
Labels include instance IDs and model scores. Coordinates remain in `hrl_scan`.
Edges are nearest-furniture `near` hypotheses, not confirmed support/containment.

The command saves `graph.json` and `scene.rrd`, replacing these two files on each
run. To export inside a container without a display, add `--save-only`. Open the
recording later on the PC host:

```bash
python3 -m rerun outputs/scene_graph/hrl_preview/scene.rrd
```

Up to 30,000 points per instance are displayed; all points build the graph.
Optional `--scene docker/openyolo3d/data/hrl/scene.ply` adds gray context
(`trimesh` is already in the devcontainer image). No inference rebuild, ROS,
browser or map registration is needed. `--transform` remains available for a future measured
registration using the existing map graph builder.
