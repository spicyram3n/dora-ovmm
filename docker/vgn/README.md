# VGN: grasps from a TSDF

Turns a TSDF of the space around a target into 6-DOF grasps. [VGN](https://github.com/ethz-asl/vgn) (Breyer et al., CoRL 2020) is the grasp network [active_grasp](https://github.com/ethz-asl/active_grasp) builds its next-best-view loop on. There is no gripper setup: the model was trained for the Franka hand, whose grasp frame matches the HSRC palm (see [Grasp frame](#grasp-frame)).

## Start it

On the **host**, from the repository root:

| Command | Why |
| --- | --- |
| `bash docker/vgn/run_vgn.sh` | Build, start, and wait until warm. Ready about 5 s after the container starts. |
| `docker compose -f docker/compose.yaml logs -f vgn` | See the logs |
| `docker compose -f docker/compose.yaml exec -T vgn python3 zenoh_rpc.py tcp/127.0.0.1:7449 vgn` | Readiness probe, exit 0 = ready |
| `docker compose -f docker/compose.yaml stop vgn` | Stop it |

- **Weights:** the build downloads them (1 MB, upstream's Google Drive link) and checks their hash. No volume to keep.
- **Address:** Zenoh key `vgn/predict`, port `7449`. Clients connect to it by default ([zenoh_rpc.py](../../core/utils/zenoh_rpc.py)).
- **Cost:** about 320 MiB of GPU memory. 10-25 ms per request, round trip from the dev container.

## What goes in

A TSDF grid. GraspGenX takes points and SAM3 an image; VGN takes a fused volume.

| Property | Value |
| --- | --- |
| Shape | `(40, 40, 40)` float32, indexed `[x, y, z]` |
| Extent | A 0.3 m cube of 7.5 mm voxels. Fixed: the network was trained at this scale. |
| Values | `(tsdf + 1) / 2` in the voxels Open3D reports near a surface; `0` everywhere else, whether unseen or far from a surface |
| Placement | Axis-aligned, z up. The surface the object stands on sits 5 cm above the cube's floor, where the training scenes put their table. |

Build it with `GraspVolume` in [core/perception/tsdf.py](../../core/perception/tsdf.py). It makes the same Open3D calls VGN was trained with, and its grid matches upstream's `map_cloud_to_grid` bit for bit. Fusing one 640×480 frame takes about 35 ms. Open3D is already in the dev container.

## What comes out

| Field | Format |
| --- | --- |
| Key | `vgn/predict` |
| Payload | The grid, raw float32 bytes |
| `threshold` | Optional; default `0.9`, the minimum quality |
| `voxel_size` | Optional; default `0.0075` |
| `quality` | Optional; `1` also returns the quality volume |
| Reply payload | `(M, 4, 4)` float32 poses, `(M,)` widths in metres, `(M,)` qualities best first, then `(40, 40, 40)` quality if asked |
| Reply attachment | JSON with `num_grasps`, `voxel_size`, `quality` |

- **Frame:** poses come back in the grid's frame, in metres from its corner voxel. `GraspVolume.frame_from_grid @ poses` puts them in the fixed frame.
- **No grasps is a normal reply.** `M = 0` means nothing graspable has been seen yet: look again.
- **Post-processing is upstream's own:** Gaussian-smoothed quality, masked to voxels just outside an observed surface, thresholded, then local maxima.

Server code: [app.py](app.py).

## Use it from Python

```python
from core.grasping import vgn_client
from core.perception.tsdf import GraspVolume, origin_for

volume = GraspVolume(origin_for(target_centre, support_z))   # fixed frame, e.g. odom
volume.integrate(depth_m, k, odom_from_camera)                # once per view
poses, widths, qualities = vgn_client.predict(volume.grid())
poses = volume.frame_from_grid @ poses                        # grid -> odom
on_target = vgn_client.inside(poses, box_min, box_max)        # VGN grasps anything in the cube
```

The client is [vgn_client.py](../../core/grasping/vgn_client.py). `inside()` is active_grasp's target filter: it keeps grasps whose fingertips land in the target's box.

## Grasp frame

| | VGN (Franka hand) | HSRC `hand_palm_link` |
| --- | --- | --- |
| Approach | +z | +z |
| Fingers close along | y | y |
| Origin | Base of the fingers; they reach 5 cm past it | Palm; pads 1.5-5.4 cm in front |
| Max opening | 8 cm | 12 cm at the open fingertips |

The axes agree, so the client uses a VGN pose as the palm pose unchanged. The HSRC measurements come from [bugs/grasp-pose-convention.md](../../bugs/grasp-pose-convention.md). The gripper is symmetric: a grasp turned 180° about z is the same grasp.

## Limits

**Object width.** VGN learned the Franka hand's 8 cm opening. Synthetic cans on a table, fused over 1-6 views from 30°, 45° and 53° (HSR head-like) off vertical:

| Can diameter | Grasps at quality ≥ 0.9 |
| --- | --- |
| 4-6 cm | After every view; best 0.97-1.00, from every elevation |
| 7 cm | Rarely; best 0.8-0.9 |
| 7.5 cm (Pringles) | None; best 0.4-0.8, falling as more views arrive |

The HSRC hand opens wider than VGN believes any hand can. For Pringles-sized objects VGN is the bottleneck, not the view; view angle made no difference for cans up to 6 cm.

Other limits:

- **Not target-aware:** filter with `inside()` and a box from SAM3's points.
- **One 0.3 m cube:** objects must fit in it with table below.
- **Trained in simulation** on blocks and household meshes; real depth noise is untested here.

## Versions

| What | Pinned to |
| --- | --- |
| VGN | `devel` @ `f51c3fd` (2023; the branch active_grasp uses) |
| robot_helpers | `ba6e7af` (only `Transform` is used) |
| Weights | `vgn_conv.pth`, sha256 `ba3391d0…4cec4b`; the same file ships in the `corl2020` data archive |
| torch | 2.5.1 + CUDA 12.4 |
| zenoh | 1.9.0, as the other model servers |
