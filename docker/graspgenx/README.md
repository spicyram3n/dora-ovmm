# GraspGenX for the HSRC hand

Turns an object point cloud into grasp poses. Set the hand up once, then start the server when you need it.

## Setup at a glance

| # | Where | Command | Why | How often |
| --- | --- | --- | --- | --- |
| 1 | Dev container, repo root | `bash docker/graspgenx/gripper_export/export.sh` | Export the hand's URDF and meshes | Once, or when the hand changes |
| 2 | Host, `docker/graspgenx` | `bash run_wizard.sh` | Register the hand with GraspGenX | Once |
| 3 | Host, `docker/graspgenx` | `bash run_vis.sh`, then `bash run_demo.sh` | Check the hand and test grasps | After setup |
| 4 | Host, `docker/graspgenx` | `bash run_graspgenx.sh` | Start the server | Each session |

---

## 1. Export the hand

In the **dev container**, workspace built, from the repository root:

```bash
bash docker/graspgenx/gripper_export/export.sh
```

Output: `docker/graspgenx/gripper_export/hsrc_hand/` (URDF + meshes, from `hsrc_description`).

## 2. Register the hand

On the **host**:

```bash
cd docker/graspgenx
bash run_wizard.sh
```

Open <http://localhost:8080>, then:

1. Set **+Z** along the approach direction and **+X** along finger closing. For HSRC these are the URDF's local Z and local Y.
2. Check the open and closed joint positions.
3. Select `parallel_2f`.

The wizard saves `x_grippers/hsrc_hand/`. Then copy the mesh the viewer needs:

```bash
cp x_grippers/hsrc_hand/vis_mesh.obj x_grippers/hsrc_hand/coll_mesh.obj
```

This only fixes the display. An offset preview does not change the grasp poses.

## 3. Check it

On the host, in `docker/graspgenx`, run each separately:

| Command | Shows |
| --- | --- |
| `bash run_vis.sh` | The hand opening and closing |
| `bash run_vis.sh hsrc_hand --show-sweep-volume` | Plus the volume the fingers sweep |
| `bash run_demo.sh` | Grasps generated for sample point clouds |
| `bash run_demo.sh --vis-top-grasp-meshes --num-top-grasp-meshes 10` | The 10 best grasps as hand meshes |

The first inference downloads about 1 GB of weights into `checkpoints/`.

## 4. Start the server

```bash
bash run_graspgenx.sh
```

Starts in the background and waits until the model is ready. It listens on Zenoh key `graspgenx/generate`, port `7448`.

| From the repo root | Why |
| --- | --- |
| `docker compose -f docker/compose.yaml logs -f graspgenx` | See the logs |
| `docker compose -f docker/compose.yaml stop graspgenx` | Stop it |

- **Client:** [graspgenx_client.py](../../core/grasping/graspgenx_client.py).
- **Caller:** [pick.py](../../core/grasping/pick.py) drives it during a pick.
- **Shared setup:** the [root README](../../README.md#3-start-the-model-servers) covers setup shared with SAM3.

## Request format

| Field | Format |
| --- | --- |
| Key | `graspgenx/generate` |
| Payload | Raw float32 XYZ points, shape `(N, 3)` |
| `gripper_name` | Optional; default `hsrc_hand` |
| `num_grasps` | Optional; default `200` |
| Reply payload | `(M, 4, 4)` float32 poses, then `(M,)` float32 scores |
| Reply attachment | JSON with `num_grasps` and `gripper_name` |

Poses come back in the input cloud's frame. Server code: [app.py](app.py).

## Fix common problems

| Problem | Fix |
| --- | --- |
| Tiny cube or no hand in the viewer | Copy `vis_mesh.obj` to `coll_mesh.obj` (step 2) |
| Permission denied in model/config folders | In `docker/graspgenx`: `sudo chown -R $(whoami):$(whoami) checkpoints x_grippers` |
| `Generator config not found` | Leave `GRASPGENX_CHECKPOINT_DIR` unset; the scripts mount the default weights path |
| Python packages missing in the image | Use `uv run python3` (GraspGenX's environment) |
| Custom hand missing in the upstream viewer | Pass `--root /opt/graspgenx/assets/x_grippers` |

Keep `checkpoints/` and `x_grippers/` to reuse the weights and the registration.
