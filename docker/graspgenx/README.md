# GraspGenX for the HSRC hand

Generate grasp poses from an object point cloud. Register the hand once, then start the server when needed.

## 1. Export the hand — once

Inside the **ROS dev container**, with the workspace built, run from the repository root:

```bash
bash docker/graspgenx/gripper_export/export.sh
```

Output: `docker/graspgenx/gripper_export/hsrc_hand/`, containing the URDF and meshes. Rerun only when the hand description changes. The export uses `hsrc_description` for HSRC geometry.

## 2. Register the hand — once

In a **host terminal**:

```bash
cd docker/graspgenx
bash run_wizard.sh
```

Open `http://localhost:8080` and:

1. Set **+Z** along the approach direction and **+X** along finger closing. For HSRC, these correspond to the URDF's local Z and local Y.
2. Check open and closed joint positions.
3. Select `parallel_2f`.

The wizard saves `x_grippers/hsrc_hand/`. Add the mesh needed by the viewer:

```bash
cp x_grippers/hsrc_hand/vis_mesh.obj x_grippers/hsrc_hand/coll_mesh.obj
```

This is a display-mesh workaround. An offset preview does not change the numeric grasp poses.

## 3. Check the setup

Still on the host, in `docker/graspgenx`, run each viewer separately:

```bash
bash run_vis.sh
bash run_demo.sh
```

The first shows the hand opening and closing. The second generates grasps for sample point clouds. First inference downloads about 1 GB of weights into `checkpoints/`.

Optional views:

```bash
bash run_vis.sh hsrc_hand --show-sweep-volume
bash run_demo.sh --vis-top-grasp-meshes --num-top-grasp-meshes 10
```

## 4. Start the server

```bash
bash run_graspgenx.sh
```

The script starts the server in the background and waits for model readiness. It loads the model once and listens on Zenoh key `graspgenx/generate`. Use `docker compose -f docker/compose.yaml logs -f graspgenx` from the repository root for logs, or `docker compose -f docker/compose.yaml stop graspgenx` to stop it. See the root README for shared server setup.

A client exists at [graspgenx_client.py](../../core/grasping/graspgenx_client.py); [core/grasping/pick.py](../../core/grasping/pick.py) is the entry point that drives it.

## Request format

| Field | Format |
| --- | --- |
| Key | `graspgenx/generate` |
| Payload | Raw float32 XYZ points, shape `(N, 3)` |
| `gripper_name` | Optional; default `hsrc_hand` |
| `num_grasps` | Optional; default `200` |
| Reply payload | `(M, 4, 4)` float32 poses followed by `(M,)` float32 scores |
| Reply attachment | JSON with `num_grasps` and `gripper_name` |

Poses use the input point-cloud frame. See [app.py](app.py) for the server implementation.

## Fix common problems

| Problem | Fix |
| --- | --- |
| Tiny cube or missing hand in viewer | Copy `vis_mesh.obj` to `coll_mesh.obj` as above |
| Permission denied in model/config folders | From `docker/graspgenx`, run `sudo chown -R $(whoami):$(whoami) checkpoints x_grippers`, then retry |
| `Generator config not found` | Leave `GRASPGENX_CHECKPOINT_DIR` unset for automatic downloads; the scripts mount the default weights path |
| Python packages missing inside the image | Use `uv run python3`, which uses GraspGenX's environment |
| Custom hand missing in upstream viewer | Pass `--root /opt/graspgenx/assets/x_grippers` |

Keep `checkpoints/` and `x_grippers/` to reuse weights and registration. The Dockerfile includes the compiler, Python headers, and `uv` needed by the upstream dependencies.
