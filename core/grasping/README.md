# grasping

Turning an object's point cloud into ranked grasp poses that MoveIt can
execute.

## Files

| File | Purpose |
|---|---|
| `graspgenx_client.py` | Sends an object-centred cloud to the GraspGenX server, gets poses and scores back |
| `grasp_io.py` | Writes the top candidates to YAML, best score first |
| `visualize.py` | Plots the cloud, the candidates coloured by score, and the gripper mesh at the best one |

## Running

Nothing here is a program; `core/run_pipeline.py` is the entry point.

```bash
cd core
python3 run_pipeline.py "pringles can" --gripper hsrc_hand
```

That writes `config/targets/<slug>/` containing `grasps.yaml`, `cloud.ply` and
`plot.png`. Execute one with:

```bash
ros2 launch grasp_execution move_to_grasp.launch.py
```

## Things worth knowing

- **GraspGenX wants a single object, not a scene.** `run_pipeline.py` centres
  the cloud on its centroid before sending and undoes that on the way out, so
  poses come back in the frame the points arrived in.
- **Several grasps are saved, not just the best.** The top-scoring grasp is
  not always reachable by this arm, so the consumer works down the list.
  `grasp_io.TOP_K` sets how many.
- **The server needs the gripper registered once** before its first real run.
  See `docker/graspgenx/README.md`.
- **`visualize.py` is matplotlib only**, so it needs no display server and
  works over SSH.
