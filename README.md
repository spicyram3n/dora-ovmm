# HSR object search and grasping

Find objects with SAM3, move within reach with Nav2, and generate grasp poses with GraspGenX.

## Start here

1. Open this folder in VS Code and choose **Reopen in Container**. First setup downloads ROS packages and builds the workspace.
2. Follow the [navigation quick start](core/navigation/README.md) to launch simulation, build the scene graph, and search for an object.
3. For grasp generation, complete the [GraspGenX setup](docker/graspgenx/README.md).
4. With a saved `grasps.yaml` and MoveIt running, use [grasp execution](ros2_ws/src/grasp_execution/README.md).

For one launch from simulation startup through search and base placement, see the
[single-launch instructions](core/navigation/README.md#single-launch-simulation-to-ready-for-grasping).

**Current limit:** search stops after base placement. This checkout has no `core/run_pipeline.py`; the combined grasp-generation command shown in older docs is unavailable.

## Start the model servers

Run each command in a separate **host terminal**, from the repository root:

```bash
bash docker/sam3/run_sam3.sh
bash docker/graspgenx/run_graspgenx.sh
```

The scripts build their Docker images on first use. Register the HSRC gripper before using GraspGenX.

## Other tasks

| Task | Instructions |
| --- | --- |
| Build or query object memory | [Scene graph](core/scene_graph/README.md) |
| Extract objects from a recorded scan | [OpenYOLO3D](docker/openyolo3d/README.md) |
| Send camera data between machines | [Vision transport](vision-transport-poorna/README.md) |

Vision transport covers camera data. The old `robot_transport/` bridge setup is absent from this checkout.

## Rebuild ROS packages

After changing ROS sources, run inside the dev container:

```bash
ROS2_REBUILD=1 bash /home/ws/.devcontainer/postCreate.sh
```

Container rebuilds reuse the mounted workspace. A ROS distro, architecture, or system-library change may require a clean build.

Using Docker without VS Code? Mount this repository at `/home/ws`, then run `bash /home/ws/.devcontainer/postCreate.sh` inside the container. A plain image build does not run that setup.

## Connect model servers across machines

On one machine, Zenoh discovers the servers automatically; no router is needed.
Across machines, set this in the **client terminal**, replacing the IP with the server PC's address:

```bash
export ZENOH_CONNECT="tcp/192.168.1.50:7447,tcp/192.168.1.50:7448"
```

SAM3 uses port `7447`; GraspGenX uses `7448`. Server endpoints can be changed with `ZENOH_LISTEN`. These connections are separate from ROS 2 DDS and its RMW setting.
