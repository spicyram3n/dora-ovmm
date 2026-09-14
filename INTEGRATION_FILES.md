# Integrating with feat/pipeline

Use the matching files from feat/grasp together. This is the tested launch/tree
layout; replacing pick.py alone omits camera, planning and scene dependencies.
This list concerns the ROS mission, not the newer feat/pipeline web dashboard.
Keep a clean checkout or commit local work before copying files between branches.

## Replace existing files

Main route:
- `launch/search.launch.py`
- `core/pipeline/mission_tree.py`
- `core/pipeline/actions.py`
- `core/grasping/pick.py`

Supporting runtime:
- `core/perception/camera_ros2.py`
- `core/perception/pointcloud.py`
- `core/navigation/base_placement.py`
- `core/navigation/graspable.py`
- `core/navigation/nav2_client.py`
- `core/reasoner/query.py`
- `core/scene_graph/graph.py`
- `core/scene_graph/instance.py`
- `launch/ik_solver.launch.py`
- `launch/move_group.launch.py`
- `launch/planning_model.py`
- `config/moveit/ompl_planning.yaml`
- `config/moveit/sensors_xtion.yaml`
- `ros2_ws/src/hsr_rgbd/scripts/depth_camera_relay.py`
- `.devcontainer/Dockerfile`

## Add

- `launch/apartment.launch.py` and optional `launch/search_nvidia.sh`
- `worlds/` and `assets/models/`
- `config/map/kitchen_objects.pgm` and `config/map/kitchen_objects.yaml`
- `config/nav2/kitchen_objects.yaml`
- `config/scene_graph/kitchen_objects.json`
- `config/moveit/servo.yaml` (servo remains disabled)
- `docker/graspgenx/x_grippers/hsrc_hand/closing_profile.json`
- `.devcontainer/grasp_patches/`
- `core/grasping/test/` and `core/pipeline/test/`
- `GRASPING_STATUS.md` and `worlds/README.md` for setup and known limitations

Apply the simulator and MoveIt patches from the revisions recorded alongside
them, following GRASPING_STATUS.md, then rebuild the ROS workspace and restart
the stack. For NVIDIA rendering the ROS container needs GPU device access;
copying the launcher or driver libraries alone is insufficient.

`graspgenx_client.py`, `sam3_client.py`, `core/utils/transforms.py` and the normal
model servers remain dependencies already present in feat/pipeline. Neither
run_pipeline.py nor the old grasp_execution package is used by this structure.
The standalone launch/grasp.launch.py from feat/pipeline is not the main entry;
use search.launch.py with mode:=grasp|pickup|auto.

Current verification: 60 unit/integration tests pass. The kitchen-start live
spray trial reached the grasp pose but failed bilateral closure. Successful
physical pickup is a separate validation requirement; this integration does
not establish universal grasping.
