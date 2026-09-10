# Start the HSR grasp simulation

These instructions start the Toyota HSR Gazebo simulation, MoveIt, RGB-D
perception, and grasp execution in separate terminals. Run them from the host
workspace `/home/aswin/hsr_ros2_ws`.

The Docker container must be running first:

```bash
docker start dora-ovmm
docker start graspgenx_server
docker start sam3_server
```

## 1. Start Gazebo

For the verified visible NVIDIA window on this workstation (2026-09-09), use:

```bash
docker exec -it dora-ovmm bash -lc 'bash /home/ws/ros2_ws/launch/run_grasp_sim_nvidia.sh'
```

This uses the matching host driver libraries saved in `.runtime/nvidia-gl`
(NVIDIA 580.159.04). Refresh those libraries after a host driver update. The
original software-rendering command below remains available, but produced a
black Qt window on this workstation during the September 9 test. The NVIDIA
launcher was verified using a desktop capture, not only an internal render.

If restarting the container, wait for `docker restart dora-ovmm` to finish and
print the container name before launching Gazebo. Starting a new process while
the restart is still pending can cause that process to be terminated.

```bash
docker exec -it dora-ovmm bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ws/ros2_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  bash /home/ws/ros2_ws/launch/run_grasp_sim.sh
'
```

Wait until Gazebo shows the apartment, table, can, and stowed robot.

## 2. Start MoveIt and the RGB-D octomap

In a second terminal:

```bash
docker exec -it dora-ovmm bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ws/ros2_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ros2 launch /home/ws/ros2_ws/launch/hsrc_move_group_sim.launch.py
'
```

This uses the RGB-D image and camera-info topics for the octomap. It does not
use PointCloud2.

## 3. Capture a target and generate GraspGenX poses

The simulation publishes RGB on `image_raw`, so set both topic overrides:

```bash
docker exec -it dora-ovmm bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ws/ros2_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export RGB_TOPIC=/head_rgbd_sensor/rgb/image_raw
  export DEPTH_TOPIC=/head_rgbd_sensor/depth_registered/image
  export ZENOH_CONNECT=tcp/127.0.0.1:7447,tcp/127.0.0.1:7448
  cd /home/ws
  python3 -u core/run_pipeline.py "pringles can" --gripper hsrc_hand
'
```

The output is saved under `config/targets/pringles_can/` as `grasps.yaml`,
`cloud.ply`, and `plot.png`.

## 4. Execute a grasp

Use the normal strict executor first. It performs MoveIt IK and collision
checks before moving:

```bash
docker exec -it dora-ovmm bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ws/ros2_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ros2 launch grasp_execution move_to_grasp.launch.py \
    grasp_file:=/home/ws/config/targets/pringles_can/grasps.yaml
'
```

Do not use `allow_preview_failure:=true` for normal testing. That option is a
diagnostic bypass and can allow an unsafe approach.

## 5. Stop everything

First stop the terminal launch commands with `Ctrl-C`. If a controller goal is
still active, run:

```bash
docker exec dora-ovmm bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ws/ros2_ws/install/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  ros2 service call /arm_trajectory_controller/follow_joint_trajectory/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
  ros2 service call /omni_base_controller/follow_joint_trajectory/_action/cancel_goal action_msgs/srv/CancelGoal "{}"
'
```

To close any remaining simulation processes after the launch terminals have
been stopped:

```bash
docker exec dora-ovmm bash -lc '
  pgrep -af "hsrc_move_group_sim|hsrc_apartment_near_shelf|ign gazebo|depth_camera_relay|move_to_grasp"
'
```

Terminate only the PIDs shown by that command, then close the Gazebo window.
