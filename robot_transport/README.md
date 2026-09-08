# Robot–PC transport

## 1. Install

Run inside each **existing ROS container**. No additional containers are needed.
Copy `robot_transport/` to `/home/ws/robot_transport/` on both machines.

```bash
sudo apt-get update
sudo apt-get install -y curl unzip ros-humble-image-transport \
  ros-humble-compressed-image-transport ros-humble-compressed-depth-image-transport
uname -m
```

For **x86_64**, install the verified Zenoh bridge **1.6.2** binary. This avoids
the glibc requirement that prevented APT installation on Ubuntu 22.04.

```bash
curl -fL --retry 3 \
  https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/1.6.2/zenoh-plugin-ros2dds-1.6.2-x86_64-unknown-linux-gnu-standalone.zip \
  -o /tmp/zenoh-1.6.2.zip
printf '%s\n' 'a3401fe963240e2c65b13db14e8c58529e84d79eb769fdfe45fb27bd5b7d0c44  /tmp/zenoh-1.6.2.zip' | sha256sum -c -
```

Continue only if the checksum reports `OK`:

```bash
unzip -o /tmp/zenoh-1.6.2.zip zenoh-bridge-ros2dds -d /tmp/zenoh-1.6.2
sudo install -m 0755 /tmp/zenoh-1.6.2/zenoh-bridge-ros2dds \
  /usr/local/bin/zenoh-bridge-ros2dds
zenoh-bridge-ros2dds --version
```

For another architecture, use its matching asset from the
[1.6.2 release](https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/tag/1.6.2).
Use the same bridge version on both machines. Installation must be repeated
if the container is recreated unless included in its image.

## 2. Robot setup — domain 5

Start robot drivers and Nav2 using your normal bringup commands after setting:

```bash
source /opt/ros/humble/setup.bash
source /home/ws/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=5
```

Restart any robot nodes previously started in another domain.

In a separate terminal inside the robot ROS container, start image compression:

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=5 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 launch /home/ws/robot_transport/robot_images.launch.py
```

In another terminal inside that container, start the bridge:

```bash
cd /home/ws
sh robot_transport/run_bridge.sh robot
```

## 3. PC setup — domain 9

Inside the PC ROS container, replace `ROBOT_IP` with the robot host address:

```bash
cd /home/ws
sh robot_transport/run_bridge.sh pc ROBOT_IP
```

In a separate terminal, start image decompression:

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=9 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  ros2 launch /home/ws/robot_transport/pc_images.launch.py
```

Set every PC application terminal to domain 9 and CycloneDDS. On hardware,
use `Navigator(use_sim_time=False)` and `grab_rgbd(use_sim_time=False)`.

**Same-machine simulation:** run the simulator and robot image launch in domain
5, the PC image launch in domain 9, and both bridges. Use `127.0.0.1` for
`ROBOT_IP`. Simulation consumers use simulation time and the bridged `/clock`.

## Transferred interfaces

These are the allowlists in `robot.json5` and `pc.json5`. Delivery requires a
running publisher/server and a matching subscriber/client.

| Topic | Direction | Purpose |
| --- | --- | --- |
| `/scan` | Robot → PC | Laser scan |
| `/odom` | Robot → PC | Odometry |
| `/joint_states` | Robot → PC | Joint state |
| `/tf` | Both directions | Dynamic transforms |
| `/tf_static` | Robot → PC | Static transforms |
| `/map` | Both directions | Map from robot or PC SLAM |
| `/clock` | Robot → PC | Simulation clock |
| `/head_rgbd_sensor/rgb/camera_info` | Robot → PC | Camera calibration |
| `/zenoh/rgbd/rgb/compressed` | Robot → PC | Compressed RGB |
| `/zenoh/rgbd/depth/compressedDepth` | Robot → PC | Compressed registered depth |
| `/initialpose` | PC → robot | Initial localization pose |

| Action | Client → server |
| --- | --- |
| `/navigate_to_pose` | PC → robot |
| `/compute_path_to_pose` | PC → robot |
| `/head_trajectory_controller/follow_joint_trajectory` | PC → robot |

Actions include goal requests, feedback, results, and cancellation. Other
services/actions and PointCloud2 are excluded. Only one side should publish
`map → odom` and the active map; stop robot AMCL if PC SLAM owns that transform.

### Image topics and settings

The robot encodes the raw topics below; the PC decodes back to the same names.
Only the compressed topics in the table above cross Zenoh.

| Raw topic on robot and PC | Encoding on the link |
| --- | --- |
| `/head_rgbd_sensor/rgb/image_rect_color` | JPEG quality 85 |
| `/head_rgbd_sensor/depth_registered/image_rect_raw` | Humble compressedDepth: PNG level 3, 10 m cutoff, float quantization parameter 100 |

`camera_ros2.py::grab_rgbd()` pairs the decoded images using their preserved
capture timestamps (50 ms tolerance). The launch files do not synchronize.
16UC1 depth preserves in-range integer values; 32FC1 is quantized. Validate depth
accuracy before grasping. Republishers run continuously and do not set camera
resolution/rate or enforce a maximum frame age.

The bridge connects explicitly over TCP **7449**, with scouting disabled.
TF and scan have higher publication priority than images. Action query timeouts
are **5 s** for goal/cancel replies and **3600 s** for final results. These are
maximum waits, not delays; application deadlines still apply. A bridge timeout
alone does not cancel a goal. `run_bridge.sh` inherits `CYCLONEDDS_URI` if set.

## 4. Verify

In a PC ROS terminal:

```bash
source /opt/ros/humble/setup.bash
source /home/ws/ros2_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=9
ros2 topic echo /scan --once --field header --qos-reliability best_effort
ros2 topic hz /head_rgbd_sensor/rgb/image_rect_color --window 30
```

Ctrl+C, then check depth and navigation separately:

```bash
ros2 topic hz /head_rgbd_sensor/depth_registered/image_rect_raw --window 30
# Ctrl+C before the next commands.
ros2 action list -t
ros2 run tf2_ros tf2_echo map base_footprint
```

Compare received image rates with the robot's raw-topic rates. The 20–30 Hz
target requires the source to publish that fast. Test a short navigation goal
and cancellation with images streaming. Real-robot performance remains unverified.

If images are missing:

- Check raw images in domain 5, compressed images in domain 5, compressed images
  in domain 9, then decoded images in domain 9. This locates where delivery stops.
- Use `ros2 topic info TOPIC --verbose` to check publisher QoS. Stock Humble
  republishers require RELIABLE inputs; BEST_EFFORT needs a supported driver
  setting change or a QoS-aware republisher.
- Read both bridge and image-launch terminals. Humble's “subscribeImpl with five
  arguments” message can fall back successfully; check actual frame delivery.
  Local tests also encountered shutdown crashes, so clean shutdown is unverified.
