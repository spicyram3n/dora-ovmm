# Robot–PC communication with ZeroMQ

Use the existing ROS Humble containers on the robot and PC. This gateway keeps
CycloneDDS and sends only configured sensor topics through explicit TCP ports.
The old transport launch/config files have been removed.

## Tomorrow: install on both machines

Copy `robot_transport/` into the workspace on both machines. Run these commands
inside the ROS environment that can already see that machine's working topics,
not inside the OpenYOLO3D container:

```bash
sudo apt-get update
sudo apt-get install -y python3-zmq python3-zstandard python3-opencv ros-humble-cv-bridge
source /opt/ros/humble/setup.bash
# Source your existing robot/workspace overlay if your normal bringup needs it.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=5
/usr/bin/python3 -c 'import zmq, zstandard, cv2, rclpy; from cv_bridge import CvBridge; print("Gateway dependencies OK; ZeroMQ", zmq.zmq_version())'
```

Use `/usr/bin/python3` so the APT-installed modules and ROS use the same Python.
No pip upgrade, new DDS implementation, broker or extra container is required.
If your container runs as root, omit `sudo`. Installations disappear when a
container is recreated unless included in its image.

Use **domain 5 on both machines** for this test, including PC RViz/perception.
This preserves ordinary direct DDS communication with the robot. Domain 9 on
PC works for gateway sensors but does not preserve direct access to domain-5
TF/actions. Leave the robot's working `CYCLONEDDS_URI` unchanged; do not globally
set `ROS_LOCALHOST_ONLY=1` or copy another machine's interface XML.

Stop previous sensor bridges and image republishers in their own terminals.
Do not stop the camera driver, lidar, navigation or other working robot nodes.
Removing files here does not stop previously launched processes.

## Check the robot's local inputs first

Run on the robot, with its normal bringup running:

```bash
ros2 topic info /scan --verbose
ros2 topic echo /scan --once --field header --qos-reliability best_effort
ros2 topic info /head_rgbd_sensor/rgb/image_rect_color --verbose
ros2 topic info /head_rgbd_sensor/depth_registered/image_rect_raw --verbose
ros2 topic echo /head_rgbd_sensor/depth_registered/image_rect_raw --once --field header --qos-reliability best_effort
ros2 topic echo /head_rgbd_sensor/rgb/camera_info --once --qos-reliability best_effort
```

If these cannot receive locally, the gateway cannot repair that source: check
actual topic names, domain, publisher QoS, driver and CycloneDDS discovery.
The gateway subscribes BEST_EFFORT, VOLATILE, depth 1. CameraInfo must publish
periodically with this setup; one-shot transient-local calibration needs a QoS
adjustment. Do not run raw-topic diagnostic subscribers on the PC during the
bandwidth test: they can trigger duplicate raw DDS traffic across the network.

## Start the gateway

Find each host's reachable IP with `ip -br address`. Replace `ROBOT_IP` and
`PC_IP` below with actual IPv4 addresses; they are placeholders. Both machines
must use the same `gateway/routes.json`.

From the repository root on the robot:

```bash
ROS_DOMAIN_ID=5 /usr/bin/python3 robot_transport/gateway/gateway.py robot --bind ROBOT_IP --peer PC_IP
```

From the repository root on the PC:

```bash
ROS_DOMAIN_ID=5 /usr/bin/python3 robot_transport/gateway/gateway.py pc --bind PC_IP --peer ROBOT_IP
```

If using containers, run in your existing host-network ROS containers so these
host IPs are available. The source binds TCP 17601–17604 on the selected interface
and filters accepted connections to the peer IP. The PC connects to the robot;
replies return on those connections. If a firewall blocks them, allow only the
PC IP to those robot TCP ports. No firewall is disabled or changed by this code.

## Test on the PC

In another sourced PC terminal, use CycloneDDS and domain 5 as above:

```bash
ros2 topic echo /remote/scan --once --field header --qos-reliability best_effort
ros2 topic echo /remote/head_rgbd_sensor/depth_registered/image_rect_raw --once --field header --qos-reliability best_effort
ros2 topic echo /remote/head_rgbd_sensor/rgb/camera_info --once --qos-reliability best_effort
ros2 topic hz /remote/scan
# Ctrl+C, then:
ros2 topic hz /remote/head_rgbd_sensor/rgb/image_rect_color
# Ctrl+C, then:
ros2 topic hz /remote/head_rgbd_sensor/depth_registered/image_rect_raw
```

Select `/remote/scan` and `/remote/head_rgbd_sensor/...` in RViz, with
**BEST_EFFORT** reliability. Use an existing valid TF fixed frame. Ordinary TF
continues through DDS; missing TF can hide valid scan data in RViz, which is why
we first check the message header independently.

Initial requested ceilings: RGB 20 Hz, depth 20 Hz, scan 30 Hz, CameraInfo 2 Hz.
Actual delivery is limited by source rate, compression and request round trip.
Read receiver logs for delivered rates, errors/timeouts and stale drops. Source
logs report discovered publishers, total received samples and latest arrival age.

Run all streams for five minutes. Check that images and scan stamps advance,
RGB/depth pairs remain available to perception, and other working DDS topics
remain responsive. Topic Hz alone does not establish low latency. Capture-age
measurements require synchronized clocks; the gateway's transit-age checks use
monotonic durations and do not require clock synchronization.

While stationary, stop **only the robot gateway** with Ctrl+C, wait five seconds,
then restart the same command. PC logs should show timeouts during the outage and
resume fresh delivery afterward without restarting the camera or PC gateway.
No old stream history should replay. Then restart the PC gateway and verify again.

If depth reports stale drops, lower its `rate` in both copies of `routes.json`
and retest. Zstd preserves all original depth bits but may send more bytes than
PNG. If source delivery is healthy and network timeouts persist, check IPs,
firewall and link quality; raising ROS queue sizes is not the first fix.

## Consumers and scope

Change PC perception subscriptions to the `/remote/...` topics. See
[consumer remappings and design details](gateway/README.md).
Other topics, actions and commands continue through ordinary CycloneDDS.
The gateway is for latest-state sensor data, not one-shot command delivery.
RGB/depth keep their capture timestamps but arrive independently; use your
existing timestamp synchronizer. Reconstruct camera point clouds on the PC.

## Optional local self-test

After installing dependencies, run on either machine without changing the
robot's running nodes:

```bash
ROS_DOMAIN_ID=73 /usr/bin/python3 robot_transport/gateway/test_gateway.py
```

The test uses localhost and domain 73 in its own processes. It checks actual
ROS/ZeroMQ delivery, integer/float depth preservation, reverse direction, restart
recovery and source stalls. These tests passed in the development environment;
real-robot Wi-Fi rates remain to be measured.
