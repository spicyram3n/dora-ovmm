# Explicit robot–PC sensor gateway

Uses PyZMQ/ZeroMQ TCP and existing local CycloneDDS. No broker, multicast
discovery, replacement RMW, Docker image, or custom ROS message package.
Four configured routes, one process and port each. Source binds its selected
interface IP and accepts only the configured peer IP. This is IP filtering, not
authentication/encryption; use a trusted robot LAN or an existing VPN.

## Why this design

`test-vision` has useful BEST_EFFORT source subscriptions and a latest-sample
worker queue. Our stock Humble image republishers can fail to match BEST_EFFORT
camera publishers. That is a plausible cause of missing images, not a proven
diagnosis of the hardware incident. Depth CPU load and network backlog are other
possibilities. Missing `/scan` could be discovery, QoS or routing independently.

This gateway retains sensor QoS and a single latest source sample. The receiver
requests one sample at a time; there is no stream history to build up. Requests
time out and recreate their socket. Process restarts get a new sequence epoch.
Encoding runs outside the ROS callback. Separate processes/connections prevent
depth encoding or its TCP retransmissions from directly blocking scan processing.
They still share physical bandwidth and CPU; this is not a hard real-time guarantee.

RGB uses JPEG quality 80. Depth uses Zstd level 1 on the complete serialized ROS
message: no quantization, no depth clipping, no conversion of NaNs or units.
Scan and CameraInfo also use Zstd. CameraInfo uses volatile source QoS; if your
driver publishes calibration only once with transient-local durability, this
configuration needs adapting or the driver must republish calibration.

Purdue network_bridge is a viable generic TCP topic bridge, but adds generic
compression rather than replacing image policy. Its inspected UDP send path
puts an entire compressed message into one datagram, unsuitable for large
images. gRPC supports streaming but still requires explicit buffering and age
policy. ZeroMQ was selected for a small bounded request/reply implementation,
not because its name guarantees higher throughput.

Sources: [Purdue send path](https://github.com/brow1633/network_bridge/blob/main/src/network_bridge.cpp),
[UDP](https://github.com/brow1633/network_bridge/blob/main/src/network_interfaces/udp_interface.cpp),
[ZeroMQ queue/socket controls](https://libzmq.readthedocs.io/en/latest/zmq_setsockopt.html).

## Install and run

In each existing ROS Humble environment:

```bash
sudo apt-get update
sudo apt-get install -y python3-zmq python3-zstandard ros-humble-cv-bridge
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

Stop the old image republishers and old Zenoh bridge for this test. Keep the
robot's working CycloneDDS interface configuration. Do not copy test-vision's
NIC-specific XML or set ROS_LOCALHOST_ONLY on a robot that needs networked DDS.

For shared-domain operation, use **domain 5 on both machines**. Domain 9 on the
PC also works for these gateway routes, but ordinary DDS topics/actions will
then need their own existing bridge. The gateway never changes your domain.

Robot (substitute actual IPs):

```bash
export ROS_DOMAIN_ID=5
python3 robot_transport/gateway/gateway.py robot --bind ROBOT_IP --peer PC_IP
```

PC:

```bash
export ROS_DOMAIN_ID=5
python3 robot_transport/gateway/gateway.py pc --bind PC_IP --peer ROBOT_IP
```

Both sides need identical `routes.json` and matching ROS message definitions.
Allow TCP 17601–17604 from the PC IP to the robot IP if a firewall is active.
No ports are opened by a firewall configuration automatically. Change ports in
the JSON on both machines if occupied. A failed worker stops the launcher and
its siblings visibly instead of silently leaving a partial gateway running.

The PC subscribes to these **new names**:

| Stream | PC output | TCP port | Requested max rate |
| --- | --- | --- | --- |
| RGB | `/remote/head_rgbd_sensor/rgb/image_rect_color` | 17601 | 20 Hz |
| Depth | `/remote/head_rgbd_sensor/depth_registered/image_rect_raw` | 17602 | 20 Hz |
| Scan | `/remote/scan` | 17603 | 30 Hz |
| Calibration | `/remote/head_rgbd_sensor/rgb/camera_info` | 17604 | 2 Hz |

Change PC consumers/RViz to these names. In a shared domain, subscribing to the
original topics can still create parallel raw DDS traffic! Prefixes prevent
gateway loops; they are not network isolation. Leave ordinary working topics
and actions on their original names. No extra PointCloud2 is sent: reconstruct
the camera cloud from depth on the PC if needed.

For Python perception, use ROS remappings when launching your application:

```text
--ros-args
-r /head_rgbd_sensor/rgb/image_rect_color:=/remote/head_rgbd_sensor/rgb/image_rect_color
-r /head_rgbd_sensor/depth_registered/image_rect_raw:=/remote/head_rgbd_sensor/depth_registered/image_rect_raw
-r /head_rgbd_sensor/rgb/camera_info:=/remote/head_rgbd_sensor/rgb/camera_info
```

Original capture stamps and frame IDs are preserved. RGB and depth are delivered
independently: use the existing timestamp synchronizer, and measure matched-pair
rate. This gateway does not manufacture synchronized pairs. TF remains on your
normal working DDS path; in separate domains it needs a separate bridge.

To add a reverse **latest-state topic**, add a route with `source: "pc"`, a new
port, and distinct input/output names. The robot then connects to the PC port.
This delivery policy is inappropriate for one-shot commands, actions, services,
or reliable event history. Keep those on working CycloneDDS. Velocity control
would additionally require a robot-side watchdog; no control route is enabled.

## Overload and diagnostics

The receiver allows one outstanding request per stream. It drops responses whose
source residence time plus request/decode duration exceeds `max_age`. This is a
conservative gateway-age bound using monotonic durations; it does not need clock
synchronization. It cannot detect an already-old capture delivered by the camera
driver with a new local arrival. Check capture timestamps with synchronized
machine clocks as well. TCP kernel buffers exist; resetting a timed-out socket
discards that request, rather than replaying its response on the next connection.

Source logs report discovered publisher count, samples received, and latest
arrival age. Receiver logs report delivered rate, errors/timeouts and stale drops.
Zero publisher count indicates a local topic/domain/discovery problem; publishers
with no incoming data need QoS/driver investigation. Source data with receiver
timeouts indicates a peer/port/network issue. Stale drops indicate exceeded age
budget, not successful real-time delivery.

Verify scan first with BEST_EFFORT subscribers, then RGB/depth together:

```bash
ros2 topic echo /remote/scan --once --qos-reliability best_effort
ros2 topic hz /remote/head_rgbd_sensor/depth_registered/image_rect_raw
```

Use RViz BEST_EFFORT reliability for gateway outputs. Rates are ceilings, not
promises: source rate, codec time and request round trip limit throughput. Reduce
configured camera rates when the network cannot sustain fresh delivery. Measure
bandwidth, source CPU, frame age and paired RGB-D rate before choosing rates.
Zstd depth may cost less CPU but send more bytes than PNG on some scenes.

## Local integration test

```bash
source /opt/ros/humble/setup.bash
ROS_DOMAIN_ID=73 python3 robot_transport/gateway/test_gateway.py
```

Exercises actual ROS and ZeroMQ on localhost, RGB/depth/scan/calibration,
bit-exact integer depth, reverse direction, source gateway restart, and stale
source behaviour. Hardware Wi-Fi, sustained high-resolution throughput, clock
alignment and robot-specific discovery still require real-robot verification.
