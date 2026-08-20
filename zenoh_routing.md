# Zenoh routing: robot to workstation

Route heavy sensor topics from the HSR (`10.7.3.185`, user `administrator`) to
this workstation (`131.220.7.198`, hostname `mrtom`) over zenoh instead of DDS.

## 1. Why compression, not just zenoh

A raw `PointCloud2` from the head RGBD sensor is about 9.8 MB per message
(640x480 x 32 bytes). At 30 Hz that is 295 MB/s, roughly 2.4 Gbps. The link is
1 Gbps, so no transport can carry it. Zenoh does not help by itself because
`zenoh-bridge-ros2dds` forwards messages byte for byte.

The fix is to send less data:

| Topic | Raw | Compressed |
|---|---|---|
| rgb image | ~900 KB | ~100 KB (JPEG) |
| depth image | ~600 KB | ~200 KB (PNG) |
| pointcloud | ~9.8 MB | not sent |

We bridge only the compressed image topics that the robot already publishes
through `image_transport`. The raw pointcloud is not sent. rviz displays
compressed images directly, so nothing needs decompressing by hand.

## 2. Network fix (recurring)

The robot loses its route back to this subnet. Symptom: ping to `10.7.3.185`
fails while the gateway `10.7.3.254` answers.

Cause: the robot has several default routes and the lowest metric wins. A
teammate's TP-Link USB ethernet adapter installs `default via 192.168.1.1` at
metric 20101, which beats `default via 10.7.3.254` at 20103. A `usb0`
interface holding `131.220.7.220` caused the same problem earlier.

Run on the robot:

```bash
sudo ip route add 131.220.7.0/24 via 10.7.3.254 dev enp0s31f6
ping -c 3 131.220.7.198
```

Make it survive reboots (`enp0s31f6` is the connection named `Internet`):

```bash
sudo nmcli connection modify "Internet" +ipv4.routes "131.220.7.0/24 10.7.3.254"
sudo nmcli connection up "Internet"
```

## 3. Install

Robot already has `zenoh-bridge-ros2dds` v1.9.0 via apt. On this PC:

```bash
curl -sL -o bridge.zip "https://github.com/eclipse-zenoh/zenoh-plugin-ros2dds/releases/download/1.9.0/zenoh-plugin-ros2dds-1.9.0-x86_64-unknown-linux-gnu-debian.zip"
unzip -o bridge.zip
sudo dpkg -i zenoh-bridge-ros2dds_1.9.0_amd64.deb
zenoh-bridge-ros2dds --version
```

## 4. Port and domain layout

| What | Value | Reason |
|---|---|---|
| zenoh port | 7449 | 7447 is teammate's `vision_transport_tx` and SAM3, 7448 is GraspGenX |
| multicast scouting | disabled | UDP 7446 also taken |
| robot DDS domain | 0 | where the real sensors publish |
| PC bridge DDS domain | 1 | isolated, so anything arriving came via zenoh |

`ROS_DOMAIN_ID` in the environment overrides the `domain` field in the config
file, so always export it explicitly.

## 4b. Where the configs live

Both bridge configs are versioned in this repo so the two sides stay in sync.

| File | Used by |
|---|---|
| `.devcontainer/zenoh/bridge_pc.json5` | this PC, run in place |
| `.devcontainer/zenoh/bridge_robot.json5` | copy to robot `/etc/zenoh-bridge-ros2dds/` |
| `.devcontainer/zenoh/cdds_bridge_robot.xml` | copy to robot as `cdds_bridge.xml` |
| `.devcontainer/cyclonedds_profile.xml` | this PC, exported by `devcontainer.json` |

`.devcontainer/zenoh/session_shm.json5` is unrelated. It is an `rmw_zenoh_cpp`
session config, not a bridge config.

```bash
scp .devcontainer/zenoh/bridge_robot.json5 \
    .devcontainer/zenoh/cdds_bridge_robot.xml \
    administrator@10.7.3.185:/tmp/
```

## 5. Robot side config

`/etc/zenoh-bridge-ros2dds/cdds_bridge.xml`, a minimal CycloneDDS profile. The
robot's full profile crashes the bridge with a glibc buffer overflow, and
running with no profile at all makes the bridge unable to discover any ROS
node.

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="enp0s31f6" autodetermine="false" multicast="false"/>
      </Interfaces>
      <AllowMulticast>false</AllowMulticast>
      <EnableMulticastLoopback>false</EnableMulticastLoopback>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>99</MaxAutoParticipantIndex>
      <Peers>
        <Peer Address="localhost"/>
        <Peer Address="10.7.3.185"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
```

`/etc/zenoh-bridge-ros2dds/bridge_robot.json5`:

```json5
{
  plugins: {
    ros2dds: {
      domain: 0,
      allow: {
        subscribers: [
          "/scan",
          "/tf",
          "/tf_static",
          "/robot_description",
          "/head_rgbd_sensor/rgb/camera_info",
          "/head_rgbd_sensor/rgb/image_rect_color/compressed",
          "/head_rgbd_sensor/depth_registered/camera_info",
          "/head_rgbd_sensor/depth_registered/image_rect_raw/compressedDepth"
        ],
        publishers: [
          "/scan",
          "/tf",
          "/tf_static",
          "/robot_description",
          "/head_rgbd_sensor/rgb/camera_info",
          "/head_rgbd_sensor/rgb/image_rect_color/compressed",
          "/head_rgbd_sensor/depth_registered/camera_info",
          "/head_rgbd_sensor/depth_registered/image_rect_raw/compressedDepth"
        ]
      }
    }
  },
  transport: { shared_memory: { enabled: false } },
  scouting: { multicast: { enabled: false } },
  listen: { endpoints: ["tcp/0.0.0.0:7449"] }
}
```

Patterns are anchored with a leading `/`. Using `.*/head_rgbd_sensor/...` also
matches the teammate's `/remote/head_rgbd_sensor/...` topics and doubles the
traffic.

Allow lists must be identical on both sides.

## 6. PC side config

`/home/ws/.devcontainer/zenoh/bridge_pc.json5` is the same allow list with
`domain: 1`, `mode: "client"`, and:

```json5
connect: {
  endpoints: ["tcp/10.7.3.185:7449"],
  timeout_ms: -1,
  exit_on_failure: false,
  retry: { period_init_ms: 1000, period_max_ms: 4000, period_increase_factor: 2 }
}
```

`exit_on_failure: false` matters. In client mode the bridge exits on the first
failed connect, and this link drops often.

`.devcontainer/cyclonedds_profile.xml` needs the robot as a peer plus large
sample handling in an `<Internal>` block (32MB socket buffers, 64 defrag
samples).

## 6b. Explicit numbers

Two separate places control the data volume.

**Rate**, in `bridge_robot.json5`. Downsampling happens while routing, so the
robot's own nodes keep running at full rate and only the zenoh link is capped.

```json5
pub_max_frequencies: [
  "/head_rgbd_sensor/rgb/image_rect_color/compressed=15.0",
  "/head_rgbd_sensor/depth_registered/image_rect_raw/compressedDepth=15.0"
]
```

Budget at 15 Hz is about 4.5 MB/s, roughly 36 Mbps. 30 Hz is about 72 Mbps and
still fits on a 1 Gbps link. `/scan` and `/tf` are left uncapped.

**Quality**, set on the robot's `image_transport` publisher. These are ROS
parameters on whichever node publishes the topic, not bridge settings. Find
them first, since the prefix depends on the node:

```bash
# on the robot
ros2 node list | grep head_rgbd
ros2 param list /head_rgbd_sensor/openni2_driver | grep -i compress
```

Then set, matching the other team's values:

```bash
ros2 param set /head_rgbd_sensor/openni2_driver \
  <prefix>.compressed.jpeg_quality 80
ros2 param set /head_rgbd_sensor/openni2_driver \
  <prefix>.compressedDepth.png_level 3
```

Defaults are jpeg quality 95 and png level 9. Quality 80 is a large size drop
for little visible loss, which is why they picked it.

These are runtime parameters and reset when the node restarts. Put them in the
robot's launch config to make them stick.

## 7. Start order

Robot first, it is the listener.

```bash
# robot
export CYCLONEDDS_URI=file:///etc/zenoh-bridge-ros2dds/cdds_bridge.xml
zenoh-bridge-ros2dds -c /etc/zenoh-bridge-ros2dds/bridge_robot.json5
```

```bash
# PC
export CYCLONEDDS_URI=file:///home/ws/.devcontainer/cyclonedds_profile.xml
export ROS_DOMAIN_ID=1
zenoh-bridge-ros2dds -c /home/ws/.devcontainer/zenoh/bridge_pc.json5
```

Both bridges are foreground processes. They must stay running.

## 7b. Getting a PointCloud2 back

Only compressed images cross the link, so the cloud is rebuilt here. A depth
image plus `camera_info` holds the same information as a pointcloud, and
`depth_image_proc` projects it back. This is the same node the robot runs on
board as `/head_rgbd_sensor/points_xyzrgb`, just moved to this machine.

```bash
export CYCLONEDDS_URI=file:///home/ws/.devcontainer/cyclonedds_profile.xml
export ROS_DOMAIN_ID=1
ros2 launch /home/ws/zenoh/pointcloud_reconstruct.launch.py
```

Flow:

```
robot  depth + rgb  ->  compressed  ->  zenoh  ->  decompress  ->  PointCloud2
                                                on this PC
```

It publishes `/head_rgbd_sensor/depth_registered/rectified_points`, the same
name the robot uses, so rviz and any other consumer need no changes.

Skip this launch file if you only want the camera image and the laser scan.

## 8. Verify

```bash
export CYCLONEDDS_URI=file:///home/ws/.devcontainer/cyclonedds_profile.xml
export ROS_DOMAIN_ID=1
ros2 daemon stop
ros2 topic list
ros2 topic hz /scan
ros2 topic hz /head_rgbd_sensor/rgb/image_rect_color/compressed
ros2 topic bw /head_rgbd_sensor/rgb/image_rect_color/compressed
ros2 run tf2_ros tf2_echo odom base_range_sensor_link
```

Anything on domain 1 arrived over zenoh, since DDS cannot cross domains.

In rviz set Fixed Frame to `odom`, not `map`. The `map` frame only exists if a
localizer is publishing `map` to `odom`.

For the camera, add an Image display, set the topic to
`/head_rgbd_sensor/rgb/image_rect_color` and set Transport Hint to
`compressed`. rviz then subscribes to the `/compressed` topic directly.

## 9. Useful checks

```bash
# is the robot bridge up
timeout 3 bash -c "cat </dev/null >/dev/tcp/10.7.3.185/7449" && echo OPEN

# who holds a port on the robot
sudo ss -ltnp | grep 7447

# is a bridge running
pgrep -a zenoh-bridge-ros2dds

# does the robot publish at all, run on the robot
timeout 10 ros2 topic hz /head_rgbd_sensor/depth_registered/rectified_points

# clock skew, zenoh rejects samples more than 500ms apart
date -u
```

## 10. Things that went wrong

| Symptom | Cause |
|---|---|
| `Address already in use` 7447 | teammate's `vision_transport_tx` |
| `Unable to bind UDP 224.0.0.224:7446` | multicast scouting, disable it |
| `unknown field shared_memory` | it lives under `transport`, not top level |
| topics named `/zenoh/zenoh/zenoh/...` | `namespace` set on one bridge only, causes an infinite bridging loop |
| bridge on domain 0 despite `domain: 1` | `ROS_DOMAIN_ID` env overrides the config |
| bridge sees no ROS nodes | ran with `env -u CYCLONEDDS_URI`, no discovery config |
| glibc buffer overflow | the robot's full CycloneDDS profile, use the minimal one |
| topics listed but no data | stale `ros2 daemon`, run `ros2 daemon stop` |
| everything slow | pointcloud saturating the link |

## 11. Teammate's setup

`vision_transport_tx` runs on the robot from
`registry.gitlab.uni-bonn.de:5050/s52sgupt/ovmm_images/vision_transport:latest`
and holds port 7447. Its `/config/tx.yaml` compresses rgb to JPEG quality 80,
depth to PNG, and pointclouds with the cloudini codec at 10 Hz. It is a
separate pipeline and does not conflict with this setup on 7449.
