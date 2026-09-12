# Vision Transport

Originally built by my colleague **Sagar**, adapted for this project's HSR/HSRC setup.

**Robot camera → TX compresses → Zenoh → PC RX → ROS topics.**

## At a glance

| # | Where | Command | Why |
| --- | --- | --- | --- |
| 1 | PC | Edit `deployment/config/zenoh_rx.json5` and `rx.yaml` | Point RX at the right transmitter |
| 2 | PC, `vision-transport-poorna/` | `docker compose -f deployment/compose.yaml build dev` | Build the image |
| 3 | PC | `docker compose -f deployment/compose.yaml up -d vision_rx` | Start the receiver |
| 4 | PC, sourced | `ROS_DOMAIN_ID=5 ros2 topic hz /remote/head_rgbd_sensor/rgb/image_rect_color` | Check images arrive |

## Ports

| Machine | Service | TCP port | Purpose |
| --- | --- | --- | --- |
| Robot | Sagar's shared TX | `7447`* | Reuse the existing camera transmitter |
| Robot | This repo's separate TX | `7450` | Only if you need your own transmitter |
| Your PC | RX | `7451` | Local receiver listener |
| Your PC | SAM3 | `7447` | Object detection |
| Your PC | GraspGenX | `7448` | Grasp generation |

*From the previous team notes; confirm against its running config. The same port on different machines does not clash.*

RX connects to the robot's TX, and data comes back over that connection. No port forwarding needed.

---

## 1. Choose the transmitter

**Prefer Sagar's running TX**, so the camera is not encoded twice.

| Setting | Shared TX | Separate TX (current files) |
| --- | --- | --- |
| RX `connect.endpoints` | `tcp/10.7.3.185:7447`* | `tcp/10.7.3.185:7450` |
| RX `zenoh_key` prefix | `robot/head_rgbd_sensor/`* | `poorna/head_rgbd_sensor/` |
| Start another TX? | No | Yes, on the robot |

- **Robot IP and port:** set them in [zenoh_rx.json5](deployment/config/zenoh_rx.json5).
- **Keys:** set them in [rx.yaml](deployment/config/rx.yaml).
- **RX `listen` port:** keep it at `7451`.

⚠️ *Confirm the shared TX keys with Sagar's config: each full key, stream type and codec must match, not just the prefix. Enable only the point-cloud stream you use; `rx.yaml` currently lists Draco and Cloudini on the same ROS topic.*

## 2. Build on your PC

From `vision-transport-poorna/`:

```bash
docker compose -f deployment/compose.yaml build dev
```

Before starting, check [cyclonedds.xml](deployment/config/cyclonedds.xml): its network interface and peer address must match your machine. RX and your ROS tools must share a ROS domain; Compose defaults to **5**.

## 3. Start RX

On your **PC**, from `vision-transport-poorna/`:

| Command | Why |
| --- | --- |
| `docker compose -f deployment/compose.yaml up -d vision_rx` | Start the receiver (builds the ROS package on first start) |
| `docker compose -f deployment/compose.yaml logs -f vision_rx` | Watch it. Ctrl+C leaves it running. |

**Separate TX only:**

1. Build the same image on the **robot**.
2. Check its DDS settings and camera topics in [tx.yaml](deployment/config/tx.yaml).
3. On the robot, run `docker compose -f deployment/compose.yaml up -d vision_tx`.

## 4. See the data

In a sourced ROS terminal on your PC:

```bash
export ROS_DOMAIN_ID=5
ros2 topic hz /remote/head_rgbd_sensor/rgb/image_rect_color
```

**Nothing is sent until something subscribes.** RViz works too.

| Stream | RX ROS output |
| --- | --- |
| RGB | `/remote/head_rgbd_sensor/rgb/image_rect_color` |
| Depth | `/remote/head_rgbd_sensor/depth_registered/image_rect_raw` |
| Point cloud | `/remote/head_rgbd_sensor/depth_registered/rectified_points` |

⚠️ **Not wired into the pipeline yet:**

- **Topic names:** the camera code reads `/head_rgbd_sensor/...`, so align it with RX's `/remote/...` outputs.
- **CameraInfo and TF** must also reach the PC; this config carries images, depth and clouds only.
- **Hardware:** the sim launch is not a hardware bringup.

## Restart or stop

| Command | When |
| --- | --- |
| `docker compose -f deployment/compose.yaml up -d --force-recreate vision_rx` | After editing config or changing the ROS domain |
| `docker compose -f deployment/compose.yaml stop vision_rx` | Stop RX |

**No images?** Check, in order: robot IP/port → matching keys → local ROS domain and DDS interface → an active subscriber. The logs report `received`, `published` and `errors`.
