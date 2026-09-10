# Vision Transport

Originally developed by my colleague **Sagar**, adapted for this project's HSR/HSRC setup.

**Robot camera → TX compresses → Zenoh connection → PC RX → ROS topics.**

## Ports at a glance

| Machine | Service | TCP port | Purpose |
| --- | --- | --- | --- |
| Robot | Sagar's shared TX | `7447`* | Reuse the existing camera transmitter |
| Robot | This repository's separate TX | `7450` | Use only if you need your own transmitter |
| Your PC | RX | `7451` | Local receiver listener |
| Your PC | SAM3 | `7447` | Object detection |
| Your PC | GraspGenX | `7448` | Grasp generation |

*Shared TX port comes from the previous team notes; confirm its running configuration. The same port on different machines does not conflict.*

RX connects to the robot's TX. Data returns over that connection; no port-forwarder or explicit TX-to-PC connection is needed.

## 1. Choose the transmitter

**Prefer Sagar's running TX** to avoid encoding the same camera twice.

| Setting | Shared TX | Separate TX (current files) |
| --- | --- | --- |
| RX `connect.endpoints` | `tcp/10.7.3.185:7447`* | `tcp/10.7.3.185:7450` |
| RX `zenoh_key` prefix | `robot/head_rgbd_sensor/`* | `poorna/head_rgbd_sensor/` |
| Start another TX? | No | Yes, on the robot |

Edit [zenoh_rx.json5](deployment/config/zenoh_rx.json5) for the robot IP/port and [rx.yaml](deployment/config/rx.yaml) for keys. Keep RX's `listen` port at `7451`.

*Confirm shared TX keys and protocol with Sagar's config. Match each full key, stream type, and codec support—not just the prefix. Enable only the point-cloud stream you use; the RX file currently lists both Draco and Cloudini on the same ROS output topic.*

## 2. Build on your PC

With Docker and the repository available, run from `vision-transport-poorna`:

```bash
docker compose -f deployment/compose.yaml build dev
```

Before starting, check [cyclonedds.xml](deployment/config/cyclonedds.xml): its network interface and peer address must match your machine. RX and your local ROS consumers must use the same ROS domain; Compose defaults to **5**.

## 3. Start RX

On your **PC**, from `vision-transport-poorna`:

```bash
docker compose -f deployment/compose.yaml up -d vision_rx
docker compose -f deployment/compose.yaml logs -f vision_rx
```

Startup builds the ROS package automatically. Ctrl+C exits log viewing; RX keeps running.

Only for a **separate TX**, build the same image on the **robot**, check its DDS settings and camera topics in [tx.yaml](deployment/config/tx.yaml), then run there:

```bash
docker compose -f deployment/compose.yaml up -d vision_tx
```

## 4. View the data

In a sourced ROS terminal on your PC, use the RX's domain:

```bash
export ROS_DOMAIN_ID=5
ros2 topic hz /remote/head_rgbd_sensor/rgb/image_rect_color
```

**A ROS subscriber starts transmission.** You can also select the output topic in RViz.

| Stream | RX ROS output |
| --- | --- |
| RGB | `/remote/head_rgbd_sensor/rgb/image_rect_color` |
| Depth | `/remote/head_rgbd_sensor/depth_registered/image_rect_raw` |
| Point cloud | `/remote/head_rgbd_sensor/depth_registered/rectified_points` |

**Pipeline integration:** the current camera code reads `/head_rgbd_sensor/...`, so align those input names with RX's `/remote/...` outputs. CameraInfo and TF also need to reach the PC; this configuration transports images/depth/clouds only. The current simulation launcher is not a hardware bringup command.

## Restart or stop

After editing configuration or changing the ROS domain:

```bash
docker compose -f deployment/compose.yaml up -d --force-recreate vision_rx
```

Stop RX:

```bash
docker compose -f deployment/compose.yaml stop vision_rx
```

No images? Check **robot IP/port → matching keys → local ROS domain/DDS interface → active ROS subscriber**. Logs report `received`, `published`, and `errors`.
