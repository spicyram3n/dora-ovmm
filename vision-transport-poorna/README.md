# Vision Transport

Originally developed by my colleague Poorna and adapted for this project’s HSR/HSRC setup. This directory is maintained as part of the main repository.

Send ROS 2 images, depth, and point clouds over Zenoh. TX compresses data; RX restores ROS messages. The host only needs Docker.

## 1. Build

Run from the `vision-transport-poorna` directory:

```bash
docker compose -f deployment/compose.yaml build dev
docker compose -f deployment/compose.yaml run --rm dev \
  colcon build --symlink-install
```

## 2. Configure

Edit [tx.yaml](deployment/config/tx.yaml) and [rx.yaml](deployment/config/rx.yaml). Match each stream's `zenoh_key` on both sides.

TX example:

```yaml
streams:
  rgb:
    type: image
    input_topic: /camera/color/image_raw
    output_topic: /remote/camera/color/image_raw
    zenoh_key: robot/front_camera/rgb
    max_rate_hz: 10.0
    queue_depth: 1
    resize_scale: 0.5
    codec:
      type: jpeg
      quality: 80
```

RX example:

```yaml
streams:
  rgb:
    type: image
    output_topic: /remote/camera/color/image_raw
    zenoh_key: robot/front_camera/rgb
    queue_depth: 1
```

For **two machines**, change the endpoint in [zenoh_rx.json5](deployment/config/zenoh_rx.json5) from `127.0.0.1` to the TX machine's IP:

```json5
connect: {
  endpoints: ["tcp/192.168.1.20:7447"],
}
```

TX listens on TCP port `7447`. Both services use host networking. Shared memory is disabled.

## 3. Start TX and RX

Run each in a separate terminal, on its respective host:

```bash
docker compose -f deployment/compose.yaml up vision_tx
```

```bash
docker compose -f deployment/compose.yaml up vision_rx
```

**Subscribe to the RX output topic to start data flow.** TX subscribes to the source only while RX has a listener. It keeps the latest pending sample and drops pending data when demand stops.

## Choose a codec

| Stream `type` | `codec.type` options |
| --- | --- |
| `image` | `jpeg`, `png`, `raw` |
| `depth` | `compressed_depth`, `raw` |
| `pointcloud` | `draco`, `cloudini`, `zstd`, `raw` (matching plugins required) |

Use `raw` to compare transmission without compression. Image/depth resizing still applies. Packed `rgb`/`rgba` point-cloud fields are protected during Cloudini compression and restored for RViz.

## Settings reference

| TX setting | Meaning |
| --- | --- |
| `max_rate_hz` | Input rate limit; `0.0` disables it |
| `queue_depth` | ROS QoS queue depth |
| `resize_scale` | Image/depth resize factor; depth uses nearest neighbor |
| `codec.quality` | JPEG quality, `1..100` |
| `codec.png_level` | PNG compression, `0..9` |

`voxel_size`, `codec.zstd_level`, and `codec.cloudini_resolution` are parsed but reserved for future tuning.

RX requires `type`, `output_topic`, and `zenoh_key`. Its `queue_depth` defaults to `1`; codec details arrive with the data.

| ROS parameter | TX default | RX default |
| --- | --- | --- |
| `config_path` | `/config/tx.yaml` | `/config/rx.yaml` |
| `zenoh_config_path` | `/config/zenoh_tx.json5` | `/config/zenoh_rx.json5` |

## Keep raw DDS off Wi-Fi

Compose sets CycloneDDS, `ROS_DOMAIN_ID=30`, and `CYCLONEDDS_URI=file:///config/cyclonedds.xml`.
Edit [cyclonedds.xml](deployment/config/cyclonedds.xml) to use the robot-local or wired interface. Check interface counters or packet capture: compression will not save wireless bandwidth if raw DDS also crosses Wi-Fi.

## Test and debug

```bash
docker compose -f deployment/compose.yaml run --rm dev \
  colcon test --event-handlers console_direct+
docker compose -f deployment/compose.yaml run --rm dev \
  colcon test-result --verbose
```

Tests cover configuration, codecs, metadata, Zenoh/ROS round trips, subscriber demand, and TX restart recovery.

Open a development shell:

```bash
docker compose -f deployment/compose.yaml run --rm dev bash
```

Run the synthetic codec benchmark:

```bash
docker compose -f deployment/compose.yaml run --rm dev bash -lc \
  'source /workspace/install/setup.bash && ros2 run vision_transport vision_transport_codec_benchmark'
```

It reports CSV sizes and encode/decode times. It does not measure Wi-Fi performance.

Check logs for `demand_active`, `published`, `errors`, and `last_envelope_bytes`. TX also reports `dropped` and `last_payload_bytes`; RX reports `received`. TX logs metrics every 5 seconds.
