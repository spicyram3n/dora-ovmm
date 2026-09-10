# poorna instance

Private clone of `vision-transport` so the shared team gateway can keep
running untouched. Nothing here writes to the team's containers, images,
volumes, ports or Zenoh keys.

Upstream is unchanged at `../vision-transport` (branch `main`). This clone
is on branch `poorna` and shares the same `origin`
(`git@github.com:spicyram3n/test-vision.git`), so `git fetch origin` then
`git rebase origin/main` pulls colleague changes in.

## What differs from upstream

| | team (shared) | poorna |
|---|---|---|
| compose project | `deployment` | `poorna-vision` |
| containers | `deployment-vision_tx-1` | `poorna-vision-vision_tx-1` |
| docker image | `registry.gitlab.uni-bonn.de:5050/.../vision_transport:latest` | `vision_transport:poorna` (built locally) |
| colcon volumes | `deployment_colcon_*` | `poorna-vision_colcon_*` |
| Zenoh TX listen | `7447` | `7450` |
| Zenoh RX listen | unset (ephemeral) | `7451` |
| Zenoh keys | `robot/head_rgbd_sensor/*` | `poorna/head_rgbd_sensor/*` |
| DDS interface | `enp0s31f6` | `enp5s0` (this PC) |
| DDS discovery | multicast off, peer `localhost` | multicast on, peer `131.220.7.198` |

Separate Zenoh keys are what actually guarantee isolation: even if both
gateways are up, neither can see the other's streams.

## Ports on the PC

`7447` SAM3, `7448` GraspGenX, `7450` poorna TX, `7451` poorna RX.

RX pins its listen port because a peer-mode Zenoh session with no explicit
`listen` picks its own port and can take `7447` from SAM3.

## ROS topics are NOT renamed

RX still publishes `/remote/head_rgbd_sensor/...`, same as the team gateway,
because the perception code consumes those names. That is fine while your RX
and a colleague's RX are on different machines. If two RX instances ever run
on the same host and ROS domain, both publish those topics and consumers get
interleaved frames from two decoders with no error. Check with:

    ros2 topic info /remote/head_rgbd_sensor/rgb/image_rect_color --verbose

Expect exactly one publisher.

## Run

TX on the robot (see below for first-time deploy), RX on this PC:

    cd ~/dora-ovmm/vision-transport-poorna
    docker compose -f deployment/compose.yaml up -d vision_rx

The PC runs on domain 5, matching the robot, and compose already defaults
to 5. Nothing needs to be exported.

`docker compose restart` reuses the environment a container was CREATED
with. After changing anything environment-related, use
`up -d --force-recreate` instead.

First run builds `vision_transport:poorna` locally:

    docker compose -f deployment/compose.yaml build dev

## Cost of running a second TX

The team TX already uses ~85% of a core on the robot encoding the same
camera. A second TX subscribes to the same topics and encodes them again,
roughly doubling that. If you do not need to change TX settings, delete
the `vision_tx` service from your compose file, point `zenoh_rx.json5` at
`tcp/10.7.3.185:7447`, and revert the keys in `rx.yaml` to `robot/...` --
several RX clients can share one TX.

## DDS discovery on the PC

Upstream pinned the interface to one NIC with multicast off, then listed
only `localhost` as a unicast peer. Those contradict: with the interface
pinned, participants advertise on the NIC address and never listen on
loopback, so nothing could discover RX. This copy enables multicast on
`enp5s0` and points the unicast peer at this host's own address.

That matters on this PC specifically because `docker0` (172.17.0.1) and
`br-24c2f3e55113` (172.18.0.1) sit alongside `enp5s0`. CycloneDDS with
stock defaults auto-selects among them and often picks a docker bridge,
which never meets a node pinned to `enp5s0`.

Verify what a container actually loaded, rather than what the file says:

    docker exec poorna-vision-vision_rx-1 sh -c \
      'grep -iE "allowmulticast|networkinterface|peer" /tmp/cdds.log.*'

A bind-mounted config file updates live, but CycloneDDS reads it once at
process start.
