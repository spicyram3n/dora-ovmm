# utils

Shared plumbing. One file.

## Files

| File | Purpose |
|---|---|
| `zenoh_rpc.py` | One query out, one reply back, for the SAM3 and GraspGenX Docker servers |

## Running

Not a program. Imported by `perception/sam3_client.py` and
`grasping/graspgenx_client.py`, which are the two things that talk to a
container.

```python
from utils.zenoh_rpc import query
meta, body = query("sam3/detect?prompt=cup;conf=0.5", jpeg_bytes, timeout=30)
```

Both servers speak the same protocol: a JSON metadata attachment plus a raw
bytes payload.

## Things worth knowing

- **zenoh, not ROS topics**, so SAM3 and GraspGenX can keep their own CUDA and
  Python versions, separate from the ROS 2 workspace. This is independent of
  whichever RMW ROS 2 is using.
- **No router process is needed.** Each server opens a plain zenoh session on
  a fixed TCP port and starts listening.
- **Multicast scouting is off** on both servers, because it binds one shared
  host-wide UDP port and a second `--net=host` container then fails to start.
  That is why the client connects explicitly.
- **`ZENOH_CONNECT`** defaults to `tcp/127.0.0.1:2002,tcp/127.0.0.1:2003`,
  which is right when the servers run on the same machine. Set it to the PC's
  real address when the robot is talking to a separate PC.
