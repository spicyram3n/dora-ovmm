"""Shared zenoh plumbing for the SAM3 and GraspGenX clients. Both speak the
same protocol: one query in, one reply out, carrying a JSON metadata
attachment plus a raw bytes payload."""

import json
import os

import zenoh

# Comma-separated zenoh endpoints to connect to explicitly, e.g.
# "tcp/192.168.1.50:2002,tcp/192.168.1.50:2003". SAM3/GraspGenX disable
# multicast scouting (it binds a single shared host-wide UDP port, which
# breaks when more than one zenoh session runs with --net=host), so this
# must be set to reach them. Defaults to localhost, which is correct when
# this client and the servers run on the same machine; override with the
# PC's real address when SAM3/GraspGenX run on a different host (e.g. the
# real robot talking to a PC).
_CONNECT_ENDPOINTS = [
    e for e in os.environ.get("ZENOH_CONNECT", "tcp/127.0.0.1:2002,tcp/127.0.0.1:2003").split(",") if e
]


def query(selector, payload, timeout):
    """Send `payload` to `selector` and return (meta, body) from the first
    reply: the parsed JSON attachment and the raw payload bytes."""
    config = zenoh.Config()
    config.insert_json5("transport/shared_memory/enabled", "false")
    if _CONNECT_ENDPOINTS:
        config.insert_json5("connect/endpoints", json.dumps(_CONNECT_ENDPOINTS))

    with zenoh.open(config) as session:
        for reply in session.get(selector, payload=payload, timeout=timeout):
            if not reply.ok:
                raise RuntimeError(f"{selector} failed: {reply.err.payload.to_bytes().decode()}")
            return json.loads(reply.ok.attachment.to_bytes()), reply.ok.payload.to_bytes()

    raise RuntimeError(f"{selector}: no reply")
