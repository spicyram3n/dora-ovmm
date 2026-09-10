"""Shared zenoh plumbing for the SAM3 and GraspGenX clients."""

import json
import os
import zenoh

_CONNECT_ENDPOINTS = []
for e in os.environ.get("ZENOH_CONNECT", "").split(","):
    if e:
        _CONNECT_ENDPOINTS.append(e)


def query(selector, payload, timeout):
    """Return JSON metadata and binary payload from the first reply."""
    config = zenoh.Config()
    config.insert_json5("transport/shared_memory/enabled", "false")
    if _CONNECT_ENDPOINTS:
        config.insert_json5("connect/endpoints", json.dumps(_CONNECT_ENDPOINTS))
    with zenoh.open(config) as session:
        for reply in session.get(selector, payload=payload, timeout=timeout):
            if not reply.ok:
                raise RuntimeError(
                    f"{selector} failed: {reply.err.payload.to_bytes().decode()}"
                )
            return (
                json.loads(reply.ok.attachment.to_bytes()),
                reply.ok.payload.to_bytes(),
            )
    raise RuntimeError(f"{selector}: no reply")
