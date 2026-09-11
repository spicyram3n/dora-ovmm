"""Shared Zenoh clients and small, serialized model-server runner."""

import atexit
import json
import math
import os
import queue
import threading
import time
import uuid
from urllib.parse import quote, unquote

import zenoh

_session = None
_session_lock = threading.Lock()


def close():
    """Close the process's shared client session after callers have stopped."""
    global _session
    with _session_lock:
        if _session is not None:
            _session.close()
            _session = None


atexit.register(close)


def query(selector, payload, timeout, *, metadata=None):
    """Return metadata and bytes; reuse one session across calls and threads.

    metadata may contain frame_id, stamp (ROS sec/nanosec), and object_id.
    These describe the input; the server echoes them without interpreting TF.
    """
    global _session
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError("timeout must be finite and positive")
    context = dict(metadata or {})
    context.update(request_id=uuid.uuid4().hex, timeout_s=timeout)
    attachment = json.dumps(context).encode()
    with _session_lock:
        if _session is None:
            config = zenoh.Config()
            config.insert_json5("transport/shared_memory/enabled", "false")
            config.insert_json5("scouting/multicast/enabled", "false")
            endpoints = [e for e in os.environ.get(
                "ZENOH_CONNECT", "tcp/127.0.0.1:7447,tcp/127.0.0.1:7448"
            ).split(",") if e]
            if endpoints:
                config.insert_json5("connect/endpoints", json.dumps(endpoints))
            _session = zenoh.open(config)
        session = _session
    for reply in session.get(selector, payload=payload, attachment=attachment, timeout=timeout):
        if not reply.ok:
            message = reply.err.payload.to_bytes().decode()
            if message == "Timeout" or message.startswith("TIMEOUT:"):
                raise TimeoutError(f"{selector}: {message}")
            raise RuntimeError(f"{selector}: {message}")
        meta = json.loads(reply.ok.attachment.to_bytes())
        if meta.get("request_id") != context["request_id"]:
            raise RuntimeError("Model reply has missing or mismatched request_id; rebuild model images")
        return meta, reply.ok.payload.to_bytes()
    raise TimeoutError(f"{selector}: no reply within {timeout:g}s (server unavailable or deadline exceeded)")


def parameter(value):
    """Encode text used in Zenoh selector parameters."""
    return quote(str(value), safe="")


def serve(key, handler, endpoint):
    """One inference worker, a bounded waiting queue, and a readiness endpoint.

    handler(query) returns (metadata, bytes). GPU work already running cannot
    be cancelled; expired results are discarded and expired queued work skipped.
    """
    capacity = int(os.environ.get("MODEL_QUEUE_SIZE", "2"))
    if capacity < 1:
        raise ValueError("MODEL_QUEUE_SIZE must be at least 1")
    pending = queue.Queue(maxsize=capacity)

    def error(request, message):
        try:
            request.reply_err(message.encode())
        except Exception:
            pass  # The caller may already have timed out.

    def enqueue(request):
        try:
            context = json.loads(request.attachment.to_bytes()) if request.attachment else {}
            timeout = float(context.get("timeout_s", 30))
            if not math.isfinite(timeout) or timeout <= 0:
                raise ValueError("invalid timeout_s")
            pending.put_nowait((request, context, time.monotonic() + timeout))
        except queue.Full:
            error(request, "BUSY: model queue is full; retry later")
        except Exception as exc:
            error(request, f"INVALID_REQUEST: {exc}")

    def worker():
        while True:
            request, context, deadline = pending.get()
            try:
                if time.monotonic() >= deadline:
                    error(request, "TIMEOUT: expired before inference")
                    continue
                meta, body = handler(request)
                if time.monotonic() >= deadline:
                    error(request, "TIMEOUT: inference exceeded deadline")
                    continue
                meta.update({k: context[k] for k in ("request_id", "frame_id", "stamp", "object_id") if k in context})
                request.reply(request.key_expr, payload=body, attachment=json.dumps(meta).encode())
            except Exception as exc:
                error(request, f"INFERENCE_ERROR: {type(exc).__name__}: {exc}")
            finally:
                pending.task_done()
                del request  # Release Zenoh query so the reply stream completes.

    threading.Thread(target=worker, daemon=True).start()
    config = zenoh.Config()
    config.insert_json5("transport/shared_memory/enabled", "false")
    # Host-networked models use dedicated TCP listeners; multicast scouting
    # would also bind UDP 7446 and can collide with other host services.
    config.insert_json5("scouting/multicast/enabled", "false")
    config.insert_json5("listen/endpoints", json.dumps([endpoint]))
    with zenoh.open(config) as session:
        inference = session.declare_queryable(key, enqueue)
        health = session.declare_queryable(key.split("/")[0] + "/health", lambda q: q.reply(q.key_expr, payload=b"ready"))
        print(f"[{key}] ready", flush=True)
        while True:
            time.sleep(1)


if __name__ == "__main__":
    # Container healthcheck: model main() registers this only after warmup.
    import sys
    config = zenoh.Config()
    config.insert_json5("connect/endpoints", json.dumps([sys.argv[1]]))
    config.insert_json5("scouting/multicast/enabled", "false")
    with zenoh.open(config) as session:
        for reply in session.get(sys.argv[2] + "/health", timeout=3):
            if reply.ok and reply.ok.payload.to_bytes() == b"ready":
                sys.exit(0)
    sys.exit(1)
