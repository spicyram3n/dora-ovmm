"""
Zenoh client for the SAM3 detection server. See docker/sam3/app.py's module
docstring for the exact query/reply protocol this follows.
"""

import json

import cv2
import numpy as np
import zenoh


def detect(image_bgr, prompt, conf=0.5, timeout=30):
    """Query SAM3 for `prompt` in `image_bgr` (H, W, 3 bgr8).

    Returns (masks, boxes, scores):
        masks:  (N, h, w) bool. h, w may differ from image_bgr's own size,
                since SAM3 resizes internally before segmenting.
        boxes:  (N, 4) float32, xyxy in that same (h, w) resolution.
        scores: (N,) float32.
    """
    _, encoded = cv2.imencode(".jpg", image_bgr)
    payload = encoded.tobytes()

    cfg = zenoh.Config()
    cfg.insert_json5("transport/shared_memory/enabled", "false")

    with zenoh.open(cfg) as session:
        # zenoh selectors separate parameters with ';', not '&' like HTTP
        # query strings. An '&' here would end up glued onto prompt's value.
        query = f"sam3/detect?prompt={prompt};conf={conf}"
        for reply in session.get(query, payload=payload, timeout=timeout):
            if not reply.ok:
                raise RuntimeError(f"sam3/detect failed: {reply.err.payload.to_bytes().decode()}")

            meta = json.loads(reply.ok.attachment.to_bytes())
            n, h, w = meta["num_instances"], meta["height"], meta["width"]
            body = reply.ok.payload.to_bytes()

            mask_bytes = n * h * w
            box_bytes = n * 4 * 4
            masks = np.frombuffer(body[:mask_bytes], dtype=bool).reshape(n, h, w)
            boxes = np.frombuffer(body[mask_bytes:mask_bytes + box_bytes], dtype=np.float32).reshape(n, 4)
            scores = np.frombuffer(body[mask_bytes + box_bytes:], dtype=np.float32)
            return masks, boxes, scores

    raise RuntimeError("sam3/detect: no reply")


def best_instance(masks, scores):
    """Pick the highest-scoring detected instance out of detect()'s output."""
    i = int(np.argmax(scores))
    return masks[i], float(scores[i])
