"""Client for the SAM3 detection server. See docker/sam3/app.py for the query/reply protocol."""

import cv2
import numpy as np
from core.utils.zenoh_rpc import query, parameter


class ObjectNotFound(RuntimeError):
    """The detector replied successfully but found no matching instance."""


def detect_all(image_bgr, prompt, conf=0.5, timeout=30, *, metadata=None):
    """Return masks, xyxy boxes, scores, and echoed observation metadata.

    Empty results are valid. Indices are local to this image, not tracking IDs.
    Caller-supplied object_id identifies the requested target, not every mask.
    """
    ok, jpeg = cv2.imencode(".jpg", image_bgr)
    if not ok:
        raise ValueError("Could not encode image")
    meta, body = query(
        f"sam3/detect?prompt={parameter(prompt)};conf={conf}", jpeg.tobytes(), timeout,
        metadata=metadata,
    )
    count, height, width = (meta["num_instances"], meta["height"], meta["width"])
    masks_end = count * height * width
    boxes_end = masks_end + count * 4 * 4
    if len(body) != boxes_end + count * 4:
        raise RuntimeError("Malformed SAM3 reply")
    masks = np.frombuffer(body[:masks_end], dtype=bool).reshape(count, height, width)
    boxes = np.frombuffer(body[masks_end:boxes_end], dtype=np.float32).reshape(count, 4).copy()
    scores = np.frombuffer(body[boxes_end:], dtype=np.float32)
    target_h, target_w = image_bgr.shape[:2]
    if (height, width) != (target_h, target_w):
        masks = np.asarray([
            cv2.resize(m.astype(np.uint8), (target_w, target_h), interpolation=cv2.INTER_NEAREST)
            for m in masks
        ], dtype=bool).reshape(count, target_h, target_w)
        boxes *= np.array([target_w / width, target_h / height] * 2)
    return {"masks": masks, "boxes": boxes, "scores": scores, "metadata": meta}


def detect(image_bgr, prompt, conf=0.5, timeout=30, *, metadata=None):
    """Compatibility API: return the highest-scoring instance as (mask, score)."""
    result = detect_all(image_bgr, prompt, conf, timeout, metadata=metadata)
    if not len(result["scores"]):
        raise ObjectNotFound(f"SAM3 found no instance of '{prompt}'")
    best = int(np.argmax(result["scores"]))
    return result["masks"][best], float(result["scores"][best])
