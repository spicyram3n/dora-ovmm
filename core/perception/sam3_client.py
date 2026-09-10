"""Client for the SAM3 detection server. See docker/sam3/app.py for the query/reply protocol."""

import cv2
import numpy as np
from utils.zenoh_rpc import query


class ObjectNotFound(RuntimeError):
    """The detector replied successfully but found no matching instance."""


def detect(image_bgr, prompt, conf=0.5, timeout=30):
    """Segment `prompt` in `image_bgr` (bgr8) and return the highest-scoring instance as (mask, score)."""
    _, jpeg = cv2.imencode(".jpg", image_bgr)
    meta, body = query(
        f"sam3/detect?prompt={prompt};conf={conf}", jpeg.tobytes(), timeout
    )
    count, height, width = (meta["num_instances"], meta["height"], meta["width"])
    if count == 0:
        raise ObjectNotFound(f"SAM3 found no instance of '{prompt}'")
    masks_end = count * height * width
    boxes_end = masks_end + count * 4 * 4
    masks = np.frombuffer(body[:masks_end], dtype=bool).reshape(count, height, width)
    scores = np.frombuffer(body[boxes_end:], dtype=np.float32)
    best = int(np.argmax(scores))
    mask = cv2.resize(
        masks[best].astype(np.uint8),
        image_bgr.shape[1::-1],
        interpolation=cv2.INTER_NEAREST,
    )
    return (mask.astype(bool), float(scores[best]))
