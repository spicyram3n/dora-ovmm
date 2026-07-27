"""
Trigger SAM3 detection on one frame from the HSR head camera, over zenoh
(no HTTP): grab a frame from /head_rgbd_sensor/rgb/image_rect_color, send it
to the SAM3 container's "sam3/detect" queryable, save the masked result.

Usage: python3 sam3_trigger.py "<prompt>"
"""

import json
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
import zenoh
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

TOPIC = "/head_rgbd_sensor/rgb/image_rect_color"
OUT_PATH = Path(__file__).resolve().parent / "sam3_result.png"
COLORS = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255)]


def grab_frame():
    rclpy.init()
    node = Node("sam3_trigger")
    bridge = CvBridge()
    frame = {}

    def on_image(msg):
        frame["img"] = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

    node.create_subscription(Image, TOPIC, on_image, 1)
    print(f"[sam3_trigger] waiting for a frame on {TOPIC}...")
    while "img" not in frame:
        rclpy.spin_once(node)
    node.destroy_node()
    rclpy.shutdown()
    return frame["img"]


def detect(img, prompt):
    ok, encoded = cv2.imencode(".jpg", img)
    payload = encoded.tobytes()

    cfg = zenoh.Config()
    cfg.insert_json5("transport/shared_memory/enabled", "false")

    with zenoh.open(cfg) as session:
        replies = session.get(f"sam3/detect?prompt={prompt}", payload=payload, timeout=30)
        for reply in replies:
            if not reply.ok:
                print("[sam3_trigger] error:", reply.err.payload.to_bytes().decode())
                return
            meta = json.loads(reply.ok.attachment.to_bytes())
            n, h, w = meta["num_instances"], meta["height"], meta["width"]
            body = reply.ok.payload.to_bytes()

            mask_bytes = n * h * w
            box_bytes = n * 4 * 4
            masks = np.frombuffer(body[:mask_bytes], dtype=bool).reshape(n, h, w)
            boxes = np.frombuffer(body[mask_bytes:mask_bytes + box_bytes], dtype=np.float32).reshape(n, 4)
            scores = np.frombuffer(body[mask_bytes + box_bytes:], dtype=np.float32)

            print(f"[sam3_trigger] '{prompt}': {n} instance(s)")
            # SAM3 returns masks/boxes at whatever resolution it processed
            # the image at, which may not match the original frame size.
            overlay = cv2.resize(img, (w, h)) if img.shape[:2] != (h, w) else img.copy()
            for i, (mask, box, score) in enumerate(zip(masks, boxes, scores)):
                color = np.array(COLORS[i % len(COLORS)])
                overlay[mask] = (overlay[mask] * 0.5 + color * 0.5).astype(np.uint8)
                x1, y1, x2, y2 = box.astype(int)
                cv2.rectangle(overlay, (x1, y1), (x2, y2), COLORS[i % len(COLORS)], 2)
                print(f"  box={box.tolist()} score={score:.2f}")

            cv2.imwrite(OUT_PATH, overlay)
            print(f"[sam3_trigger] saved {OUT_PATH}")


if __name__ == "__main__":
    prompt = sys.argv[1] if len(sys.argv) > 1 else "object"
    detect(grab_frame(), prompt)
