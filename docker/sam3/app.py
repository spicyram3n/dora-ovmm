"""
SAM3 inference server (zenoh queryable).

Wraps the official SAM3 image-inference pattern
(https://github.com/facebookresearch/sam3) over zenoh so it can run in its
own CUDA 12.6 / Python 3.12 container, separate from the ROS 2 Humble
(Python 3.10) workspace, without going through HTTP:

    model = build_sam3_image_model()
    processor = Sam3Processor(model)
    state = processor.set_image(image)
    output = processor.set_text_prompt(state=state, prompt="...")
    # output = {"masks": (N,H,W) bool, "boxes": (N,4) xyxy, "scores": (N,)}

The model is loaded and warmed up at startup (not on first request) so the
first real detection call is fast. See _startup() for details.

Zenoh queryable key: "sam3/detect"
    query payload:     jpg/png encoded image bytes
    query parameters:  prompt (text concept, e.g. "red cup")
                        conf   (optional float threshold, default 0.5)

Reply payload: raw bytes = masks.tobytes() + boxes.tobytes() + scores.tobytes()
Reply attachment: json {"num_instances": N, "height": H, "width": W}

See zenoh_examples/sam3_trigger.py for a client that calls this.
"""

import io
import json
import os
import time

import numpy as np
import torch
import zenoh
from PIL import Image

# Sam3Processor resizes every image to a fixed resolution (1008×1008 by
# default), so cuDNN can pick fixed conv algorithms once and reuse them on
# every frame instead of re-benchmarking per call.
torch.backends.cudnn.benchmark = True

MODEL = None
PROCESSOR = None


def _load_model():
    global MODEL, PROCESSOR
    from sam3.model_builder import build_sam3_image_model
    from sam3.model.sam3_image_processor import Sam3Processor

    token = os.environ.get("HF_TOKEN") or None
    if token:
        from huggingface_hub import login
        login(token=token)

    print("[sam3] Loading model weights...")
    MODEL = build_sam3_image_model()
    PROCESSOR = Sam3Processor(MODEL)
    print("[sam3] Model loaded.")


def _warmup():
    """
    Run one dummy forward pass so cuDNN benchmark selects its conv algorithms
    now, at startup, rather than on the first real request.

    cudnn.benchmark = True makes the first call to each unique input shape slow
    (~10-30s) while it times different kernel implementations. Running a warmup
    at the same resolution the server will actually receive (1008×1008 after
    Sam3Processor resizes) means every subsequent real call is fast.
    """
    print("[sam3] Warming up (cuDNN benchmark + first forward pass)...")
    dummy = Image.fromarray(np.zeros((256, 256, 3), dtype=np.uint8))
    with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
        state = PROCESSOR.set_image(dummy)
        PROCESSOR.set_text_prompt(state=state, prompt="warmup")
    torch.cuda.synchronize()
    print("[sam3] Warmup done — ready for requests.")


def _on_detect(query):
    prompt = query.parameters.get("prompt", "")
    conf = float(query.parameters.get("conf", "0.5"))
    if not prompt:
        query.reply_err(b"no prompt provided")
        return

    img = Image.open(io.BytesIO(query.payload.to_bytes())).convert("RGB")

    # SAM3's checkpoint mixes bf16 and fp32 weights across submodules.
    # autocast casts both operands of every matmul to bf16 at call time,
    # so the stored weight dtypes never need to match each other.
    with torch.inference_mode(), torch.autocast("cuda", dtype=torch.bfloat16):
        state = PROCESSOR.set_image(img)
        output = PROCESSOR.set_text_prompt(state=state, prompt=prompt)

    masks = output["masks"]  # (N, 1, H, W) bool
    boxes = output["boxes"]  # (N, 4) xyxy
    scores = output["scores"]  # (N,)

    # .float() first: autocast may return bf16 tensors, which .numpy() cannot
    # convert directly.
    masks = masks.float().cpu().numpy().astype(bool)
    boxes = boxes.float().cpu().numpy().astype(np.float32)
    scores = scores.float().cpu().numpy().astype(np.float32)

    # Drop the singleton mask-channel dim: (N, 1, H, W) → (N, H, W).
    if masks.ndim == 4:
        masks = masks[:, 0]

    keep = scores >= conf
    masks, boxes, scores = masks[keep], boxes[keep], scores[keep]
    n, h, w = masks.shape

    print(f"[sam3] '{prompt}': {n} instance(s)")
    body = masks.tobytes() + boxes.tobytes() + scores.tobytes()
    meta = json.dumps({"num_instances": n, "height": h, "width": w})
    query.reply(query.key_expr, payload=body, attachment=meta.encode())


def main():
    _load_model()
    _warmup()

    # Cross-container SHM availability isn't guaranteed (separate /dev/shm,
    # possibly no shared memlock headroom), and a single detect query is
    # small/infrequent enough that plain networking is fine.
    cfg = zenoh.Config()
    cfg.insert_json5("transport/shared_memory/enabled", "false")
    # A fixed listen port so a remote client (e.g. one on the robot, see
    # ZENOH_CONNECT in core/utils/zenoh_rpc.py) can reach this server without
    # relying on multicast scouting to find an ephemeral one.
    listen = os.environ.get("ZENOH_LISTEN", "tcp/0.0.0.0:2002")
    cfg.insert_json5("listen/endpoints", json.dumps([listen]))
    # Multicast scouting binds a single shared UDP port (224.0.0.224:7446)
    # host-wide; with multiple --net=host containers (or a stray leftover
    # process) it fails with "Address already in use". Not needed since
    # clients connect via the fixed listen port above.
    cfg.insert_json5("scouting/multicast/enabled", "false")

    with zenoh.open(cfg) as session:
        session.declare_queryable("sam3/detect", _on_detect)
        print("[sam3] Listening on 'sam3/detect'...")
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
