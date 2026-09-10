"""
GraspGenX inference server, over zenoh: take an object-centric point cloud
and a gripper name, ask GraspGenX for candidate 6-DOF grasp poses.

GraspGenX is one cross-embodiment model - the same weights generalize to any
gripper whose geometry has been registered under assets/x_grippers/, with no
retraining. That's what lets it support HSRC's real gripper: run the wizard
once (see gripper_export/ and run_wizard.sh) against the exported HSRC hand
URDF, and this server picks the resulting "hsrc_hand" config up automatically.

Runs in its own container, separate from ROS 2 and SAM3 - same reasoning as
that split: independent, incompatible dependency pins, only talking to the
rest of the system over zenoh.

Zenoh queryable key: "graspgenx/generate"
    query payload:     point cloud, raw float32 bytes, shape (N, 3)
    query parameters:  gripper_name (optional, default: $GRASPGENX_DEFAULT_GRIPPER)
                        num_grasps   (optional int, default 200)

Reply payload: raw bytes = grasps.tobytes() + scores.tobytes()
    grasps: (M, 4, 4) float32, homogeneous grasp poses in the same frame
            the input point cloud was in
    scores: (M,) float32, confidence in [0, 1]
Reply attachment: json {"num_grasps": M, "gripper_name": "..."}
"""

import json
import os
import time

import numpy as np
import zenoh

ASSETS_DIR = "/opt/graspgenx/assets"
DEFAULT_GRIPPER = os.environ.get("GRASPGENX_DEFAULT_GRIPPER", "hsrc_hand")

MODEL = None
CFG = None
SAMPLERS = {}


def _load_model():
    """Load the shared, gripper-independent GraspGenX weights once. Every
    gripper gets its own lightweight sampler wrapping this same model - see
    _get_sampler()."""
    global MODEL, CFG
    from graspgenx._setup_dependencies import get_checkpoints_version_dir
    from graspgenx.grasp_server import load_grasp_gen_model
    from graspgenx.utils.checkpoint_io import load_model_cfg

    print("[graspgenx] loading model weights...")
    ckpt_dir = get_checkpoints_version_dir()  # <checkpoints_root>/release/
    CFG = load_model_cfg(str(ckpt_dir / "gen"), str(ckpt_dir / "dis"))
    MODEL = load_grasp_gen_model(CFG)
    print("[graspgenx] model loaded.")


def _get_sampler(gripper_name):
    """Lazily build and cache a sampler for a gripper name. Cheap once the
    shared model is loaded - a sampler is just that model plus the named
    gripper's geometry read from ASSETS_DIR/x_grippers/<name>/."""
    from graspgenx.grasp_server import GraspGenXSampler

    sampler = SAMPLERS.get(gripper_name)
    if sampler is None:
        print(f"[graspgenx] loading gripper '{gripper_name}'...")
        sampler = GraspGenXSampler(CFG, gripper_name, assets_dir=ASSETS_DIR, model=MODEL)
        SAMPLERS[gripper_name] = sampler
    return sampler


def _warmup():
    """Run one dummy inference pass so CUDA kernels are compiled/selected now,
    at startup, rather than on the first real request (~10-30s slower)."""
    from graspgenx.grasp_server import GraspGenXSampler

    print(f"[graspgenx] warming up with default gripper '{DEFAULT_GRIPPER}'...")
    try:
        sampler = _get_sampler(DEFAULT_GRIPPER)
    except ValueError as exc:
        # Most likely the wizard hasn't been run yet for this gripper name -
        # see run_wizard.sh. Not fatal: the server still starts, and a real
        # request naming an existing gripper will work fine.
        print(f"[graspgenx] skipping warmup: {exc}")
        return

    dummy_pc = np.random.uniform(-0.05, 0.05, size=(1024, 3)).astype(np.float32)
    GraspGenXSampler.run_inference(dummy_pc, sampler)
    print("[graspgenx] warmup done - ready for requests.")


def _on_generate(query):
    from graspgenx.grasp_server import GraspGenXSampler

    payload = query.payload.to_bytes()
    if not payload:
        query.reply_err(b"no point cloud provided")
        return

    pc = np.frombuffer(payload, dtype=np.float32).reshape(-1, 3)
    gripper_name = query.parameters.get("gripper_name", DEFAULT_GRIPPER)
    num_grasps = int(query.parameters.get("num_grasps", "200"))

    sampler = _get_sampler(gripper_name)
    grasps, scores = GraspGenXSampler.run_inference(pc, sampler, num_grasps=num_grasps)
    grasps = grasps.detach().cpu().numpy().astype(np.float32) if len(grasps) else np.zeros((0, 4, 4), np.float32)
    scores = scores.detach().cpu().numpy().astype(np.float32) if len(scores) else np.zeros((0,), np.float32)

    print(f"[graspgenx] '{gripper_name}': {pc.shape[0]} input points -> {len(scores)} grasp(s)")
    body = grasps.tobytes() + scores.tobytes()
    meta = json.dumps({"num_grasps": len(scores), "gripper_name": gripper_name})
    query.reply(query.key_expr, payload=body, attachment=meta.encode())


def main():
    _load_model()
    _warmup()
    cfg = zenoh.Config()
    cfg.insert_json5("transport/shared_memory/enabled", "false")
    listen = os.environ.get("ZENOH_LISTEN", "tcp/0.0.0.0:7448")
    cfg.insert_json5("listen/endpoints", json.dumps([listen]))

    with zenoh.open(cfg) as session:
        session.declare_queryable("graspgenx/generate", _on_generate)
        print("[graspgenx] listening on 'graspgenx/generate'...")
        while True:
            time.sleep(1)


if __name__ == "__main__":
    main()
