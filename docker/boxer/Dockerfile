FROM nvidia/cuda:12.6.3-cudnn-runtime-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
# ffmpeg is not optional: utils/video.py shells out to it (libx264) to encode
# boxer_viz_final.mp4, and visualisation is on unless --skip_viz is passed.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.12 python3.12-venv python3-pip git wget ca-certificates \
    ffmpeg libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN python3.12 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# Boxer only asks for torch>=2.0. Pinned to the SAM3 image's version so the two
# CUDA 12.6 containers share one tested torch build.
RUN pip install --no-cache-dir torch==2.7.0 \
    --index-url https://download.pytorch.org/whl/cu126

ARG BOXER_REF=1f86542dc342a4b1d474c87c97c5d1d6566d9148
RUN git clone https://github.com/facebookresearch/boxer.git /opt/boxer \
    && cd /opt/boxer && git checkout "$BOXER_REF"
WORKDIR /opt/boxer

# Upstream's dependency list with opencv-python swapped for the headless wheel:
# nothing on the run_boxer.py path calls cv2.imshow, so the GUI stack is dead
# weight. projectaria-tools reads the .vrs sample sequences the demo uses.
RUN pip install --no-cache-dir \
    numpy opencv-python-headless tqdm dill pillow projectaria-tools

# The view_*.py demos need moderngl plus a real display, which this image does
# not have. Build with --build-arg BOXER_VIEWER=1 if you intend to forward X11.
# scipy and scikit-learn come with it: the viewer and the tests want them, the
# headless pipeline does not.
ARG BOXER_VIEWER=0
RUN if [ "$BOXER_VIEWER" = 1 ]; then pip install --no-cache-dir \
    moderngl moderngl-window imgui-bundle scipy scikit-learn; fi

# Fail the build, not the first run, if a dependency or the video encoder is
# missing. get_video_codec() greps `ffmpeg -codecs` and raises without h264.
RUN python - <<'PY'
import os

import cv2
import numpy
import torch
import tqdm

from boxernet.boxernet import BoxerNet
from loaders.aria_loader import AriaLoader
from loaders.ca_loader import CALoader
from loaders.omni_loader import OmniLoader
from loaders.scannet_loader import ScanNetLoader
from owl.owl_wrapper import OwlWrapper
from utils.demo_utils import CKPT_PATH, EVAL_PATH, SAMPLE_DATA_PATH
from utils.video import find_ffmpeg, get_video_codec

assert torch.__version__.startswith('2.7.0'), torch.__version__
print('ffmpeg:', find_ffmpeg(os.path.expanduser('~')), get_video_codec())
print('Boxer build smoke test PASSED')
PY

# Checkpoints, sample sequences and results all resolve relative to the repo
# root (utils/demo_utils.py), so run_boxer.sh mounts these three paths.
# ckpts/ must stay writable: OwlWrapper caches text embeddings next to the .pt.
VOLUME ["/opt/boxer/ckpts", "/opt/boxer/sample_data", "/opt/boxer/output"]

ENTRYPOINT ["python", "run_boxer.py"]
