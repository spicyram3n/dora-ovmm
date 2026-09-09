FROM pytorch/pytorch:1.12.1-cuda11.3-cudnn8-devel

ENV DEBIAN_FRONTEND=noninteractive
ENV CUDA_HOME=/usr/local/cuda

# Native build/runtime dependencies for Python, MinkowskiEngine, PointNet2, Open3D.
RUN apt-get update && apt-get install -y --no-install-recommends \
    git build-essential gcc g++ \
    libopenblas-dev libgl1 libglib2.0-0 libgomp1 \
    ninja-build wget unzip ca-certificates \
    libssl-dev zlib1g-dev libbz2-dev libreadline-dev \
    libsqlite3-dev libffi-dev liblzma-dev uuid-dev \
    && rm -rf /var/lib/apt/lists/*

ARG OPENYOLO3D_REF=24f344d001d28d1cdd4087d3769c0d53c34d64cd
ARG TORCH_CUDA_ARCH_LIST="7.0 7.5 8.0 8.6+PTX"
ENV TORCH_CUDA_ARCH_LIST=${TORCH_CUDA_ARCH_LIST} \
    MAX_JOBS=2 \
    OMP_NUM_THREADS=2

RUN git clone https://github.com/aminebdj/OpenYOLO3D.git /opt/openyolo3d && \
    cd /opt/openyolo3d && git checkout "$OPENYOLO3D_REF"
WORKDIR /opt/openyolo3d

# This pinned OpenYOLO3D commit imports supervision in utils_2d.py but never
# uses it. Remove the dead import rather than adding another modern CV package
# and its dependency constraints to this frozen Torch/CUDA environment.
RUN sed -i '/^import supervision as sv$/d' utils/utils_2d.py

# Keep the old CUDA image's glibc. Build a clean Python 3.10 interpreter instead
# of upgrading the image's Conda environment.
ARG PYTHON_VERSION=3.10.21
RUN wget -q "https://www.python.org/ftp/python/${PYTHON_VERSION}/Python-${PYTHON_VERSION}.tgz" -O /tmp/python.tgz && \
    mkdir /tmp/python-src && \
    tar -xzf /tmp/python.tgz -C /tmp/python-src --strip-components=1 && \
    cd /tmp/python-src && \
    PATH=/usr/bin:/bin ./configure --prefix=/opt/python310 --with-ensurepip=install && \
    PATH=/usr/bin:/bin make -j2 && \
    PATH=/usr/bin:/bin make install && \
    /opt/python310/bin/python3.10 -m venv /opt/openyolo-venv && \
    rm -rf /tmp/python-src /tmp/python.tgz

ENV PATH="/opt/openyolo-venv/bin:/usr/local/nvidia/bin:/usr/local/cuda/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

RUN python -c "import sys,ssl,sqlite3,bz2,lzma,ctypes; assert sys.version_info[:2] == (3,10); assert sys.prefix == '/opt/openyolo-venv'; print(sys.version, ssl.OPENSSL_VERSION)"

# Freeze packaging tools before touching the 2022/2023 dependency stack.
RUN python -m pip install --no-cache-dir \
    pip==23.3.2 setuptools==65.7.0 wheel==0.41.3 Cython==0.29.37

# Core CUDA stack. Never allow later package installs to replace these.
RUN python -m pip install --no-cache-dir \
    torch==1.12.1+cu113 torchvision==0.13.1+cu113 \
    --extra-index-url https://download.pytorch.org/whl/cu113 && \
    python -c "import torch,torchvision; assert torch.__version__ == '1.12.1+cu113'; assert torch.version.cuda == '11.3'; print(torch.__version__, torchvision.__version__, torch.version.cuda)"

COPY requirements-upstream.txt /tmp/requirements-upstream.txt

# Global resolver guardrails. PIP_CONSTRAINT also reaches isolated build envs.
RUN printf '%s\n' \
    'Cython<3' \
    'PyYAML==5.4.1' \
    'numpy==1.24.2' \
    > /tmp/constraints.txt
ENV PIP_CONSTRAINT=/tmp/constraints.txt

# Two upstream pins are hostile to modern PyPI builds:
#   PyYAML 5.4.1 needs Cython <3 and no build isolation.
#   pycocotools 2.0.4 has a broken sdist missing ../common/maskApi.c.
# Install them safely first, then remove their lines from the bulk requirements.
RUN python -m pip install --no-cache-dir --no-build-isolation PyYAML==5.4.1 && \
    python -m pip install --no-cache-dir --only-binary=:all: pycocotools==2.0.7 && \
    sed -E '/^[[:space:]]*(PyYAML|pycocotools)==/d' /tmp/requirements-upstream.txt > /tmp/requirements-safe.txt && \
    python -c "import yaml,pycocotools; assert yaml.__version__ == '5.4.1'"

# Install the frozen upstream Python stack, preferring wheels to avoid needless
# source builds against today's packaging ecosystem.
RUN python -m pip install --no-cache-dir --prefer-binary -r /tmp/requirements-safe.txt

# Extra packages used by Mask3D/OpenYOLO3D. Install with the core torch already
# present; its version remains protected by the environment and later checks.
RUN python -m pip install --no-cache-dir --prefer-binary \
    pillow==9.3.0 \
    iopath==0.1.7 \
    albumentations==1.2.1 \
    pytorch-lightning==1.7.2 \
    torchmetrics==0.11.4 \
    open3d==0.17.0 \
    transformers==4.36.2 \
    timm==0.6.13 \
    einops fvcore gdown yapf==0.40.1 && \
    python -c "import torch,yaml,open3d,transformers,pytorch_lightning; assert torch.__version__ == '1.12.1+cu113'; assert yaml.__version__ == '5.4.1'; print('python deps OK')"

# Match the original Mask3D stack exactly. 2.1.0 has a cp310/cu113 wheel.
RUN python -m pip install --no-cache-dir torch-scatter==2.1.0 \
    -f https://data.pyg.org/whl/torch-1.12.1+cu113.html && \
    python -m pip install --no-cache-dir --no-deps \
    'git+https://github.com/facebookresearch/detectron2.git@710e7795d0eeadf9def0e7ef957eea13532e34cf'

# MinkowskiEngine needs the recursive checkout and the CUDA toolkit from the
# devel image. Explicit CUDA_HOME prevents it from accidentally inspecting a
# different CUDA installation.
RUN git clone --recursive https://github.com/NVIDIA/MinkowskiEngine /tmp/MinkowskiEngine && \
    cd /tmp/MinkowskiEngine && \
    git checkout 02fc608bea4c0549b0a7b00ca1bf15dee4a0b228 && \
    python setup.py install --force_cuda --cuda_home=/usr/local/cuda --blas=openblas && \
    rm -rf /tmp/MinkowskiEngine

RUN cd models/Mask3D/third_party/pointnet2 && \
    python setup.py install

# Do not let either vendored project resolve its own modern dependency metadata.
RUN python -m pip install --no-cache-dir --no-deps ./models/Mask3D && \
    python -m pip install --no-cache-dir --no-deps --no-build-isolation -e ./models/YOLO-World

# Install compiled MMCV FIRST. Use the actual torch1.12.0 wheel directory; it is
# the wheel family used for PyTorch 1.12.x + CUDA 11.3. Installing mmyolo before
# this can pull mmcv-lite and break mmcv.ops.
RUN python -m pip install --no-cache-dir mmcv==2.0.0 \
    -f https://download.openmmlab.com/mmcv/dist/cu113/torch1.12.0/index.html && \
    python -m pip install --no-cache-dir \
    mmengine==0.10.3 mmdet==3.0.0 mmyolo==0.6.0

# Fail the image build here, not at runtime, if any critical native extension or
# frozen version was displaced.
RUN python - <<'PY'
import torch
import yaml
import pycocotools
import MinkowskiEngine
import torch_scatter
import detectron2
import mmcv
import mmengine
import mmdet
import mmyolo
from mmcv.ops import nms
import pointnet2.pointnet2_utils
from utils import OpenYolo3D

assert torch.__version__ == '1.12.1+cu113', torch.__version__
assert torch.version.cuda == '11.3', torch.version.cuda
assert yaml.__version__ == '5.4.1', yaml.__version__
assert mmcv.__version__ == '2.0.0', mmcv.__version__
print('OpenYOLO3D build smoke test PASSED')
PY

COPY app.py /opt/openyolo3d/app.py
ENTRYPOINT ["python", "app.py"]