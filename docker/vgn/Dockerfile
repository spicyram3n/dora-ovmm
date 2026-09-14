FROM nvidia/cuda:12.4.1-cudnn-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.10 python3.10-venv git curl unzip ca-certificates \
    && rm -rf /var/lib/apt/lists/*

RUN python3.10 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

# VGN is a 1 MB 3D CNN; any recent torch runs it.
RUN pip install --no-cache-dir torch==2.5.1 --index-url https://download.pytorch.org/whl/cu124

# zenoh is pinned to match the ROS 2 side and the other model containers.
# scipy < 2: VGN's post-processing calls scipy.ndimage.morphology, which 2.0 drops.
RUN pip install --no-cache-dir "eclipse-zenoh==1.9.0" "numpy<2" "scipy<2"

# The devel branch is the one active_grasp builds on. Its detection code needs
# only robot_helpers' Transform, so neither is installed: see PYTHONPATH below.
RUN git clone -b devel https://github.com/ethz-asl/vgn.git /opt/vgn \
    && git -C /opt/vgn checkout f51c3fda136f12326b50141aa519395644da5c27 \
    && git clone https://github.com/mbreyer/robot_helpers.git /opt/robot_helpers \
    && git -C /opt/robot_helpers checkout ba6e7af

# Upstream's pretrained model (the devel README's Google Drive link), checked
# against the copy this image was set up with.
RUN mkdir -p /opt/vgn/assets/models && cd /tmp \
    && curl -fsSL -o models.zip \
       "https://drive.usercontent.google.com/download?id=1J3cPjyVQ59LpcLZZrA7EfeV3xTmITchr&export=download&confirm=t" \
    && unzip -j models.zip models/vgn_conv.pth -d /opt/vgn/assets/models \
    && echo "ba3391d0805e9c9b178cd18106866313cee808ff2b654f689663e92a814cec4b  /opt/vgn/assets/models/vgn_conv.pth" \
       | sha256sum -c - \
    && rm models.zip

ENV PYTHONPATH=/opt/vgn/src:/opt/robot_helpers

WORKDIR /app
COPY docker/vgn/app.py /app/app.py
# The healthcheck in docker/compose.yaml runs this inside the container.
COPY core/utils/zenoh_rpc.py /app/zenoh_rpc.py

CMD ["python3", "app.py"]
