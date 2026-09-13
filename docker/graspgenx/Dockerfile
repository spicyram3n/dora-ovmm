FROM nvidia/cuda:12.4.1-cudnn-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.10 python3.10-venv python3.10-dev git git-lfs curl build-essential \
    libgl1 libglib2.0-0 libosmesa6-dev \
    && git lfs install --system \
    && rm -rf /var/lib/apt/lists/*


RUN curl -LsSf https://astral.sh/uv/install.sh | sh
ENV PATH="/root/.local/bin:$PATH"

RUN git clone https://github.com/NVlabs/GraspGenX.git /opt/graspgenx
WORKDIR /opt/graspgenx
RUN uv sync --python 3.10

RUN uv add "eclipse-zenoh==1.9.0"

COPY docker/graspgenx/app.py app.py
# The healthcheck in docker/compose.yaml runs this inside the container.
COPY core/utils/zenoh_rpc.py /opt/graspgenx/zenoh_rpc.py

CMD ["uv", "run", "python3", "app.py"]
