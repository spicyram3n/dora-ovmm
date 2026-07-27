FROM nvidia/cuda:12.6.3-cudnn-runtime-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.12 python3.12-venv python3-pip git wget \
    libgl1 libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

RUN python3.12 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

RUN pip install --no-cache-dir torch==2.7.0 torchvision \
    --index-url https://download.pytorch.org/whl/cu126


RUN git clone https://github.com/facebookresearch/sam3.git /opt/sam3 \
    && pip install --no-cache-dir -e "/opt/sam3[train]"


RUN pip install --no-cache-dir eclipse-zenoh pillow "numpy<2" \
    opencv-python-headless huggingface_hub einops pycocotools pandas psutil

ARG HF_TOKEN=""
ENV HF_TOKEN=$HF_TOKEN

WORKDIR /app
COPY app.py /app/app.py

CMD ["python3", "app.py"]
