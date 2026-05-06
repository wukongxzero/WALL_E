ARG TARGETARCH

# 1. Jetson Base (L4T ML includes Python 3, JupyterLab, PyTorch, and CUDA OpenCV!)
FROM nvcr.io/nvidia/l4t-ml:r36.2.0-py3 AS base-arm64

# 2. Desktop Base
# Note: The standard cuda-devel image doesn't have Python built-in like l4t-ml does, 
# so we install it here for x86 compatibility.
FROM nvidia/cuda:12.4.1-devel-ubuntu22.04 AS base-amd64
RUN apt-get update && apt-get install -y python3 python3-pip && rm -rf /var/lib/apt/lists/*

# 3. Final Stage
FROM base-${TARGETARCH} AS final

# Set environment variables
ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC

RUN mkdir -p /app/
RUN mkdir /app/projects

# Install system dependencies

RUN apt-get update && apt-get install -y \
    ffmpeg build-essential cmake pkg-config wget pybind11-dev \
    && rm -rf /var/lib/apt/lists/*

ENV LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libstdc++.so.6"
ENV LD_LIBRARY_PATH="/usr/local/cuda/lib64:/usr/lib/aarch64-linux-gnu:${LD_LIBRARY_PATH}"

COPY requirements.txt .

RUN mkdir -p /app/builds/ 
RUN mkdir -p /app/tankstatus/ 
COPY tankstatus /app/tankstatus

# Install JupyterLab (for x86 base) and your requirements
# Note: l4t-ml already has Jupyter installed, so it will just skip it on the Jetson
RUN python3 -m pip install --no-cache-dir -r requirements.txt

WORKDIR /app/builds/
RUN cmake /app/tankstatus/
RUN make
RUN cp -r * /app/projects/


# Create User and Setup Permissions
RUN useradd -m dockerUser
RUN chown -R dockerUser:dockerUser /app/projects

USER dockerUser
WORKDIR /app/projects

EXPOSE 8888

# Run Jupyter directly from the system Python environment
ENTRYPOINT ["python3", "-m", "jupyter", "lab", "--ip=0.0.0.0", "--port=8888", "--no-browser"]
