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
  ffmpeg build-essential cmake pkg-config wget pybind11-dev



COPY WebSocketGateway/requirements.txt .

RUN python3 -m pip install --no-cache-dir -r requirements.txt
RUN mkdir -p /app/tankstatus/ 

COPY TankStatusProtocol /app/tankstatus

# Fix the path to match the destination above
WORKDIR /app/tankstatus/builds

# Now CMake will find your CMakeLists.txt in the parent directory
RUN rm -rf * && cmake .. && make

WORKDIR /app/

RUN find /app/tankstatus -name "*.so"

ENV PYTHONPATH="/app/tankstatus:/app/tankstatus/builds:${PYTHONPATH:-}"
RUN python3 -c "import tankstatus_wrapper; print('Success! Wrapper imported cleanly.')"

# Create User and Setup Permissions
RUN useradd -m dockerUser
RUN chown -R dockerUser:dockerUser /app/projects
RUN usermod -aG dialout dockerUser



WORKDIR /app/projects
USER dockerUser

EXPOSE 8888

# Run Jupyter directly from the system Python environment
ENTRYPOINT ["python3", "-m", "jupyter", "lab", "--ip=0.0.0.0", "--port=8888", "--no-browser" , "--allow-root"]
