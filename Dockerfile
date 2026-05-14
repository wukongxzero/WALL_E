FROM ros:humble

# ── System dependencies ────────────────────────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    python3-pip \
    python3-opencv \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-nav2-bringup \
    ros-humble-navigation2 \
    ros-humble-rtabmap-slam \
    ros-humble-realsense2-camera \
    ros-humble-tf2-ros \
    ros-humble-cv-bridge \
    ros-humble-rviz2 \
    ros-humble-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*
    
# ── Python dependencies ────────────────────────────────────────────────────────
# torch from PyPI pulls the full CUDA stack (~1GB on aarch64).
# If you want GPU-accelerated torch on Jetson, swap this for the NVIDIA JetPack
# wheel: https://developer.download.nvidia.com/compute/redist/jp/
# For now we install the same packages your command: block was installing.
RUN pip3 install --no-cache-dir \
    ollama \
    ultralytics \
    aiortc \
    aiohttp \
    av

# ── Entrypoint ─────────────────────────────────────────────────────────────────
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /ros2_ws
ENTRYPOINT ["/entrypoint.sh"]
