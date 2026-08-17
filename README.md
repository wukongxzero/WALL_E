# WALL-E V3 — Autonomous Tank Robot

A fully autonomous tracked robot running on a Jetson Orin Nano. WALL-E V3 combines natural-language command parsing (LLaMA 3.2), semantic object navigation (YOLOv8 + RealSense D435), and a hardware abstraction layer for three concurrent serial buses — all orchestrated through a ROS 2 state machine.

## What it does

- **Natural-language control**: speak or type commands ("go find the chair") — LLaMA 3.2:3b running locally via Ollama parses intent into structured JSON navigation goals
- **Semantic navigation**: YOLOv8n detects objects in the depth camera feed and projects 2D detections to 3D map coordinates; the robot drives to within 0.5 m of the target
- **Manual override**: Parallax Bluetooth controller for direct teleoperation
- **Stabilisation**: MPU6050 IMU on an Arduino Uno keeps a 2-axis servo gimbal level

## Hardware

| Component | Interface | Baud |
|-----------|-----------|------|
| Arduino Mega (drive motors, odometry) | `/dev/ttyUSB0` | 115200 |
| Arduino Uno (MPU6050 + servo gimbal) | `/dev/ttyUSB1` | 115200 |
| Parallax Bluetooth controller | `/dev/rfcomm0` | 9600 |
| Intel RealSense D435 | USB 3 | — |
| NVIDIA Jetson Orin Nano | — | — |

Tank tread differential drive. Motors driven by H-bridge from Mega PWM outputs.

## Architecture

```
Parallax BT Controller (/dev/rfcomm0)
        │
        ▼
controller_node → /joy_cmd_vel
                        │
LLaMA 3.2:3b ──────────▼
(Ollama, local)  state_machine (MANUAL / AUTONOMOUS / IDLE)
                        │
YOLOv8n + D435 ─────────┤
→ /goal_pose            │
                        ▼
                    /cmd_vel
                        │
              ┌─────────┴────────┐
              ▼                  ▼
        mega_bridge         uno_bridge
        /dev/ttyUSB0        /dev/ttyUSB1
        Arduino Mega        Arduino Uno
        (drive motors)      (gimbal stabiliser)
```

## Node descriptions

| Node | Language | What it does |
|------|----------|--------------|
| `mega_node` | C++ | Background read thread, parses 22-byte odometry packets (0xAA 0xBB sync header), broadcasts TF `odom→base_link`, watchdog 500ms |
| `uno_bridge` | C++ | Reads pitch/roll from Uno, publishes `/wall_e/pitch` |
| `state_machine` | C++ | Routes `/joy_cmd_vel` or `/nav_cmd_vel` to `/cmd_vel`; handles MANUAL/AUTONOMOUS/IDLE transitions and emergency stop |
| `controller_node` | C++ | Reads Parallax Bluetooth controller, publishes `/joy_cmd_vel` |
| `llama_interpreter_node` | Python | Sends natural-language commands to LLaMA 3.2:3b via Ollama; parses JSON response into `/goal_pose` |
| `yolo_nav_node` | Python | YOLOv8n inference on D435 colour frame; pinhole-projects detection centroid to 3D, transforms to map frame, publishes `/goal_pose` 0.5 m in front of object |

## LLaMA NL command flow

```
User text input
    │
    ▼
Ollama (LLaMA 3.2:3b, local, Docker container)
    │  System prompt: JSON-only output with robot pose/mode/pitch context
    ▼
{"action": "navigate", "target": "chair", "x": 1.2, "y": 0.5}
    │
    ▼
/goal_pose → state_machine → Nav2
```

## YOLO projection math

```python
# Project 2D detection centroid to 3D using depth + pinhole model
depth = depth_frame[cy, cx]                         # metres
x_cam = (cx - cx_) * depth / fx_                   # camera-frame X
y_cam = (cy - cy_) * depth / fy_                   # camera-frame Y

# Transform to map frame via TF
point_camera = PointStamped(frame="camera_frame", x=x_cam, y=y_cam, z=depth)
point_map = tf_buffer_.transform(point_camera, "map")

# Publish goal 0.5m in front of detected object
goal.pose.position.x = point_map.x + 0.5 * cos(heading)
```

## Serial packet protocol (Mega → Jetson)

```
Odometry packet (22 bytes):
  [0]    = 0xAA  (sync byte 1)
  [1]    = 0xBB  (sync byte 2)
  [2-5]  = left encoder (int32)
  [6-9]  = right encoder (int32)
  [10-13]= heading (float, radians)
  [14-21]= padding / checksum
```

## State machine

| State | Source | Trigger |
|-------|--------|---------|
| `MANUAL` | Parallax controller → `/joy_cmd_vel` | Default on startup |
| `AUTONOMOUS` | Nav2 / YOLO / LLaMA → `/nav_cmd_vel` | Controller toggle button |
| `IDLE` | No output — motors stopped | Emergency stop button |

## Setup

### Pair Bluetooth controller
```bash
bluetoothctl scan on
sudo rfcomm bind 0 <PARALLAX_MAC>
```

### Run with Docker Compose
```bash
docker compose up   # starts ros:humble + ollama/ollama services
```

### Build (native)
```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select wall_e_bringup
source install/setup.bash
ros2 launch wall_e_bringup wall_e.launch.py
```

### Dependencies
```bash
sudo apt install ros-humble-realsense2-camera ros-humble-navigation2 \
                 ros-humble-nav2-bringup ros-humble-robot-localization
pip install ultralytics ollama
```

## Tech stack

ROS 2 Humble · C++ · Python · YOLOv8n · LLaMA 3.2:3b · Ollama · Intel RealSense · NVIDIA Jetson Orin Nano · Docker Compose
