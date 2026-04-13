# WALL-E ROS 2 Autonomous Navigation Stack

## Overview
ROS 2 Humble autonomous navigation stack for WALL-E tank bot running on Jetson Orin.
Migrated from ROS 1 Noetic simulation to real hardware.

## Hardware
- NVIDIA Jetson Orin
- Arduino Mega (drive motors)
- Arduino Uno (2-axis stabilization via MPU6050 + servos)
- Intel RealSense D435 (SLAM + visual odometry)
- Parallax custom Bluetooth controller
- Tank tread differential drive

## Architecture

Parallax Controller (Bluetooth /dev/rfcomm0)
        down
controller_node publishes /joy_cmd_vel
        down
state_machine (MANUAL / AUTONOMOUS / IDLE)
        down
/cmd_vel
        down
mega_bridge -> /dev/ttyUSB0 -> Arduino Mega -> motors

uno_bridge <- /dev/ttyUSB1 <- Arduino Uno -> stabilizer
        down
/wall_e/pitch

D435 -> RTAB-Map -> map + odometry
        down
Nav2 -> /nav_cmd_vel -> state_machine

## Nodes

| Node             | Description                                  |
|------------------|----------------------------------------------|
| mega_bridge      | Sends TankStatus packets to Mega over USB    |
| uno_bridge       | Reads pitch from Uno over USB                |
| state_machine    | Gates cmd_vel between manual and autonomous  |
| controller_node  | Reads Parallax BT controller                 |

## Serial Protocol
16-byte TankStatus packet matching Arduino TankStatus.h

buf[0]   = driveLeft  (unsigned, 127=stop)
buf[1]   = driveRight (unsigned, 127=stop)
buf[2-3] = padding
buf[4-5] = eulerX (short)
buf[6-7] = eulerY (short)
buf[8]   = eulerZ (char, default 90)
buf[9-15]= padding

## Setup

### Hardware Connections
Jetson /dev/ttyUSB0 -> Arduino Mega (115200 baud)
Jetson /dev/ttyUSB1 -> Arduino Uno  (115200 baud)
Jetson /dev/rfcomm0 -> Parallax controller (9600 baud)
Jetson USB          -> Intel RealSense D435

### Pair Parallax Controller
bluetoothctl scan on
sudo rfcomm bind 0 <MAC_ADDRESS>

### Install Dependencies
sudo apt install ros-humble-realsense2-camera -y
sudo apt install ros-humble-rtabmap-ros -y
sudo apt install ros-humble-navigation2 -y
sudo apt install ros-humble-nav2-bringup -y
sudo apt install ros-humble-robot-localization -y

### Build
cd ~/WALL_E_ROS2
source /opt/ros/humble/setup.bash
colcon build --packages-select wall_e_bringup
source install/setup.bash

### Run
ros2 launch wall_e_bringup wall_e.launch.py

## State Machine

| State      | Description              | Trigger                   |
|------------|--------------------------|---------------------------|
| MANUAL     | Parallax controller      | Default on startup        |
| AUTONOMOUS | Nav2 drives bot          | Toggle button on controller|
| IDLE       | Motors stopped           | Emergency stop button     |

## TODO
- URDF robot description
- RTAB-Map SLAM launch config
- Nav2 configuration
- Odometry from encoders
- Object/person following
- LCD state display node


