# WALL-E — TA Office Hours Prep (2026-09-11)

## Commands — build, launch, verify

**One-time workspace note**: only `~/WALL_E/ros2_ws/` is real (has actual source). Any stray top-level `~/WALL_E/install`/`build`/`log` are orphaned copies — already cleaned up, but if they ever reappear, ignore them.

**Build:**
```bash
cd ~/WALL_E/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

**Launch, Terminal 1 — Isaac Sim + ROS2 bridge (GUI window):**
```bash
cd ~/WALL_E/simulation/scripts
__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia DISPLAY=:1 \
  ~/WALL_E/isaac-sim/python.sh wall_e_perception.py
```
Takes ~20-25s to reach "app ready" in the log — extension loading, not a hang.

**Launch, Terminal 2 — the ROS2 stack:**
```bash
source /opt/ros/jazzy/setup.bash
source ~/WALL_E/ros2_ws/install/setup.bash
ros2 launch wall_e_bringup wall_e_isaac.launch.py
```

**Verify it's actually alive, not just launched:**
```bash
ros2 topic hz /odom          # should show ~55-60Hz
ros2 node list                # should show the full Nav2 stack, rtabmap, state_machine, rviz2
```

**Demo the two known bugs live, side by side:**
```bash
# Bug 1: topic-name mismatch (issue #1 on the list below)
ros2 topic info /nav_cmd_vel --verbose    # Publisher count: 0 -- nothing feeds state_machine's subscription
ros2 topic info /cmd_vel_nav --verbose    # Nav2's controller_server IS publishing here instead

# Bug 3: TF disconnect (issue #3 on the list below)
ros2 run tf2_ros tf2_echo odom base_footprint   # hangs or reports no connection if actually broken
ros2 run tf2_tools view_frames                   # generates a PDF of the live TF tree -- visual proof
```

## What it is

WALL-E is a tracked mobile robot running a full ROS2 autonomy stack: SLAM (RTAB-Map + RealSense D435), Nav2 navigation, YOLO-based semantic object detection, and a local-LLM (LLaMA 3.2 via Ollama) natural-language command layer. Software-only work — hardware chapter is closed. Currently ported into Isaac Sim/Isaac Lab with a working `skrl` (PPO/TD3) RL training pipeline for a learned navigation policy.

## Architecture — what's actually running

```
[Bluetooth controller] → controller_node (C++) → /joy_cmd_vel, /state_toggle, /emergency_stop
                                                          │
[camera/D435] → realsense2_camera → RTAB-Map SLAM → /rtabmap/odom
                                                          │
                                                   state_machine (C++)
                                          MANUAL / AUTONOMOUS / IDLE mode arbiter
                                          + safety_watchdog(): camera/localization/serial staleness → forces IDLE
                                                          │
                                                     /cmd_vel
                                                          │
[Arduino Mega] ←── mega_node (C++, serial) ──→ /odom, TF odom→base_footprint

[Arduino Uno]  ←── uno_bridge (C++, serial, publish-only) ──→ /wall_e/pitch, /wall_e/roll

[YOLO + camera] → yolo_nav_node.py → navigate_to_pose ACTION → Nav2 stack → (should feed back to state_machine)

llama_interpreter_node.py (local LLaMA 3.2 via Ollama) → /yolo_target → triggers yolo_nav_node
```

Isaac Sim side: `wall_e_perception.py` (OmniGraph-based) replaces the hardware nodes for sim-driven testing, publishing the same `/cmd_vel`/`/odom`/`/tf` contract — in principle a drop-in swap for the real hardware, though the camera topics don't currently agree (see open issues).

## What's validated and working

- Autonomous goal navigation end-to-end in both Gazebo and Isaac Sim
- SLAM map-building confirmed with a real depth-camera feed in Isaac Sim
- Manual drive path fully traced and confirmed correct, including the firmware-level differential-drive sign convention
- `skrl` PPO/TD3 RL training pipeline runs end-to-end (real training loop, policy updates confirmed executing)
- CI (GitHub Actions: colcon build/test, style lint) + local pre-commit hook wired up for the ROS2 package

## Open issues — found by tracing the actual code, not guessed

1. **Topic-name mismatch, AUTONOMOUS mode is currently dead on real hardware.** `state_machine.cpp` subscribes to `/nav_cmd_vel`; Nav2's `controller_server` is configured to publish `/cmd_vel_nav` (`nav2_params.yaml`). Different topic, never remapped — AUTONOMOUS mode never receives a drive command.
2. **Same bug class, second instance.** The Isaac Sim bridge (`wall_e_perception.py`) publishes camera topics under different names than what `state_machine.cpp`/`yolo_nav_node.py` actually subscribe to — three different naming conventions across three files, none reconciled.
3. **TF tree disconnect**, self-reported in an old commit message as "found, not yet investigated": `wall_e_isaac.launch.py`'s Nav2 costmap reports `odom` and `base_footprint` as disconnected frames.
4. **CI only validates x86_64.** GitHub's hosted runners don't match the target Jetson (aarch64) hardware — no current way to catch an ARM-specific dependency or build issue before deployment.

## Questions for today

1. Is there an actual standard ROS2 pattern for enforcing that a publisher and subscriber agree on a topic name, or is manual discipline genuinely how people handle this in practice? (Directly caused issues #1 and #2 above.)
2. What's your actual debugging approach for a disconnected TF tree — where do you start looking first?
3. Is there a standard way to keep a sim backend and a real-hardware backend honestly interchangeable behind the same topics, or is that also manual discipline?
4. What's the standard way to validate ARM-specific build/dependency issues before deploying to a Jetson — self-hosted runner on real hardware, QEMU cross-compilation in CI, or is that overkill until it's actually running there?
