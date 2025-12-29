# ROS2 Virtual Odometry (Terminal-based)

This repository provides a simple **virtual odometry example for ROS 2 (Humble)**.  
It is designed to help engineers understand the relationship between:

- `/cmd_vel` (robot-centric velocity commands)
- `/odom` (world-centric odometry)
- coordinate frames and interpretation

This project intentionally avoids GUI tools such as RViz and focuses on  
**terminal-based understanding**.

---

## Overview

This package contains two ROS 2 nodes:

### 1. `virtual_odometry_node`
- Subscribes to `/cmd_vel`
- Integrates linear and angular velocity over time
- Publishes virtual `/odom`
- No real sensors or hardware required

### 2. `odom_analyzer_node`
- Subscribes to `/odom`
- Interprets motion from the robot's point of view
- Displays movement state in a single terminal line:
  - FORWARD / BACKWARD
  - LEFT / RIGHT
  - current position and yaw

---

## Package Structure

```
virtual_odometry/
├── virtual_odometry/
│   ├── virtual_odometry_node.py
│   └── odom_analyzer_node.py
├── package.xml
├── setup.py
└── resource/
```

---

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- `teleop_twist_keyboard`

Install teleop if needed:

```bash
sudo apt update
sudo apt install ros-humble-teleop-twist-keyboard
```

---

## Build

From your ROS 2 workspace root:

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## Run (3 Terminals)

### Terminal 1: Virtual Odometry

```bash
ros2 run virtual_odometry virtual_odometry_node
```

---

### Terminal 2: Odometry Analyzer

```bash
ros2 run virtual_odometry odom_analyzer_node
```

You will see a continuously updated single-line status output.

---

### Terminal 3: Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Use keyboard input to control the robot.

---

## What to Observe

- Press forward key → `move=FORWARD`, x increases
- Press backward key → `move=BACKWARD`, x decreases
- Rotate left → `turn=LEFT`, yaw increases
- Rotate right → `turn=RIGHT`, yaw decreases

This confirms the relationship between `/cmd_vel` and `/odom`.

---

## Purpose

This project is intended for:

- ROS 2 beginners
- Engineers who want to **understand motion and odometry before using RViz**
- Terminal-based learning and debugging
- Educational articles and tutorials

---
## Related Article (Japanese)

This repository is used in the following article series: <br/>
[https://independence-sys.net/main/?p=7363](*ROS2で「動かす」ために押さえておきたい考え方*) <br/>
[https://independence-sys.net/main/?p=7363](*ROS2 仮想オドメトリをターミナルで理解する*) <br/>

## License

MIT License
