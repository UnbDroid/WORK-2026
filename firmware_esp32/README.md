# ESP32 + Micro-ROS Mecanum Robot — Setup & Teleoperation Guide

## Overview

This repository implements a **holonomic mecanum robot** controlled by an **ESP32 running Micro-ROS**, interfacing with a ROS 2 system (Raspberry Pi 5).

### System Capabilities

* Receives velocity commands via `/cmd_vel`;
* Executes mecanum kinematics;
* Controls 4 DC motors with encoders;
* Publishes wheel speed feedback via `/wheel_speeds`;
* Supports keyboard teleoperation.

## Software Stack

### On ESP32

* PlatformIO
* Micro-ROS
* Custom classes:
  * `MotorDC`
  * `Robot`

### On Raspberry PI

* ROS 2 (Jazzy)
  * `micro_ros_agent`
  * `teleop_twist_keyboard`

## System Architecture

```bash
[Keyboard Teleop]
        │
        ▼
[/cmd_vel] (Twist)
        │
        ▼
[Micro-ROS Agent] (Serial)
        │ 
        ▼
[ESP32 Node]
   ├── Subscriber: /cmd_vel
   ├── Motor Control (PI)
   └── Publisher: /wheel_speeds
```

## Setup Instructions

### 0. Initialize Container

```bash
# Install Distrobox:
sudo apt-get install distrobox podman

# Create a Ubuntu 24.04 Container:
distrobox create --name ubuntu --image ubuntu:24.04

# Access Ubuntu Container:
distrobox enter ubuntu
```

### 1. Start Micro-ROS Agent

```bash
# Initialize ROS2:
source /opt/ros/jazzy/setup.bash

# Access ROS2 Micro-ROS Directory & Initialize Local Setup:
cd robot_ws
source install/local_setup.bash

# Run Serial (USB) Communication:
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0 -v6
```

### 2. Flash ESP32 Firmware

Using PlatformIO:

```bash
pio run --target upload
```

### 3. Verify Node Connection

```bash
ros2 node list
```

Expected:

```bash
/esp32_mecanum_node
```

### 4. Verify Topic Interfaces

```bash
ros2 topic list
```

Expected:

```bash
/cmd_vel
/wheel_speeds
```

## Teleoperation

### Run Keyboard Teleop

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Default Controls

| Key          | Action                                  |
| ------------ | --------------------------------------- |
| i            | Forward                                 |
| ,            | Backward                                |
| j            | Rotate left                             |
| l            | Rotate right                            |
| u/o/m/.      | Diagonal (holonomic)                    |
| SHIFT + keys | Holonomic mode (enables lateral motion) |

---

## Control Pipeline

```bash
cmd_vel → [vx, vy, wz]
        ↓
Robot::moveRobot()
        ↓
Wheel speeds [ω1, ω2, ω3 ω4]
        ↓
MotorDC PI control
        ↓
PWM signals
```

## Debugging Guide

### 1. Check `/cmd_vel`

```bash
ros2 topic echo /cmd_vel
```

### 2. Check `/wheel_speeds`

```bash
ros2 topic echo /wheel_speeds
```
