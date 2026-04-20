# WORK\_ROS2\_WS

Standard Workspace for ROS 2 Development (Robot Operating System 2).

## Prerequisites

  * **Operational System:** Ubuntu 24.04 (Noble Numbat);
  * **ROS 2 Version:** Jazzy Jalisco.

## Micro-ROS Agent Setup in New Machines:

``` bash
cd robot_ws
source /opt/ros/jazzy/setup.bash

# Fetch Dependencies:
vcs import src < micro_ros.repos

# Install Dependencies:
rosdep install --from-paths src --ignore-src -y

# Build and Initialize Robot Workspace:
colcon build
source install/setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

## Build Workflow

### On Development Machine:

``` bash
cd dev_ws
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -y

colcon build --symlink-install
source install/setup.bash
```

### On Robot

``` bash
cd robot_ws
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -y

colcon build --symlink-install
source install/setup.bash

ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
```
