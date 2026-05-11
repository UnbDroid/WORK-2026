# WORK\_ROS2\_WS

Standard Workspace for ROS 2 Development (Robot Operating System 2).

## Prerequisites

  * **Operational System:** Ubuntu 24.04 (Noble Numbat);
  * **ROS 2 Version:** Jazzy Jalisco.

## Build Workflow:

``` bash
source /opt/ros/jazzy/setup.bash

rosdep install --from-paths src --ignore-src -y

colcon build --symlink-install
source install/local_setup.bash
```

## Micro-ROS Agent Setup in New Machines:

``` bash
source /opt/ros/jazzy/setup.bash

# Fetch Dependencies:
vcs import src < micro_ros.repos

# Install Dependencies:
rosdep install --from-paths src --ignore-src -y

# Build and Initialize Robot Workspace:
colcon build
source install/local_setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

## Simulation (URDF, Rviz, Gazebo) - Launching & Setup:

``` bash
# Install URDF & Rviz Dependencies (If Not Yet Installed): 
sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui 

# Install Gazebo (If Not Yet Installed):
sudo apt install ros-jazzy-ros-gz

# Run Python Script For Rviz Visualization:
ros2 launch salamander_bot display.launch.py

# Run Python Script For Gazebo Simulation:
ros2 launch salamander_bot gazebo.launch.py
```

* if you build your workspace with colcon build --symlink-install then you don’t need to rebuild every time you update the URDF, it will be automatic EXCEPT whenever you add a new file. That file needs to get built/copied once, and after that will be kept updated.

* You’ll need to quit and relaunch display.launch.py (or gazebo.launch.py) each time you make a change.

* RViz sometimes won’t pick up all your changes straight away. In this case you'll need to hit the “Reset” button in the bottom corner. If it’s still not updating, try ticking and unticking the display items, or closing and reopening the program.
