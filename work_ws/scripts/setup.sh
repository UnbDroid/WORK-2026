#!/bin/bash

# se qualquer comando falhar, o script para
set -e 

cd ~/WORK-2026

git checkout main

git pull

cd work_ws

source /opt/ros/jazzy/setup.bash

if [ -f dependencies.repos ]; then
  vcs import src < dependencies.repos
fi

rosdep install --from-paths src --ignore-src -y

colcon build --symlink-install

source install/setup.bash

ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh

sudo apt install ros-jazzy-xacro ros-jazzy-joint-state-publisher-gui 

sudo apt install ros-jazzy-ros-gz
