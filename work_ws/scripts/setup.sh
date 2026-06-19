#!/bin/bash

# se qualquer comando falhar, o script para
set -e

cd ~/WORK-2026/work_ws

git pull

source /opt/ros/jazzy/setup.bash

vcs import src < dependencies.repos

colcon build --symlink-install

source install/setup.bash

ros2 run micro_ros_setup build_agent.sh
