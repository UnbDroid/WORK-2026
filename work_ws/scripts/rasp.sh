#!/bin/bash

# se qualquer comando falhar, o script para
set -e 

cd ~/WORK-2026/work_ws

# Mostrar em qual branch estamos trabalhando
git branch 

git pull

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/local_setup.bash

ros2 run micro_ros_setup build_agent.sh

ros2 launch salamander_bot mapping.launch.py
