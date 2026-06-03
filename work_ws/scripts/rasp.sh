#!/bin/bash

# se qualquer comando falhar, o script para
set -e 

# ssh droid@192.168.1.200
# distrobox enter ubuntu

# Reorganizar pastas na rasp --> atualmente está com teste_work
cd ~/WORK-2026

git checkout main

git pull

cd work_ws

source /opt/ros/jazzy/setup.bash

colcon build --symlink-install

source install/local_setup.bash

ros2 run micro_ros_setup build_agent.sh

sudo chmod 777 /dev/ttyUSB0

ros2 launch salamander_bot odomLidar.launch.py
