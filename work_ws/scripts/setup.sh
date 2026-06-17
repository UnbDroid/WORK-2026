#!/bin/bash

# se qualquer comando falhar, o script para
set -e



#git checkout main

#git pull


source /opt/ros/jazzy/setup.bash

vcs import src < dependencies.repos

#rosdep install --from-paths src --ignore-src -y

colcon build --symlink-install

source install/setup.bash

#ros2 run micro_ros_setup create_agent_ws.sh
#ros2 run micro_ros_setup build_agent.sh
