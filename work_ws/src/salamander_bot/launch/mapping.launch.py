import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch_ros.actions import Node

import xacro

''' Sobe:
        robot_state_publisher
        odom_node
        ekf_node
        slam_toolbox
'''

def generate_launch_description():
    salamander_share = get_package_share_directory('salamander_bot')

    robot_description_path = os.path.join(
        salamander_share,
        'description',
        'robot.urdf.xacro'
    )

    ekf_config_path = os.path.join(
        salamander_share,
        'config',
        'ekf.yaml'
    )

    slam_config_path = os.path.join(
        salamander_share,
        'config',
        'mapper_params_online_async.yaml'
    )

    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch_path = os.path.join(
        slam_toolbox_share,
        'launch',
        'online_async_launch.py'
    )

    robot_description = xacro.process_file(robot_description_path).toxml()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {'robot_description': robot_description}
            ],
            output='screen'
        ),

        Node(
            package='robot_odometry',
            executable='odom_node',
            name='odom_node',
            output='screen'
        ),

        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config_path],
            output='screen'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_path),
            launch_arguments={
                'slam_params_file': slam_config_path,
                'use_sim_time': 'false',
            }.items()
        )
    ])
