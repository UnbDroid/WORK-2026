import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro


def generate_launch_description():
    salamander_share = get_package_share_directory('salamander_bot')
    sllidar_share = get_package_share_directory('sllidar_ros2')
    rf2o_share = get_package_share_directory('rf2o_laser_odometry')

    robot_description_file = os.path.join(salamander_share, 'description', 'robot.urdf.xacro')
    
    robot_description_config = xacro.process_file(robot_description_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    config_dir = os.path.join(get_package_share_directory('salamander_bot'), 'config', 'robot_body_filter.yaml')

    robot_state_publisher = Node(
        package='robot_state_publisher',    
        executable='robot_state_publisher',
        name='state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_share, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'frame_id': 'rplidar_a1m8_1',
            'serial_port': '/dev/ttyUSB1', # Agora o lidar está configurado para usar a porta USB0
        }.items()
    )

    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        parameters=[config_dir],
        remappings=[
            ('scan', '/scan'),
            ('scan_filtered', '/scan_filtered')
        ],
        output='screen'
    )

    rf2o = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rf2o_share, 'launch', 'rf2o_laser_odometry.launch.py')
                ),
            )
        ]
    )

    return LaunchDescription([
        lidar,
        laser_filter_node,
        robot_state_publisher,
        rf2o,
    ])
