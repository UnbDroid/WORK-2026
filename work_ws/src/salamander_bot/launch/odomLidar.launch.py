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

    robot_description_path = os.path.join(
        salamander_share,
        'description',
        'robot.urdf.xacro'
    )
    robot_description = xacro.process_file(robot_description_path).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sllidar_share, 'launch', 'sllidar_a1_launch.py')
        ),
        launch_arguments={
            'frame_id': 'rplidar_a1m8_1',
            'serial_port': '/dev/ttyUSB1', # Agora o lidar está configurado para usar a porta USB1
        }.items()
    )

    rf2o = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(rf2o_share, 'launch', 'rf2o_laser_odometry.launch.py')
                )
            )
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        lidar,
        rf2o,
    ])
