import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription

''' Sobe:
        odomLidar
        slam_toolbox
'''

def generate_launch_description():
    salamander_share = get_package_share_directory('salamander_bot')

    slam_config_path = os.path.join(
        salamander_share,
        'config',
        'mapper_params_online_async.yaml'
    )

    odom_lidar_launch_path = os.path.join(
        salamander_share,
        'launch',
        'odomLidar.launch.py'
    )

    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch_path = os.path.join(
        slam_toolbox_share,
        'launch',
        'online_async_launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odom_lidar_launch_path)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_path),
            launch_arguments={
                'slam_params_file': slam_config_path,
                'use_sim_time': 'false',
            }.items()
        )
    ])
