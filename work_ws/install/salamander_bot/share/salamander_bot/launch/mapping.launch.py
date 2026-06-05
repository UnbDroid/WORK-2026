import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

def generate_launch_description():
    salamander_share = get_package_share_directory('salamander_bot')

    # DECLARADO: Argumento para controlar o tempo de simulação de forma dinâmica
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

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
        # Declaração do argumento para o sistema
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odom_lidar_launch_path)
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_toolbox_launch_path),
            launch_arguments={
                'slam_params_file': slam_config_path,
                'use_sim_time': use_sim_time, # MODIFICADO: Agora aceita dinamicamente true ou false
            }.items()
        )
    ])