# Localização com o slam_toolbox num mapa já gravado.
# É o "irmão" do mapping.launch.py: sobe a odometria (odomLidar) + slam_toolbox,
# mas em MODO LOCALIZAÇÃO, carregando o mapa serializado em vez de criar um novo.
#
# Uso:
#   ros2 launch salamander_bot localization.launch.py
# Depois, o MESMO launch de navegação que vocês usam com o mapping.
# O caminho do mapa fica no config/mapper_params_localization.yaml (map_file_name).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    salamander_share = get_package_share_directory('salamander_bot')
    slam_toolbox_share = get_package_share_directory('slam_toolbox')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_virtual_walls = LaunchConfiguration('use_virtual_walls')
    virtual_walls_yaml = LaunchConfiguration('virtual_walls_yaml')

    slam_params_path = os.path.join(salamander_share, 'config', 'mapper_params_localization.yaml')
    odom_lidar_launch_path = os.path.join(salamander_share, 'launch', 'odomLidar.launch.py')
    slam_localization_launch_path = os.path.join(slam_toolbox_share, 'launch', 'localization_launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'use_virtual_walls', default_value='true',
            description='Publica a máscara de paredes virtuais (fitas) no tópico /virtual_walls'),

        DeclareLaunchArgument(
            'virtual_walls_yaml',
            default_value=os.path.join(salamander_share, 'maps', 'virtual_walls.yaml'),
            description='YAML da máscara gerada pelo gerar_paredes_virtuais.py'),

        # Odometria (rf2o), lidar, filtro e robot_state_publisher: igual ao modo mapping
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(odom_lidar_launch_path)
        ),

        # slam_toolbox em MODO LOCALIZAÇÃO (substitui AMCL + map_server)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_localization_launch_path),
            launch_arguments={
                'slam_params_file': slam_params_path,
                'use_sim_time': use_sim_time,
            }.items()
        ),

        # Paredes virtuais: um map_server separado publica a máscara em /virtual_walls.
        # Os costmaps a leem como uma segunda camada estática (ver nav2_params.yaml).
        Node(
            condition=IfCondition(use_virtual_walls),
            package='nav2_map_server',
            executable='map_server',
            name='virtual_walls_server',
            output='screen',
            parameters=[{
                'yaml_filename': virtual_walls_yaml,
                'topic_name': 'virtual_walls',
                'frame_id': 'map',
                'use_sim_time': use_sim_time,
            }],
        ),
        Node(
            condition=IfCondition(use_virtual_walls),
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_virtual_walls',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['virtual_walls_server'],
                'use_sim_time': use_sim_time,
            }],
        ),
    ])