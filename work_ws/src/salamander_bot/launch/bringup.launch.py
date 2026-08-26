import os

# Importa utilitários do ROS 2 para gerenciar pacotes instalados
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Pega o diretório de instalação (share) do seu pacote de simulação do robô
    salamander_dir = get_package_share_directory('salamander_bot')
    
    # Configurações de argumentos locais
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    # Caminho padrão para os parâmetros 
    default_params_path = os.path.join(salamander_dir, 'config', 'nav2_params.yaml')
    default_map_path = os.path.join(salamander_dir, 'maps', 'sala.yaml')

    # Declara os argumentos de inicialização que o terminal aceitará
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')


    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=default_params_path,
        description='Full path to the ROS2 parameters file to use')

    nav_launch_path = os.path.join(salamander_dir, 'launch', 'navigation_launch.py')
    run_navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    slam_toolbox_share = get_package_share_directory('slam_toolbox')
    slam_toolbox_launch_path = os.path.join(
            slam_toolbox_share,
            'launch',
            'localization_launch.py'
        )
    slam_localization = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_toolbox_launch_path),
                launch_arguments={
                    'slam_params_file': os.path.join(salamander_dir, 'config', 'slam_toolbox_localization.yaml'),
                    'use_sim_time': use_sim_time,
                }.items()
            )

    # Cria a descrição de lançamento mestre que agrupa todas as ações criadas acima
    ld = LaunchDescription()


    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    
    
    # Inicializa todos os servidores de navegação (MPPI Controller, Planner, Behaviors, etc.)
    ld.add_action(run_navigation_stack)
    ld.add_action(slam_localization)
    return ld