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
    default_map_path = os.path.join(salamander_dir, 'launch', 'mapper_params_online_async.yaml')

    # Declara os argumentos de inicialização que o terminal aceitará
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='Full path to map yaml file to load')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file', default_value=default_params_path,
        description='Full path to the ROS2 parameters file to use')

    # Lista de nós gerenciados que pertencem à etapa de Localização do robô
    localization_nodes = ['map_server', 'amcl']
    
    #Cria o nó do Map Server: responsável por ler o arquivo YAML e publicar o mapa no tópico /map
    run_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'yaml_filename': map_yaml_file}]
    )

    # Cria o nó do AMCL: lê o sensor laser e estima onde o robô está no mapa global
    run_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}]
    )

    # Gerenciador de ciclo de vida dedicado para carregar a localização primeiro
    run_lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': localization_nodes}]
    )

    # Inclui o seu arquivo de navegação existente (o que carrega o MPPI, Planner, etc.)
    # e repassa os parâmetros lidos por este script para dentro dele
    navigation_launch_path = os.path.join(salamander_dir, 'launch', 'navigation_launch.py')
    run_navigation_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_path),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )

    # Cria a descrição de lançamento mestre que agrupa todas as ações criadas acima
    ld = LaunchDescription()


    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    
    # Inicializa a infraestrutura de localização (Servidor de mapa, AMCL e seu Gerenciador)
    ld.add_action(run_map_server)
    ld.add_action(run_amcl)
    ld.add_action(run_lifecycle_manager_localization)
    
    # Inicializa todos os servidores de navegação (MPPI Controller, Planner, Behaviors, etc.)
    ld.add_action(run_navigation_stack)

    return ld