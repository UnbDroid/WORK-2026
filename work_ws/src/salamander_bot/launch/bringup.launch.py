import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap

def generate_launch_description():
    salamander_dir = get_package_share_directory('salamander_bot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup') 
    
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    default_params_path = os.path.join(salamander_dir, 'config', 'nav2_params.yaml')
    default_map_path = os.path.join(salamander_dir, 'config', 'map_BTT.yaml')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map', default_value=default_map_path,
        description='Full path to map yaml file to load')

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

    # Usa o AMCL e o Map Server do Nav2
    amcl_launch_path = os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
    run_amcl_localization = GroupAction(
        actions=[
            SetRemap(src='/scan', dst='/scan_filtered'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(amcl_launch_path),
                launch_arguments={
                    'map': map_yaml_file,
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': 'true'
                }.items()
            )
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    
    ld.add_action(run_amcl_localization)
    ld.add_action(run_navigation_stack)
    return ld