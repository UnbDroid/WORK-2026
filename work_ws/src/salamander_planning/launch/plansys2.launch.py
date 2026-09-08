import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # Mapeia o diretório do pacote da equipe DROID
    salamander_pkg_dir = get_package_share_directory('salamander_planning')

    # Aponta para o arquivo de domínio (escalável para o futuro)
    domain_file = os.path.join(salamander_pkg_dir, 'pddl', 'domain.pddl')

    # Aciona o motor do PlanSys2 subindo os 4 nós centrais
    plansys2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('plansys2_bringup'),
            'launch',
            'plansys2_bringup_launch.py'
        )),
        launch_arguments={
            'model_file': domain_file
        }.items()
    )

    return LaunchDescription([
        plansys2_cmd
    ])