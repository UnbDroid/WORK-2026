import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


# display.launch.py agora abre APENAS o RViz.
# O robot_state_publisher e o joint_state_publisher foram para o odomLidar.launch.py,
# que roda junto com o robô. Assim:
#  - não existem dois robot_state_publisher publicando a mesma TF;
#  - as juntas (rodas e manipulador) existem desde o início, mesmo sem abrir o RViz;
#  - dá para rodar este launch no notebook (RViz fora do PC do robô = menos CPU no robô).


def generate_launch_description():
    share_dir = get_package_share_directory('salamander_bot')
    rviz_config_file = os.path.join(share_dir, 'config', 'caldo.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )

    return LaunchDescription([
        rviz_node,
    ])