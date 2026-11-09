import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    salamander_share = get_package_share_directory('salamander_planning')

    cenario_file = os.path.join(
        salamander_share, 'config', 'cenario_2.yaml'
    )

    '''manipulation_file = os.path.join(
        salamander_share, 'pddl', 'basic_transportation_test_1.pddl'
    )'''

    plansys2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                salamander_share, 'launch', 'plansys2.launch.py'
            )
        )
    )

    move_action_node = Node(
        package='salamander_planning',
        executable='move_action_node',
        name='move_action_node',
        parameters=[cenario_file],
    )

    align_action_node = Node(
        package='salamander_planning',
        executable='align_action_node',
        name='align_action_node',
        parameters=[cenario_file],
    )

    align_container_node = Node(
        package='salamander_planning',
        executable='align_container_node',
        name='align_container_node',
        parameters=[cenario_file],
    )

    stack_cube_base = Node(
        package='salamander_planning',
        executable='stack_cube_base_node',
        name='stack_cube_base_node',
    )

    stack_cube_top = Node(
        package='salamander_planning',
        executable='stack_cube_top_node',
        name='stack_cube_top_node',
    )

    vision_node = Node(
        package='robot_vision',
        executable='vision_node',
        name='vision_node',
    )

    arm_control = Node(
        package='arm_control',
        executable='control_node',
        name='control_node',
    )

    get_cube_table = Node(
        package='salamander_planning',
        executable='get_cube_table_node',
        name='get_cube_table_node',
    )

    get_cube_slot = Node(
        package='salamander_planning',
        executable='get_cube_slot_node',
        name='get_cube_slot_node',
    )

    put_cube_table = Node(
        package='salamander_planning',
        executable='put_cube_table_node',
        name='put_cube_table_node',
    )

    put_cube_slot = Node(
        package='salamander_planning',
        executable='put_cube_slot_node',
        name='put_cube_slot_node',
    )

    reset = Node(
        package='salamander_planning',
        executable='reset_arm_node',
        name='reset_arm_node',
    )

    zero_position = Node(
        package='salamander_planning',
        executable='zero_position_node',
        name='zero_position_node',
    )

    return LaunchDescription([
        plansys2_launch,
        move_action_node,
        align_action_node,
        align_container_node,
        stack_cube_base,
        stack_cube_top,
        vision_node,
        arm_control,
        get_cube_table,
        get_cube_slot,
        put_cube_table,
        put_cube_slot,
        reset,
        zero_position
    ])