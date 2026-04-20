from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('behavior_planner'),
        'config', 'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='behavior_planner',
            executable='behavior_planner',
            name='behavior_planner',
            parameters=[params_file]
        ),
        Node(
            package='plan_long_emergency',
            executable='plan_long_emergency_node',
            name='plan_long_emergency',
            parameters=[params_file]
        ),
        Node(
            package='ctrl_long_emergency',
            executable='ctrl_long_emergency',
            name='ctrl_long_emergency',
            parameters=[params_file]
        ),
    ])