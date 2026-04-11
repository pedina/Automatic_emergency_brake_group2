from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Döntéshozó Node
        Node(
            package='behavior_planner',
            executable='behavior_planner_node',
            name='behavior_planner',
            output='screen' # Így a logok egy terminálba ömlenek
        ),
        # 2. Tervező Node (A te kódod)
        Node(
            package='plan_long_emergency',
            executable='plan_long_emergency_node',
            name='plan_long_emergency',
            output='screen'
        ),
        # 3. Végrehajtó (Control) Node
        Node(
            package='ctrl_long_emergency',
            executable='ctrl_long_emergency',
            name='ctrl_long_emergency',
            output='screen'
        )
    ])
