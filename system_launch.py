from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. A Behavior Planner node indítása
        Node(
            package='behavior_planner',
            executable='behavior_planner_node',
            name='behavior_planner',
            output='screen' # Így a logok egy terminálba ömlenek
        ),
        # 2. A Plan Long Emergency node indítása
        Node(
            package='plan_long_emergency',
            executable='plan_long_emergency_node',
            name='plan_long_emergency',
            output='screen'
        ),
        # 3. A Control Long Emergency node indítása
        Node(
            package='ctrl_long_emergency',
            executable='ctrl_long_emergency_node',
            name='ctrl_long_emergency',
            output='screen'
        )
    ])
