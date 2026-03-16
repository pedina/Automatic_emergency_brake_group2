from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    debug_arg = DeclareLaunchArgument(
        'debug',
        default_value='false',
        description='Whether debug messages should be shown.'
    )

    control_publisher = Node(
        package='ctrl_long_emergency',
        executable='ctrl_long_emergency',
        name='ctrl_long_emergency',
        output='screen',
        parameters=[
            {
                'debug_enabled': LaunchConfiguration('debug')
            }
        ]
    )

    return LaunchDescription([
        debug_arg,

        control_publisher
    ])