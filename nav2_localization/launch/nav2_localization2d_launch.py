from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='nav2_localization',
            namespace='nav2_localization',
            executable='nav2_localization',
            name='nav2_localization_node'
        )
    ])
