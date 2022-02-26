import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    lifecycle_nodes = ['map_server']

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            prefix=['xterm -e gdb -ex run --args'],
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': "/home/josh/nav2_ws/src/navigation2/nav2_bringup/maps/turtlebot3_world.yaml"},
                        {'topic_name': "map"}]),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}])
    ])