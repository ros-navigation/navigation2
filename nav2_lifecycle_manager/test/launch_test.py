import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription

from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription([
    Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': 'false'},
                    {'autostart': 'false'},
                    {'node_names': 'lifecycle_node_test'}]),

    Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{'use_sim_time': 'false'},
                    {'autostart': 'false'},
                    {'node_names': 'lifecycle_node_test'}])
  ])

