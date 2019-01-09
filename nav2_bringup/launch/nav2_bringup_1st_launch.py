import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    map_yaml_file = launch.substitutions.LaunchConfiguration('map')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'map', description='Full path to map file to load'),
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        launch_ros.actions.Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}, { 'yaml_filename': map_yaml_file }]),

        launch_ros.actions.Node(
            package='nav2_amcl',
            node_executable='amcl',
            node_name='amcl',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

    ])
