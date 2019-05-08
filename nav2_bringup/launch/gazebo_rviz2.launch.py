import os
import sys
from launch import LaunchDescription
from launch.substitutions import EnvironmentVariable
import launch.actions
import launch_ros.actions

def generate_launch_description():
    world = launch.substitutions.LaunchConfiguration('world')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'world', description='Full path to world file to load'),
        
        launch.actions.ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', world],
            output='screen'),

        launch_ros.actions.Node(package='rviz2', node_executable='rviz2', output='screen')
    ])
