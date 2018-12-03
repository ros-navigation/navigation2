import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        launch_ros.actions.Node(
            package='nav2_world_model',
            node_executable='world_model',
            node_name='world_model',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='dwb_controller',
            node_executable='dwb_controller',
            node_name='FollowPathNode',
            output='screen',
            parameters=[{ 'prune_plan': False }, {'debug_trajectory_details': True }, { 'use_sim_time': use_sim_time }]),

        launch_ros.actions.Node(
            package='nav2_navfn_planner',
            node_executable='navfn_planner',
            node_name='navfn_planner',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='nav2_simple_navigator',
            node_executable='simple_navigator',
            node_name='simple_navigator',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='nav2_mission_executor',
            node_executable='mission_executor',
            node_name='mission_executor',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

    ])
