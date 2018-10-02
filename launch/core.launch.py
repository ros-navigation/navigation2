import os
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    mapFile = os.path.join(os.getenv('TEST_LAUNCH_DIR'), 'test_map.yaml')

    return LaunchDescription([
        launch_ros.actions.Node(
            package='nav2_map_server', node_executable='map_server', output='screen', arguments=[[mapFile], 'occupancy']),
        launch_ros.actions.Node(
            package='nav2_dwa_controller', node_executable='dwa_controller', output='screen'),
        launch_ros.actions.Node(
            package='nav2_simple_navigator', node_executable='simple_navigator', output='screen'),
        launch_ros.actions.Node(
            package='nav2_mission_executor', node_executable='mission_executor', output='screen'),
        launch_ros.actions.Node(
            package='nav2_costmap_world_model', node_executable='costmap_world_model', output='screen'),
        launch_ros.actions.Node(
            package='nav2_dijkstra_planner', node_executable='dijkstra_planner', output='screen'),
        launch_ros.actions.Node(
            package='nav2_amcl', node_executable='amcl', output='screen'),
    ])

