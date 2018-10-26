import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions

def generate_launch_description():
    mapFile = os.path.join(os.getenv('TEST_LAUNCH_DIR'), 'test_map.yaml')

    return LaunchDescription([
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='nav2_map_server', node_executable='map_server', output='screen', arguments=[[mapFile], 'occupancy'])
                ], period = 1.0),
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='nav2_controller_dwb', node_executable='nav2_controller_dwb', output='screen')
                ], period = 5.0),
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='nav2_dijkstra_planner', node_executable='dijkstra_planner', output='screen')
                ], period = 10.0),
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='nav2_simple_navigator', node_executable='simple_navigator', output='screen')
                ], period = 15.0),
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='nav2_mission_executor', node_executable='mission_executor', output='screen')
                ], period = 20.0),
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='nav2_costmap_world_model', node_executable='costmap_world_model', output='screen')
                ], period = 25.0),
        launch.actions.TimerAction(
            actions = [
                launch_ros.actions.Node( package='nav2_amcl', node_executable='amcl', output='screen')
                ], period = 30.0),
    ])
