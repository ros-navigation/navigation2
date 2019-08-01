import os
from launch import LaunchDescription
import launch.actions
import launch_ros.actions
from ament_index_python.packages import get_package_prefix

def generate_launch_description():
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time', default='false')
    params_file = launch.substitutions.LaunchConfiguration('params', default=
        [launch.substitutions.ThisLaunchFileDir(), '/nav2_params.yaml'])
   
    bt_navigator_install_path = get_package_prefix('nav2_bt_navigator')
    bt_navigator_xml = os.path.join(bt_navigator_install_path,
                                    'behavior_trees',
                                    'navigate_w_recovery_retry.xml') # TODO(mkhansen): change to an input parameter

    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        launch_ros.actions.Node(
            package='nav2_world_model',
            node_executable='world_model',
            output='screen',
            parameters=[params_file]),

        launch_ros.actions.Node(
            package='dwb_controller',
            node_executable='dwb_controller',
            output='screen',
            parameters=[params_file]),

        launch_ros.actions.Node(
            package='nav2_navfn_planner',
            node_executable='navfn_planner',
            node_name='navfn_planner',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='nav2_recoveries',
            node_executable='recoveries_node',
            node_name='recoveries',
            output='screen',
            parameters=[{ 'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='nav2_bt_navigator',
            node_executable='bt_navigator',
            node_name='bt_navigator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, {'bt_xml_filename': bt_navigator_xml}])

    ])
