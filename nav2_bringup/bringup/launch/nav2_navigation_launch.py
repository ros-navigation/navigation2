# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from nav2_common.launch import RewrittenYaml
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

import launch.actions
import launch_ros.actions


def generate_launch_description():
    robot_name = launch.substitutions.LaunchConfiguration('robot_name')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    autostart = launch.substitutions.LaunchConfiguration('autostart')
    params_file = launch.substitutions.LaunchConfiguration('params_file')
    bt_xml_file = launch.substitutions.LaunchConfiguration('bt_xml_file')
    use_lifecycle_mgr = launch.substitutions.LaunchConfiguration('use_lifecycle_mgr')
    use_remappings = launch.substitutions.LaunchConfiguration('use_remappings')

    # TODO(orduno) Remove once `PushNodeRemapping` is resolved
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [((robot_name, '/tf'), '/tf'),
                  ((robot_name, '/tf_static'), '/tf_static'),
                  ('/scan', 'scan'),
                  ('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/cmd_vel', 'cmd_vel'),
                  ('/map', 'map')]

    # Create our own temporary YAML files that include substitutions
    namespace_substitutions = {'robot_name': robot_name}

    param_substitutions = {
        'use_sim_time': use_sim_time,
        'bt_xml_filename': bt_xml_file,
        'autostart': autostart,
        'map_subscribe_transient_local': 'False'
    }

    configured_params = RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            key_rewrites=namespace_substitutions,
            convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        launch.actions.SetEnvironmentVariable(
            'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        launch.actions.DeclareLaunchArgument(
            'robot_name', default_value='',
            description='Identification name for the robot'),

        launch.actions.DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        launch.actions.DeclareLaunchArgument(
            'autostart', default_value='true',
            description='Automatically startup the nav2 stack'),

        launch.actions.DeclareLaunchArgument(
            'params_file',
            default_value=[launch.substitutions.ThisLaunchFileDir(),
                           '/nav2_params.yaml'],
            description='Full path to the ROS2 parameters file to use'),

        launch.actions.DeclareLaunchArgument(
            'bt_xml_file',
            default_value=os.path.join(get_package_prefix('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            description='Full path to the behavior tree xml file to use'),

        launch.actions.DeclareLaunchArgument(
            'use_lifecycle_mgr', default_value='true',
            description='Whether to launch the lifecycle manager'),

        launch.actions.DeclareLaunchArgument(
            'use_remappings', default_value='false',
            description='Arguments to pass to all nodes launched by the file'),

        launch_ros.actions.Node(
            condition=UnlessCondition(use_remappings),
            package='nav2_world_model',
            node_executable='world_model',
            output='screen',
            parameters=[configured_params]),

        # TODO(orduno) Remove once `PushNodeRemapping` is resolved
        #              https://github.com/ros2/launch_ros/issues/56
        launch_ros.actions.Node(
            condition=IfCondition(use_remappings),
            package='nav2_world_model',
            node_executable='world_model',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        launch_ros.actions.Node(
            condition=UnlessCondition(use_remappings),
            package='dwb_controller',
            node_executable='dwb_controller',
            output='screen',
            parameters=[configured_params]),

        # TODO(orduno) Remove once `PushNodeRemapping` is resolved
        #              https://github.com/ros2/launch_ros/issues/56
        launch_ros.actions.Node(
            condition=IfCondition(use_remappings),
            package='dwb_controller',
            node_executable='dwb_controller',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        launch_ros.actions.Node(
            condition=UnlessCondition(use_remappings),
            package='nav2_planner',
            node_executable='planner_server',
            node_name='planner_server',
            output='screen',
            parameters=[configured_params]),

        # TODO(orduno) Remove once `PushNodeRemapping` is resolved
        #              https://github.com/ros2/launch_ros/issues/56
        launch_ros.actions.Node(
            condition=IfCondition(use_remappings),
            package='nav2_navfn_planner',
            node_executable='navfn_planner',
            node_name='navfn_planner',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        launch_ros.actions.Node(
            condition=UnlessCondition(use_remappings),
            package='nav2_recoveries',
            node_executable='recoveries_node',
            node_name='recoveries',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        # TODO(orduno) Remove once `PushNodeRemapping` is resolved
        #              https://github.com/ros2/launch_ros/issues/56
        launch_ros.actions.Node(
            condition=IfCondition(use_remappings),
            package='nav2_recoveries',
            node_executable='recoveries_node',
            node_name='recoveries',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=remappings),

        launch_ros.actions.Node(
            condition=UnlessCondition(use_remappings),
            package='nav2_bt_navigator',
            node_executable='bt_navigator',
            node_name='bt_navigator',
            output='screen',
            parameters=[configured_params]),

        # TODO(orduno) Remove once `PushNodeRemapping` is resolved
        #              https://github.com/ros2/launch_ros/issues/56
        launch_ros.actions.Node(
            condition=IfCondition(use_remappings),
            package='nav2_bt_navigator',
            node_executable='bt_navigator',
            node_name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            remappings=remappings),

        launch_ros.actions.Node(
            condition=IfCondition(use_lifecycle_mgr),
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            node_name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': ['dwb_controller',
                                        'planner_server',
                                        'bt_navigator']}]),

    ])
