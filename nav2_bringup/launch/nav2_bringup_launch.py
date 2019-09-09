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
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition

import launch.actions
import launch_ros.actions

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get the launch directory
    launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Create the launch configuration variables
    map_yaml_file = launch.substitutions.LaunchConfiguration('map')
    use_sim_time = launch.substitutions.LaunchConfiguration('use_sim_time')
    params_file = launch.substitutions.LaunchConfiguration('params')
    bt_xml_file = launch.substitutions.LaunchConfiguration('bt_xml_file')
    autostart = launch.substitutions.LaunchConfiguration('autostart')

    stdout_linebuf_envvar = launch.actions.SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file,
        'bt_xml_filename': bt_xml_file,
        'autostart': autostart
    }

    configured_params = RewrittenYaml(
        source_file=params_file, rewrites=param_substitutions,
        convert_types=True)

    # Declare the launch arguments
    declare_map_yaml_cmd = launch.actions.DeclareLaunchArgument(
        'map',
        default_value=os.path.join(launch_dir, 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    declare_use_sim_time_cmd = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = launch.actions.DeclareLaunchArgument(
        'params',
        default_value=os.path.join(launch_dir, 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = launch.actions.DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_bt_xml_cmd = launch.actions.DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(get_package_prefix('nav2_bt_navigator'),
            'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    start_map_server_cmd = launch_ros.actions.Node(
        package='nav2_map_server',
        node_executable='map_server',
        node_name='map_server',
        output='screen',
        parameters=[configured_params])

    start_localizer_cmd = launch_ros.actions.Node(
        package='nav2_amcl',
        node_executable='amcl',
        node_name='amcl',
        output='screen',
        parameters=[configured_params])

    start_world_model_cmd = launch_ros.actions.Node(
        package='nav2_world_model',
        node_executable='world_model',
        output='screen',
        parameters=[configured_params])

    start_dwb_cmd = launch_ros.actions.Node(
        package='dwb_controller',
        node_executable='dwb_controller',
        output='screen',
        parameters=[configured_params])

    start_planner_cmd = launch_ros.actions.Node(
        package='nav2_navfn_planner',
        node_executable='navfn_planner',
        node_name='navfn_planner',
        output='screen',
        parameters=[configured_params])

    start_recovery_cmd = launch_ros.actions.Node(
        package='nav2_recoveries',
        node_executable='recoveries_node',
        node_name='recoveries',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}])

    start_navigator_cmd = launch_ros.actions.Node(
        package='nav2_bt_navigator',
        node_executable='bt_navigator',
        node_name='bt_navigator',
        output='screen',
        parameters=[configured_params])

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        node_executable='lifecycle_manager',
        node_name='lifecycle_manager',
        output='screen',
        parameters=[configured_params])

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_bt_xml_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_localizer_cmd)
    ld.add_action(start_world_model_cmd)
    ld.add_action(start_dwb_cmd)
    ld.add_action(start_planner_cmd)
    ld.add_action(start_recovery_cmd)
    ld.add_action(start_navigator_cmd)

    return ld
