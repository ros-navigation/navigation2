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

    stdout_linebuf_envvar = launch.actions.SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }
    configured_params = RewrittenYaml(
        source_file=params_file, rewrites=param_substitutions,
        convert_types=True)
    
    # Declare the launch arguments
    declare_map_yaml_cmd = launch.actions.DeclareLaunchArgument(
        'map',
        default_value='test_map.yaml',
        description='Full path to map file to load')

    declare_use_sim_time_cmd = launch.actions.DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = launch.actions.DeclareLaunchArgument(
        'params',
        default_value='nav2_params.yaml',
        description='Full path to the ROS2 parameters file to use for all launched nodes')
        
    start_map_server_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_map_server'),
                'lib/nav2_map_server/map_server'),
            ['__params:=', configured_params]],
        cwd=[launch_dir], output='screen')

    start_localizer_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_amcl'),
                'lib/nav2_amcl/amcl'),
            ['__params:=', configured_params]],
        cwd=[launch_dir], output='screen')

    start_world_model_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_world_model'),
                'lib/nav2_world_model/world_model'),
            ['__params:=', configured_params]],
        cwd=[launch_dir], output='screen')

    start_dwb_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('dwb_controller'),
                'lib/dwb_controller/dwb_controller'),
            ['__params:=', configured_params]],
        cwd=[launch_dir], output='screen')

    start_planner_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_navfn_planner'),
                'lib/nav2_navfn_planner/navfn_planner'),
            ['__params:=', configured_params]],
        cwd=[launch_dir], output='screen')

    start_navigator_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_bt_navigator'),
                'lib/nav2_bt_navigator/bt_navigator'),
            ['__params:=', configured_params]],
        cwd=[launch_dir], output='screen')

    start_lifecycle_manager_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_lifecycle_manager'),
                'lib/nav2_lifecycle_manager/lifecycle_manager'),
            ['__params:=', configured_params]],
        cwd=[launch_dir], output='screen')

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    
    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_localizer_cmd)
    ld.add_action(start_world_model_cmd)
    ld.add_action(start_dwb_cmd)
    ld.add_action(start_planner_cmd)
    ld.add_action(start_navigator_cmd)

    return ld
