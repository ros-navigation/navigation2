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

import os, sys
sys.path.append(os.path.dirname(__file__))

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from nav2_common.launch import Node
from nav2_common.launch import RewrittenYaml
import nav2_bringup_common as common


def generate_launch_description():
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': common.use_sim_time,
        'bt_xml_filename': common.bt_xml_file,
        'autostart': common.autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    configured_params = RewrittenYaml(
            source_file=common.params_file,
            root_key=common.namespace,
            param_rewrites=param_substitutions,
            convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),
        
        common.declare_namespace_cmd,

        common.declare_use_sim_time_cmd,

        common.declare_autostart_cmd,

        common.declare_params_file_cmd,

        common.declare_bt_xml_cmd,

        common.declare_use_lifecycle_mgr_cmd,

        common.declare_use_remappings_cmd,

        DeclareLaunchArgument(
            'map_subscribe_transient_local', default_value='false',
            description='Whether to set the map subscriber QoS to transient local'),

        Node(
            package='nav2_controller',
            node_executable='controller_server',
            output='screen',
            parameters=[configured_params],
            use_remappings=IfCondition(common.use_remappings),
            remappings=common.remappings),

        Node(
            package='nav2_planner',
            node_executable='planner_server',
            node_name='planner_server',
            output='screen',
            parameters=[configured_params],
            use_remappings=IfCondition(common.use_remappings),
            remappings=common.remappings),

        Node(
            package='nav2_recoveries',
            node_executable='recoveries_server',
            node_name='recoveries_server',
            output='screen',
            parameters=[{'use_sim_time': common.use_sim_time}],
            use_remappings=IfCondition(common.use_remappings),
            remappings=common.remappings),

        Node(
            package='nav2_bt_navigator',
            node_executable='bt_navigator',
            node_name='bt_navigator',
            output='screen',
            parameters=[configured_params],
            use_remappings=IfCondition(common.use_remappings),
            remappings=common.remappings),

        Node(
            condition=IfCondition(common.use_lifecycle_mgr),
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            node_name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'use_sim_time': common.use_sim_time},
                        {'autostart': common.autostart},
                        {'node_names': ['controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator']}]),

    ])
