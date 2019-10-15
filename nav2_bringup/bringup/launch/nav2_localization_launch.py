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

from nav2_common.launch import RewrittenYaml
from nav2_common.launch import Node
import nav2_bringup_common as common

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'use_sim_time': common.use_sim_time,
        'yaml_filename': common.map_yaml_file}

    configured_params = RewrittenYaml(
        source_file=common.params_file,
        root_key=common.namespace,
        param_rewrites=param_substitutions,
        convert_types=True)

    return LaunchDescription([
        # Set env var to print messages to stdout immediately
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        common.declare_namespace_cmd,

        common.declare_tb3_map_yaml_cmd,

        common.declare_use_sim_time_cmd,

        common.declare_autostart_cmd,

        common.declare_params_file_cmd,

        common.declare_use_lifecycle_mgr_cmd,

        common.declare_use_remappings_cmd,

        Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server',
            output='screen',
            parameters=[configured_params],
            use_remappings=IfCondition(common.use_remappings),
            remappings=common.remappings),

        Node(
            package='nav2_amcl',
            node_executable='amcl',
            node_name='amcl',
            output='screen',
            parameters=[configured_params],
            use_remappings=IfCondition(common.use_remappings),
            remappings=common.remappings),

        Node(
            condition=IfCondition(common.use_lifecycle_mgr),
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            node_name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': common.use_sim_time},
                        {'autostart': common.autostart},
                        {'node_names': ['map_server', 'amcl']}])
    ])
