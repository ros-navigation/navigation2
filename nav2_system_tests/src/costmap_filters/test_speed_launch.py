#!/usr/bin/env python3

# Copyright (c) 2018 Intel Corporation
# Copyright (c) 2020 Samsung Research Russia
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
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import (
    AppendEnvironmentVariable,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_sys_test_dir = get_package_share_directory('nav2_system_tests')

    world_sdf_xacro = os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro')
    robot_sdf = os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')

    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    map_yaml_file = os.path.join(nav2_sys_test_dir, 'maps', 'map_circular.yaml')
    filter_mask_file = os.getenv('TEST_MASK')

    bt_navigator_xml = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        os.getenv('BT_NAVIGATOR_XML'),
    )

    params_file = os.getenv('PARAMS_FILE')

    # Replace the `use_astar` setting on the params file
    param_substitutions = {
        'planner_server.ros__parameters.GridBased.use_astar': os.getenv('ASTAR'),
        'filter_mask_server.ros__parameters.yaml_filename': filter_mask_file,
        'map_server.ros__parameters.yaml_filename': map_yaml_file,
    }
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True,
    )
    context = LaunchContext()
    new_yaml = configured_params.perform(context)
    return LaunchDescription(
        [
            SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
            SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
            # Launch gazebo server for simulation
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models')
            ),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                str(Path(os.path.join(sim_dir)).parent.resolve())
            ),
            ExecuteProcess(
                cmd=['gz', 'sim', '-r', '-s', world_sdf_xacro],
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'True',
                    'robot_sdf': robot_sdf,
                    'x_pose': '-2.0',
                    'y_pose': '-0.5',
                    'z_pose': '0.01',
                    'roll': '0.0',
                    'pitch': '0.0',
                    'yaw': '0.0',
                }.items(),
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {'use_sim_time': True, 'robot_description': robot_description}
                ],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_filters',
                output='screen',
                parameters=[
                    {
                        'node_names': [
                            'filter_mask_server',
                            'costmap_filter_info_server',
                        ]
                    },
                    {'autostart': True},
                ],
            ),
            # Nodes required for Costmap Filters configuration
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                output='screen',
                parameters=[new_yaml],
            ),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                output='screen',
                parameters=[new_yaml],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'namespace': '',
                    'map': map_yaml_file,
                    'use_sim_time': 'True',
                    'params_file': new_yaml,
                    'bt_xml_file': bt_navigator_xml,
                    'use_composition': 'False',
                    'autostart': 'True',
                }.items(),
            ),
        ]
    )


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test1_action = ExecuteProcess(
        cmd=[
            os.path.join(os.getenv('TEST_DIR'), 'tester_node.py'),
            '-t',
            'speed',
            '-r',
            '-2.0',
            '-0.5',
            '0.0',
            '-0.5',
        ],
        name='tester_node',
        output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return_code = lts.run(ls)
    return return_code


if __name__ == '__main__':
    sys.exit(main())
