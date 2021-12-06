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
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    aws_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    map_yaml_file = os.path.join(aws_dir, 'maps', '005', 'map.yaml')
    filter_mask_file = os.getenv('TEST_MASK')
    world = os.path.join(aws_dir, 'worlds', 'no_roof_small_warehouse',
                         'no_roof_small_warehouse.world')

    bt_navigator_xml = os.path.join(get_package_share_directory('nav2_bt_navigator'),
                                    'behavior_trees',
                                    os.getenv('BT_NAVIGATOR_XML'))

    bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = os.getenv('PARAMS_FILE')

    # Replace the `use_astar` setting on the params file
    param_substitutions = {
        'planner_server.ros__parameters.GridBased.use_astar': os.getenv('ASTAR'),
        'filter_mask_server.ros__parameters.yaml_filename': filter_mask_file,
        'yaml_filename': filter_mask_file}
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)
    context = LaunchContext()
    new_yaml = configured_params.perform(context)
    urdf = os.getenv('TEST_URDF')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),

        # Launch gazebo server for simulation
        ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                 '--minimal_comms', world],
            cwd=[aws_dir],
            output='screen'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'use_sim_time': True, 'robot_description': robot_description}
            ]),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=['-entity', 'turtlebot3_waffle', '-file', os.getenv('TEST_MODEL'),
                       '-robot_namespace', '', '-x', '0.0', '-y', '0.0', '-z', '0.01',
                       '-R', '0.0', '-P', '0.0', '-Y', '0.0']),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_filters',
            output='screen',
            parameters=[{'node_names': ['filter_mask_server', 'costmap_filter_info_server']},
                        {'autostart': True}]),

        # Nodes required for Costmap Filters configuration
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            parameters=[new_yaml]),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[new_yaml]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={'namespace': '',
                              'use_namespace': 'False',
                              'map': map_yaml_file,
                              'use_sim_time': 'True',
                              'params_file': new_yaml,
                              'bt_xml_file': bt_navigator_xml,
                              'autostart': 'True'}.items()),
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test1_action = ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'tester_node.py'),
             '-t', 'speed', '-r', '0.0', '0.0', '1.0', '5.0'],
        name='tester_node',
        output='screen')

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
