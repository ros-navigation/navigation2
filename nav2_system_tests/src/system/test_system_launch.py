#!/usr/bin/env python3

# Copyright (c) 2018 Intel Corporation
# Copyright (c) 2020 Florian Gramss
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
from launch.actions import (
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
    map_yaml_file = os.getenv('TEST_MAP')
    world = os.getenv('TEST_WORLD')

    bt_navigator_xml = os.path.join(
        get_package_share_directory('nav2_bt_navigator'),
        'behavior_trees',
        os.getenv('BT_NAVIGATOR_XML'),
    )

    bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')

    # Replace the default parameter values for testing special features
    # without having multiple params_files inside the nav2 stack
    context = LaunchContext()
    param_substitutions = {}

    if os.getenv('ASTAR') == 'True':
        param_substitutions.update({'use_astar': 'True'})

    param_substitutions.update(
        {'planner_server.ros__parameters.GridBased.plugin': os.getenv('PLANNER')}
    )
    param_substitutions.update(
        {'controller_server.ros__parameters.FollowPath.plugin': os.getenv('CONTROLLER')}
    )

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True,
    )

    new_yaml = configured_params.perform(context)

    return LaunchDescription(
        [
            SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
            SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
            # Launch gazebo server for simulation
            ExecuteProcess(
                cmd=[
                    'gzserver',
                    '-s',
                    'libgazebo_ros_init.so',
                    '--minimal_comms',
                    world,
                ],
                output='screen',
            ),
            # TODO(orduno) Launch the robot state publisher instead
            #              using a local copy of TB3 urdf file
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_scan'],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'namespace': '',
                    'use_namespace': 'False',
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
            os.path.join(os.getenv('TEST_DIR'), os.getenv('TESTER')),
            '-r',
            '-2.0',
            '-0.5',
            '0.0',
            '2.0',
            '-e',
            'True',
        ],
        name='tester_node',
        output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
