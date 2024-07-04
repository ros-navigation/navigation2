#! /usr/bin/env python3
# Copyright (c) 2012 Samsung Research America
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
    params_file = os.path.join(bringup_dir, 'params/nav2_params.yaml')

    # Replace the `use_astar` setting on the params file
    configured_params = RewrittenYaml(
        source_file=params_file, root_key='', param_rewrites='', convert_types=True
    )

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
                    'map': map_yaml_file,
                    'use_sim_time': 'True',
                    'params_file': configured_params,
                    'bt_xml_file': bt_navigator_xml,
                    'use_composition': 'False',
                    'autostart': 'True',
                }.items(),
            ),
        ]
    )


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    testExecutable = os.getenv('TEST_EXECUTABLE')

    test1_action = ExecuteProcess(
        cmd=[testExecutable], name='test_assisted_teleop_behavior_node', output='screen'
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
