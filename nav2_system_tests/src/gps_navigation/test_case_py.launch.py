#! /usr/bin/env python3
# Copyright (c) 2019 Samsung Research America
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
    world = os.getenv('TEST_WORLD')

    launch_dir = os.path.dirname(os.path.realpath(__file__))
    params_file = os.path.join(launch_dir, 'nav2_no_map_params.yaml')
    bringup_dir = get_package_share_directory('nav2_bringup')

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
                arguments=[
                    '--x', '0',
                    '--y', '0',
                    '--z', '0',
                    '--roll', '0',
                    '--pitch', '0',
                    '--yaw', '0',
                    '--frame-id', 'base_footprint',
                    '--child-frame-id', 'base_link'
                ],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=[
                    '--x', '0',
                    '--y', '0',
                    '--z', '0',
                    '--roll', '0',
                    '--pitch', '0',
                    '--yaw', '0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'base_scan'
                ],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=[
                    '--x', '-0.32',
                    '--y', '0',
                    '--z', '0.068',
                    '--roll', '0',
                    '--pitch', '0',
                    '--yaw', '0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'imu_link'
                ],
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=[
                    '--x', '0',
                    '--y', '0',
                    '--z', '0',
                    '--roll', '0',
                    '--pitch', '0',
                    '--yaw', '0',
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'gps_link'
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'True',
                    'params_file': configured_params,
                    'autostart': 'True',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'dual_ekf_navsat.launch.py')
                ),
            ),
        ]
    )


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test1_action = ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'tester.py')],
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
