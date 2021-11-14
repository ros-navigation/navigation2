#!/usr/bin/env python3

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
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService


def generate_launch_description():
    aws_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    map_yaml_file = os.path.join(aws_dir, 'maps', '005', 'map.yaml')
    world = os.path.join(aws_dir, 'worlds', 'small_warehouse', 'small_warehouse.world')

    urdf = os.getenv('TEST_URDF')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),

        # Launch gazebo server for simulation
        ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
                 world],
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
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_yaml_file}]),

        Node(
            package='nav2_amcl',
            executable='amcl',
            output='screen'),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'node_names': ['map_server', 'amcl']}, {'autostart': True}])
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    testExecutable = os.getenv('TEST_EXECUTABLE')

    test1_action = ExecuteProcess(
        cmd=[testExecutable],
        name='test_localization_node',
        output='screen')

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
