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

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService


def generate_launch_description():
    params_file = os.path.join(os.getenv('TEST_DIR'), 'error_code_param.yaml')

    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    lifecycle_nodes = ['controller_server', 'planner_server', 'smoother_server']

    load_nodes = GroupAction(
        actions=[
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
                    '--frame-id', 'map',
                    '--child-frame-id', 'odom'
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
                    '--frame-id', 'odom',
                    '--child-frame-id', 'base_link'
                ],
            ),
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[params_file],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
            ),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[params_file],
                remappings=remappings,
            ),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn_delay=2.0,
                parameters=[params_file],
                remappings=remappings,
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                parameters=[{'autostart': True}, {'node_names': lifecycle_nodes}],
            ),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(load_nodes)

    return ld


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test_error_codes_action = ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'test_error_codes.py')],
        name='test_error_codes',
        output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test_error_codes_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
