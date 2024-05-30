#! /usr/bin/env python3
# Copyright (c) 2020 Shivang Patel
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
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_test',
                output='screen',
                parameters=[
                    {'use_sim_time': False},
                    {'autostart': False},
                    {'bond_timeout': 0.0},
                    {'node_names': ['lifecycle_node_test']},
                ],
            ),
        ]
    )


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    testExecutable = os.getenv('TEST_EXECUTABLE')

    test1_action = ExecuteProcess(
        cmd=[testExecutable], name='test_lifecycle_node_gtest', output='screen'
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
