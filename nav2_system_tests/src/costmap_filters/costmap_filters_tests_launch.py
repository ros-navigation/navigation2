#!/usr/bin/env python3

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

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess
import launch_ros.actions
from launch_testing.legacy import LaunchTestService


def main(argv=sys.argv[1:]):
    lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'node_names': ['map_server', 'map_mask_server',
                                    'costmap_filter_info_server', 'filters_tester']},
                    {'autostart': True}])

    map_file = os.getenv('TEST_MAP')
    map_server_cmd = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}])

    mapMaskFile = os.getenv('TEST_MASK')
    map_mask_server_cmd = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_mask_server',
            output='screen',
            parameters=[{'yaml_filename': mapMaskFile},
                        {'topic_name': 'filter_mask'}])

    filterParamsFile = os.getenv('FILTER_PARAMS')
    costmap_filter_info_server_cmd = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            parameters=[filterParamsFile])

    testExecutable = os.getenv('TEST_EXECUTABLE')
    test_action = ExecuteProcess(
        cmd=[testExecutable],
        name='costmap_tests',
        output='screen'
    )

    ld = LaunchDescription()

    # Add map server running
    ld.add_action(lifecycle_manager_cmd)
    ld.add_action(map_server_cmd)
    ld.add_action(map_mask_server_cmd)
    ld.add_action(costmap_filter_info_server_cmd)

    lts = LaunchTestService()
    lts.add_test_action(ld, test_action)

    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
