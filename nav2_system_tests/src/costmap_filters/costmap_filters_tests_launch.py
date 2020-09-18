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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch_testing.legacy import LaunchTestService


def main(argv=sys.argv[1:]):
    lifecycle_manager_cmd = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'node_names': ['map_server', 'filters_tester']},
                    {'autostart': True}])

    map_file = os.getenv('TEST_MAP')
    map_server_cmd = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'yaml_filename': map_file}])

    bringup_dir = get_package_share_directory('nav2_bringup')
    costmap_filter_info_launch_file = os.path.join(
        bringup_dir, 'launch', 'costmap_filter_info.launch.py')
    params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    map_mask_file = os.getenv('TEST_MASK')
    costmap_filter_info_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(costmap_filter_info_launch_file),
        launch_arguments={'filter_namespace': 'global_costmap',
                          'autostart': 'True',
                          'params_file': params_file,
                          'mask': map_mask_file}.items())

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
    ld.add_action(costmap_filter_info_server_cmd)

    lts = LaunchTestService()
    lts.add_test_action(ld, test_action)

    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
