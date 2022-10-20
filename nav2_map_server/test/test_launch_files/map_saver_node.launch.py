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

from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions


def generate_launch_description():
    map_publisher = f"{os.path.dirname(os.getenv('TEST_EXECUTABLE'))}/test_map_saver_publisher"

    ld = LaunchDescription()

    map_saver_server_cmd = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_saver_server',
        output='screen',
        parameters=[os.path.join(os.getenv('TEST_DIR'),
                    'map_saver_params.yaml')])

    map_publisher_cmd = ExecuteProcess(
        cmd=[map_publisher])

    ld.add_action(map_saver_server_cmd)
    ld.add_action(map_publisher_cmd)

    return ld
