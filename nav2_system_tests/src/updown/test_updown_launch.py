# Copyright (c) 2019 Intel Corporation
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

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory

import launch.actions
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    # Configuration parameters for the launch
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_yaml_file = os.path.join(nav2_bringup_dir, 'maps', 'tb3_sandbox.yaml')

    # Specify the actions
    start_tf_cmd_1 = Node(
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
    )

    start_tf_cmd_2 = Node(
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
            '--child-frame-id', 'base_footprint'
        ],
    )

    start_tf_cmd_3 = Node(
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
    )

    start_tf_cmd_4 = Node(
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
    )

    nav2_bringup = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': 'True',
            'use_composition': 'False',
            'autostart': 'False',
        }.items(),
    )

    start_test = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_system_tests'),
                'lib/nav2_system_tests/test_updown',
            )
        ],
        cwd=[nav2_bringup_dir],
        output='screen',
    )

    test_exit_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=start_test,
            on_exit=launch.actions.EmitEvent(
                event=launch.events.Shutdown(reason='Done!')
            ),
        )
    )

    # Compose the launch description

    ld = launch.LaunchDescription()

    ld.add_action(start_tf_cmd_1)
    ld.add_action(start_tf_cmd_2)
    ld.add_action(start_tf_cmd_3)
    ld.add_action(start_tf_cmd_4)

    ld.add_action(nav2_bringup)
    ld.add_action(start_test)
    ld.add_action(test_exit_event_handler)

    return ld
