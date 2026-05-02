# mypy: ignore-errors
# Copyright (c) 2024 Nav2 Contributors
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(
        nav2_bringup_dir, 'params', 'nav2_params.yaml'
    )
    map_file = os.path.join(nav2_bringup_dir, 'maps', 'tb3_sandbox.yaml')
    lifecycle_nodes = ['map_server', 'planner_server']

    # Configure planner to use DStarLite
    config = RewrittenYaml(
        source_file=nav2_params,
        root_key='planner_server',
        param_rewrites={
            'ros__parameters.planner_plugins': ['GridBased'],
            'ros__parameters.GridBased.plugin':
                'nav2_dstar_lite_planner/DStarLitePlanner',
        },
        convert_types=True
    )

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'yaml_filename': map_file},
                {'topic_name': 'map'},
            ],
        ),
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[config],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'map'
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'base_link',
                '--child-frame-id', 'odom'
            ],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'autostart': True},
                {'node_names': lifecycle_nodes},
            ],
        ),
    ])
