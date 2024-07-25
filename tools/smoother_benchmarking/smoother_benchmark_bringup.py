# Copyright (c) 2022 Samsung Research America
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
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    benchmark_dir = os.getcwd()
    metrics_py = os.path.join(benchmark_dir, 'metrics.py')
    config = os.path.join(
        get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml'
    )
    map_file = os.path.join(benchmark_dir, 'maps', 'smoothers_world.yaml')
    lifecycle_nodes = ['map_server', 'planner_server', 'smoother_server']

    static_transform_one = Node(
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
            '--child-frame-id', 'map'
        ],
    )

    static_transform_two = Node(
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
            '--child-frame-id', 'odom'
        ],
    )

    start_map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'yaml_filename': map_file},
            {'topic_name': 'map'},
        ],
    )

    start_planner_server_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[config],
    )

    start_smoother_server_cmd = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[config],
    )

    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[
            {'use_sim_time': True},
            {'autostart': True},
            {'node_names': lifecycle_nodes},
        ],
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={'namespace': '', 'use_namespace': 'False'}.items(),
    )

    metrics_cmd = ExecuteProcess(
        cmd=['python3', '-u', metrics_py], cwd=[benchmark_dir], output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(static_transform_one)
    ld.add_action(static_transform_two)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_planner_server_cmd)
    ld.add_action(start_smoother_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(metrics_cmd)
    return ld
