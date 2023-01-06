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

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    config = os.path.join(get_package_share_directory('nav2_bringup'), 'params', 'nav2_params.yaml')
    map_file = os.path.join(nav2_bringup_dir, 'maps', 'turtlebot3_world.yaml')
    lifecycle_nodes = ['map_server', 'planner_server']

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': map_file},
                        {'topic_name': "map"}]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[config]),

        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            output = 'screen',
            arguments = ["0", "0", "0", "0", "0", "0", "base_link", "map"]),

        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            output = 'screen',
            arguments = ["0", "0", "0", "0", "0", "0", "base_link", "odom"]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
            launch_arguments={'namespace': '',
                              'use_namespace': 'False'}.items())

    ])
