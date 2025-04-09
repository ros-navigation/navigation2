#!/usr/bin/env python3

# Copyright (c) 2024 John Chrosniak
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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import launch_ros.actions


def generate_launch_description():

    # Nodes launching commands
    map_file = LaunchConfiguration("yaml_filename")

    declare_map_file_cmd = DeclareLaunchArgument(
        "yaml_filename",
        default_value="",
        description="Full path to an occupancy grid map yaml file",
    )

    start_route_tool_cmd = launch_ros.actions.Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[
            "-d"
            + os.path.join(
                get_package_share_directory("nav2_rviz_plugins"),
                "rviz",
                "route_tool.rviz",
            )
        ],
    )

    start_map_server = launch_ros.actions.Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_file}],
    )

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[
            {"use_sim_time": False},
            {"autostart": True},
            {"node_names": ["map_server"]},
        ],
    )

    return LaunchDescription(
        [
            declare_map_file_cmd,
            start_route_tool_cmd,
            start_map_server,
            start_lifecycle_manager_cmd,
        ]
    )
