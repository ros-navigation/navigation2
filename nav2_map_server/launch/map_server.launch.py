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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description() -> LaunchDescription:
    # Parameters
    bringup_dir = get_package_share_directory("nav2_bringup")
    map_yaml_file = os.path.join(bringup_dir, "maps", "depot.yaml")

    lifecycle_nodes = ["map_server"]
    use_sim_time = True
    autostart = True
    topic_name = "map"
    frame_id = "map"
    service_introspection_mode = "disabled"

    start_map_server_cmd = launch_ros.actions.Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_yaml_file},
            {"topic_name": topic_name},
            {"frame_id": frame_id},
            {"service_introspection_mode": service_introspection_mode},
        ],
    )

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"autostart": autostart},
            {"node_names": lifecycle_nodes},
        ],
    )

    ld = LaunchDescription()

    ld.add_action(start_map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld
