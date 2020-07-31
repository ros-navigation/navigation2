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

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess
import launch_ros.actions


def generate_launch_description():
    # Parameters
    namespace = '/local_costmap'
    lifecycle_nodes = ['map_mask_server']
    mask_topic = 'map_mask'
    use_sim_time = True
    autostart = True

    # Paths
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    yaml_dir = os.path.join(os.path.split(launch_dir)[0], 'map')
    yaml_filename = os.path.join(yaml_dir, 'keepout_mask.yaml')
    info_pub_dir = os.path.join(get_package_prefix('nav2_costmap_2d'), 'lib', 'test')
    info_pub_bin = os.path.join(info_pub_dir, 'costmap_filter_info_pub')

    # Nodes launching commands
    start_map_server_cmd = launch_ros.actions.Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_mask_server',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'yaml_filename': yaml_filename},
                        {'topic_name': mask_topic}])

    start_lifecycle_manager_cmd = launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            namespace=namespace,
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])

    start_costmap_filter_info_pub_cmd = ExecuteProcess(
           cmd=[info_pub_bin, '--ros-args', '--remap', '__ns:=' + namespace],
           cwd=[info_pub_dir],
           output='screen',
           emulate_tty=True  # https://github.com/ros2/launch/issues/188
    )

    ld = LaunchDescription()

    ld.add_action(start_map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_costmap_filter_info_pub_cmd)

    return ld
