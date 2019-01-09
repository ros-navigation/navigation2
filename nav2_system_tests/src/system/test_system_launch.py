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
import sys


from launch import LaunchDescription
from launch import LaunchService
import launch.actions
import launch_ros.actions
from launch_testing import LaunchTestService


def generate_launch_description():
    map_yaml_file = os.getenv('TEST_MAP')
    world = os.getenv('TEST_WORLD')
    params_file = os.getenv('TEST_PARAMS')
    use_sim_time = True

    return LaunchDescription([
        # Launch gazebo server for simulation
        launch.actions.ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '--minimal_comms', world],
            output='screen'),

        # Launch navigation2 nodes
        launch_ros.actions.Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']),

        launch_ros.actions.Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            output='screen',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_scan']),

        launch_ros.actions.Node(
            package='nav2_map_server',
            node_executable='map_server',
            node_name='map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, {'yaml_filename': map_yaml_file}]),

        launch_ros.actions.Node(
            package='nav2_world_model',
            node_executable='world_model',
            output='screen',
            parameters=[params_file]),

        launch_ros.actions.Node(
            package='nav2_amcl',
            node_executable='amcl',
            node_name='amcl',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='dwb_controller',
            node_executable='dwb_controller',
            output='screen',
            parameters=[params_file]),

        launch_ros.actions.Node(
            package='nav2_navfn_planner',
            node_executable='navfn_planner',
            node_name='navfn_planner',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='nav2_simple_navigator',
            node_executable='simple_navigator',
            node_name='simple_navigator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test1_action = launch.actions.ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'test_system_node.py')],
        name='test_system_node',
        output='screen')

    ld.add_action(test1_action)
    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
