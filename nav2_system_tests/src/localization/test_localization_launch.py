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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
import launch.actions
from launch.actions import ExecuteProcess
import launch_ros.actions
from launch_testing.legacy import LaunchTestService


def main(argv=sys.argv[1:]):
    mapFile = os.getenv('TEST_MAP')
    testExecutable = os.getenv('TEST_EXECUTABLE')
    aws_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    world = os.path.join(
        aws_dir,
        'worlds',
        'small_warehouse',
        'small_warehouse.world'
    )

    urdf = os.getenv('TEST_URDF')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()
    launch_gazebo = launch.actions.ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', world],
        cwd=[aws_dir],
        output='screen')
    launch_robot_description = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True, 'robot_description': robot_description}
        ])
    launch_spawn_entity = launch_ros.actions.Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity',
            'turtlebot3_waffle',
            '-file',
            os.getenv('TEST_MODEL'),
            '-robot_namespace',
            '',
            '-x',
            '0.0',
            '-y',
            '0.0',
            '-z',
            '0.01',
            '-R',
            '0.0',
            '-P',
            '0.0',
            '-Y',
            '0.0',
        ])
    run_map_server = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': mapFile}])
    run_amcl = launch_ros.actions.Node(
        package='nav2_amcl',
        executable='amcl',
        output='screen')
    run_lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'node_names': ['map_server', 'amcl']}, {'autostart': True}])
    ld = LaunchDescription([launch_gazebo, launch_robot_description, launch_spawn_entity,
                            run_map_server, run_amcl, run_lifecycle_manager])

    test1_action = ExecuteProcess(
        cmd=[testExecutable],
        name='test_localization_node',
        output='screen'
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
