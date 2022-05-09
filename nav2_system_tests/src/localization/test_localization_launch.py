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

from os import environ
from os import pathsep

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch_testing.legacy import LaunchTestService

from scripts import GazeboRosPaths


def main(argv=sys.argv[1:]):
    model, plugin, media = GazeboRosPaths.get_paths()

    if 'GAZEBO_MODEL_PATH' in environ:
        model += pathsep+environ['GAZEBO_MODEL_PATH']
    if 'GAZEBO_PLUGIN_PATH' in environ:
        plugin += pathsep+environ['GAZEBO_PLUGIN_PATH']
    if 'GAZEBO_RESOURCE_PATH' in environ:
        media += pathsep+environ['GAZEBO_RESOURCE_PATH']

    aws_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')

    env = {'GAZEBO_MODEL_PATH': model,
           'GAZEBO_PLUGIN_PATH': plugin,
           'GAZEBO_RESOURCE_PATH': media}

    bringup_dir = get_package_share_directory('nav2_bringup')
    mapFile = os.path.join(aws_dir, 'maps', '005', 'map.yaml')
    testExecutable = os.getenv('TEST_EXECUTABLE')

    world = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {'x': LaunchConfiguration('x_pose', default='1.80'),
            'y': LaunchConfiguration('y_pose', default='2.20'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(bringup_dir, 'worlds', 'waffle.model'),
        description='Full path to robot sdf file to spawn the robot in gazebo')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(aws_dir, 'worlds', 'no_roof_small_warehouse',
                                   'no_roof_small_warehouse.world'),
        description='Full path to world model file to load')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle',
        description='name of the robot')

    launch_gazebo = ExecuteProcess(
        cmd=['gzserver', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so',
             '--minimal_comms', world],
        additional_env=env,
        cwd=[aws_dir], output='screen')

    run_gazebo_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-file', robot_sdf,
            '-robot_namespace', '',
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']])
    link_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'])
    footprint_scan = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_scan'])
    run_map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': mapFile}])
    run_amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        output='screen')
    run_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'node_names': ['map_server', 'amcl']}, {'autostart': True}])
    ld = LaunchDescription([declare_world_cmd, declare_robot_name_cmd, declare_robot_sdf_cmd,
                            run_gazebo_spawner, launch_gazebo, link_footprint,
                            footprint_scan, run_map_server, run_amcl, run_lifecycle_manager])

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
