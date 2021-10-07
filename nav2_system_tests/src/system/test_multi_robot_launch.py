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

from launch import LaunchDescription, LaunchService
from launch.actions import (ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node, PushRosNamespace

from launch_testing.legacy import LaunchTestService


def generate_launch_description():
    map_yaml_file = os.getenv('TEST_MAP')
    world = os.getenv('TEST_WORLD')
    urdf = os.getenv('TEST_URDF')
    sdf = os.getenv('TEST_SDF')

    bt_xml_file = os.path.join(get_package_share_directory('nav2_bt_navigator'),
                               'behavior_trees',
                               os.getenv('BT_NAVIGATOR_XML'))

    bringup_dir = get_package_share_directory('nav2_bringup')
    robot1_params_file = os.path.join(bringup_dir,  # noqa: F841
                                      'params/nav2_multirobot_params_1.yaml')
    robot2_params_file = os.path.join(bringup_dir,  # noqa: F841
                                      'params/nav2_multirobot_params_2.yaml')

    # Names and poses of the robots
    robots = [
        {'name': 'robot1', 'x_pose': 0.0, 'y_pose': 0.5, 'z_pose': 0.01},
        {'name': 'robot2', 'x_pose': 0.0, 'y_pose': -0.5, 'z_pose': 0.01}]

    # Launch Gazebo server for simulation
    start_gazebo_cmd = ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
                 '-s', 'libgazebo_ros_factory.so', '--minimal_comms', world],
            output='screen')

    # Define commands for spawing the robots into Gazebo
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                output='screen',
                arguments=[
                    '-entity', TextSubstitution(text=robot['name']),
                    '-robot_namespace', TextSubstitution(text=robot['name']),
                    '-file', TextSubstitution(text=sdf),
                    '-x', TextSubstitution(text=str(robot['x_pose'])),
                    '-y', TextSubstitution(text=str(robot['y_pose'])),
                    '-z', TextSubstitution(text=str(robot['z_pose']))]
            ))

    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # Define commands for launching the robot state publishers
    robot_state_pubs_cmds = []
    for robot in robots:
        robot_state_pubs_cmds.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=TextSubstitution(text=robot['name']),
                output='screen',
                parameters=[{'use_sim_time': True, 'robot_description': robot_description}],
                remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))

    # Define commands for launching the navigation instances
    nav_instances_cmds = []
    for robot in robots:
        params_file = eval(f"{robot['name']}_params_file")

        group = GroupAction([
            # Instances use the robot's name for namespace
            PushRosNamespace(robot['name']),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(bringup_dir, 'launch', 'bringup_launch.py')),
                launch_arguments={
                                  'namespace': robot['name'],
                                  'map': map_yaml_file,
                                  'use_sim_time': 'True',
                                  'params_file': params_file,
                                  'bt_xml_file': bt_xml_file,
                                  'autostart': 'True',
                                  'use_remappings': 'True'}.items())
        ])
        nav_instances_cmds.append(group)

    ld = LaunchDescription()
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),)
    ld.add_action(SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),)
    ld.add_action(start_gazebo_cmd)
    for spawn_robot in spawn_robots_cmds:
        ld.add_action(spawn_robot)
    for state_pub in robot_state_pubs_cmds:
        ld.add_action(state_pub)
    for nav_instance in nav_instances_cmds:
        ld.add_action(nav_instance)

    return ld


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    # TODO(orduno) remove duplicated definition of robots on `generate_launch_description`
    test1_action = ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), os.getenv('TESTER')),
             '-rs', 'robot1', '0.0', '0.5', '1.0', '0.5',
             '-rs', 'robot2', '0.0', '-0.5', '1.0', '-0.5',
             '-e', 'True'],
        name='tester_node',
        output='screen')

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
