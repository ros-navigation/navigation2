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
from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import TextSubstitution
from launch_testing.legacy import LaunchTestService


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    urdf = os.getenv('TEST_URDF')

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'nav2_multi_tb3_simulation_launch.py')),
            launch_arguments={
                              'simulator': TextSubstitution(text='gzserver'),
                              'autostart': 'True',
                              'use_rviz': 'False',
                              'use_robot_state_pub': 'False'}.items()),

        #  Launch robot state publishers for `robot1` and `robot2`
        #  defined in `nav2_multi_tb3_simulation_launch.py`
        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_namespace='robot1',
            output='screen',
            parameters=[{'use_sim_time': 'True'}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            arguments=[urdf]),

        Node(
            package='robot_state_publisher',
            node_executable='robot_state_publisher',
            node_namespace='robot2',
            output='screen',
            parameters=[{'use_sim_time': 'True'}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
            arguments=[urdf]),
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test1_action = ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'test_multi_robot_node.py')],
        name='test_multi_robot_node',
        output='screen')

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
