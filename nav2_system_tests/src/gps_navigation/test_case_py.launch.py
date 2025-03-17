#! /usr/bin/env python3
# Copyright (c) 2019 Samsung Research America
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
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import (
    AppendEnvironmentVariable,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_testing.legacy import LaunchTestService

from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    world_sdf_xacro = os.path.join(sim_dir, 'worlds', 'tb3_empty_world.sdf.xacro')
    robot_sdf = os.path.join(sim_dir, 'urdf', 'gz_waffle_gps.sdf.xacro')

    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle_gps.urdf')

    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    # use local param file
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    params_file = os.path.join(launch_dir, 'nav2_no_map_params.yaml')

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites='',
        convert_types=True,
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
            SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models')
            ),
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                str(Path(os.path.join(sim_dir)).parent.resolve()),
            ),
            ExecuteProcess(
                cmd=['gz', 'sim', '-r', '-s', world_sdf_xacro],
                output='screen',
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(sim_dir, 'launch', 'spawn_tb3_gps.launch.py')
                ),
                launch_arguments={
                    'use_sim_time': 'True',
                    'robot_sdf': robot_sdf,
                    'x_pose': '0.0',
                    'y_pose': '0.0',
                    'z_pose': '0.0',
                    'roll': '0.0',
                    'pitch': '0.0',
                    'yaw': '0.0',
                }.items(),
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {'use_sim_time': True, 'robot_description': robot_description}
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': '',
                    'use_sim_time': 'True',
                    'params_file': configured_params,
                    'use_composition': 'False',
                    'autostart': 'True',
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'dual_ekf_navsat.launch.py')
                ),
            ),
        ]
    )


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test1_action = ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'tester.py')],
        name='tester_node',
        output='screen',
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return_code = lts.run(ls)
    return return_code


if __name__ == '__main__':
    sys.exit(main())
