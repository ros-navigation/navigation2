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
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchService
from launch.actions import AppendEnvironmentVariable, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_ros.actions
from launch_testing.legacy import LaunchTestService


def main(argv=sys.argv[1:]):
    testExecutable = os.getenv('TEST_EXECUTABLE')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    world_sdf_xacro = os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro')
    robot_sdf = os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro')

    urdf = os.path.join(sim_dir, 'urdf', 'turtlebot3_waffle.urdf')
    with open(urdf, 'r') as infp:
        robot_description = infp.read()

    map_yaml_file = os.path.join(nav2_bringup_dir, 'maps', 'tb3_sandbox.yaml')

    set_env_vars_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models')
    )
    set_env_vars_resources2 = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        str(Path(os.path.join(sim_dir)).parent.resolve())
    )

    start_gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf_xacro],
        output='screen',
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'True',
            'robot_sdf': robot_sdf,
            'x_pose': '-2.0',
            'y_pose': '-0.5',
            'z_pose': '0.01',
            'roll': '0.0',
            'pitch': '0.0',
            'yaw': '0.0',
        }.items(),
    )
    run_robot_state_publisher = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': True, 'robot_description': robot_description}
        ],
    )
    run_map_server = launch_ros.actions.Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}],
    )
    run_amcl = launch_ros.actions.Node(
        package='nav2_amcl', executable='amcl', output='screen'
    )
    run_lifecycle_manager = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        parameters=[{'node_names': ['map_server', 'amcl']}, {'autostart': True}],
    )
    ld = LaunchDescription(
        [
            set_env_vars_resources,
            set_env_vars_resources2,
            start_gazebo_server,
            spawn_robot,
            run_robot_state_publisher,
            run_map_server,
            run_amcl,
            run_lifecycle_manager,
        ]
    )

    test1_action = ExecuteProcess(
        cmd=[testExecutable], name='test_localization_node', output='screen'
    )

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return_code = lts.run(ls)
    return return_code


if __name__ == '__main__':
    sys.exit(main())
