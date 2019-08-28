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


from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch import LaunchService
import launch.actions
import launch_ros.actions
from launch_testing.legacy import LaunchTestService


def generate_launch_description():
    use_sim_time = True
    map_yaml_file = os.getenv('TEST_MAP')
    world = os.getenv('TEST_WORLD')
    bringup_package = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(bringup_package, 'launch/nav2_params.yaml')
    astar = (os.getenv('ASTAR').lower() == 'true')
    bt_navigator_install_path = get_package_prefix('nav2_bt_navigator')
    bt_navigator_xml = os.path.join(bt_navigator_install_path,
                                    'behavior_trees',
                                    os.getenv('BT_NAVIGATOR_XML'))

    return LaunchDescription([
        launch.actions.SetEnvironmentVariable('RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1'),

        # Launch gazebo server for simulation
        launch.actions.ExecuteProcess(
            cmd=['gzserver', '-s', 'libgazebo_ros_init.so',
                 '--minimal_comms', world],
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
            parameters=[params_file]),

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
            parameters=[{'use_sim_time': use_sim_time}, {'use_astar': astar}]),

        launch_ros.actions.Node(
            package='nav2_recoveries',
            node_executable='recoveries_node',
            node_name='recoveries',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}]),

        launch_ros.actions.Node(
            package='nav2_bt_navigator',
            node_executable='bt_navigator',
            node_name='bt_navigator',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}, {'bt_xml_filename': bt_navigator_xml}]),

        launch_ros.actions.Node(
            package='nav2_lifecycle_manager',
            node_executable='lifecycle_manager',
            node_name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'node_names': ['map_server', 'amcl', 'world_model',
                         'dwb_controller', 'navfn_planner', 'bt_navigator']},
                        {'autostart': True}]),
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test1_action = launch.actions.ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'test_system_node.py')],
        name='test_system_node',
        output='screen')

    lts = LaunchTestService()
    lts.add_test_action(ld, test1_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
