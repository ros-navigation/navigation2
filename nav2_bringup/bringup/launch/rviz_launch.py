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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='navigation',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.'))

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    # Launch rviz
    start_rviz_cmd = Node(
        condition=UnlessCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    namespaced_rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': ('/', namespace)})

    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(use_namespace),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        arguments=['-d', namespaced_rviz_config_file],
        output='screen',
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')])

    exit_event_handler = RegisterEventHandler(
        condition=UnlessCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    # Add any conditioned actions
    ld.add_action(start_rviz_cmd)
    ld.add_action(start_namespaced_rviz_cmd)

    # Add other nodes and processes we need
    ld.add_action(exit_event_handler)
    ld.add_action(exit_event_handler_namespaced)

    return ld
