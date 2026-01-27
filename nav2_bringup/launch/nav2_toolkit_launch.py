# Copyright (C) 2025 Pranav Kolekar
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
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description() -> LaunchDescription:
    bringup_dir = get_package_share_directory('nav2_bringup')
    default_params_file = os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    default_pose_file = os.path.join(os.environ['HOME'], 'last_known_pose.yaml')

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')
    pose_file_path = LaunchConfiguration('pose_file_path')

    declare_namespace = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation time')

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to the parameters file')

    declare_pose_file_path = DeclareLaunchArgument(
        'pose_file_path',
        default_value=default_pose_file,
        description='Full path to store the pose file')

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    pose_saver_node = Node(
        package='nav2_toolkit',
        executable='pose_saver_node',
        name='pose_saver',
        output='screen',
        parameters=[configured_params, {'pose_file_path': pose_file_path}],
    )

    return LaunchDescription([
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        declare_namespace,
        declare_use_sim_time,
        declare_params_file,
        declare_pose_file_path,
        pose_saver_node,
    ])
