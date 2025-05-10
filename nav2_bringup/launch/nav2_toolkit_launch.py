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

"""Launch file for persistent pose saver/restorer node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    # Declare Launch Arguments
    declare_auto_start_saving_cmd = DeclareLaunchArgument(
        'auto_start_saving', default_value='true',
        description='Whether to automatically start saving poses')

    declare_auto_restore_pose_cmd = DeclareLaunchArgument(
        'auto_restore_pose', default_value='true',
        description='Whether to automatically restore pose on startup')

    declare_pose_file_path_cmd = DeclareLaunchArgument(
    'pose_file_path', 
    default_value=os.path.join(
        get_package_share_directory('nav2_toolkit'), 
        'config', 
        'last_known_pose.yaml'),
    description='File path to store and restore pose from'
    )



    declare_save_interval_sec_cmd = DeclareLaunchArgument(
        'save_interval_sec', default_value='5.0',
        description='Interval in seconds to save pose')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    # Node
    pose_saver_node = Node(
        package='nav2_toolkit',
        executable='pose_saver_node',
        name='pose_saver',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
    )

    return LaunchDescription([
        declare_auto_start_saving_cmd,
        declare_auto_restore_pose_cmd,
        declare_pose_file_path_cmd,
        declare_save_interval_sec_cmd,
        declare_params_file_cmd,
        pose_saver_node,
    ])
