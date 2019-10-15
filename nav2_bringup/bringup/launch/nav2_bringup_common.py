# Copyright (c) 2019 Intel Corporation
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

import os, sys
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


# Get the launch directory
bringup_dir = get_package_share_directory('nav2_bringup')
launch_dir = os.path.join(bringup_dir, 'launch')

# Create the launch configuration variables
namespace = LaunchConfiguration('namespace')
map_yaml_file = LaunchConfiguration('map')
use_sim_time = LaunchConfiguration('use_sim_time')
autostart = LaunchConfiguration('autostart')
params_file = LaunchConfiguration('params_file')
use_lifecycle_mgr = LaunchConfiguration('use_lifecycle_mgr')
use_remappings = LaunchConfiguration('use_remappings')
bt_xml_file = LaunchConfiguration('bt_xml_file')

# Launch configuration variables specific to simulation
rviz_config_file = LaunchConfiguration('rviz_config_file')
use_simulator = LaunchConfiguration('use_simulator')
simulator = LaunchConfiguration('simulator')
world = LaunchConfiguration('world')

# TODO(orduno) Remove once `PushNodeRemapping` is resolved
#              https://github.com/ros2/launch_ros/issues/56
remappings = [((namespace, '/tf'), '/tf'),
                ((namespace, '/tf_static'), '/tf_static'),
                ('/scan', 'scan'),
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
                ('/cmd_vel', 'cmd_vel'),
                ('/map', 'map'),
                ('/goal_pose', 'goal_pose')]

# Declare the launch arguments
declare_autostart_cmd = DeclareLaunchArgument(
    'autostart', default_value='true',
    description='Automatically startup the nav2 stack')

declare_namespace_cmd = DeclareLaunchArgument(
    'namespace',
    default_value='',
    description='Top-level namespace')

declare_map_yaml_cmd = DeclareLaunchArgument(
    'map',
    description='Full path to map yaml file to load')

declare_use_sim_time_cmd = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Use simulation (Gazebo) clock if true')

declare_bt_xml_cmd = DeclareLaunchArgument(
    'bt_xml_file',
    default_value=os.path.join(
        get_package_prefix('nav2_bt_navigator'),
        'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
    description='Full path to the behavior tree xml file to use')

declare_params_file_cmd = DeclareLaunchArgument(
    'params_file',
    default_value=os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
    description='Full path to the ROS2 parameters file to use for all launched nodes')

declare_use_remappings_cmd = DeclareLaunchArgument(
    'use_remappings', default_value='false',
    description='Arguments to pass to all nodes launched by the file')

declare_simulator_cmd = DeclareLaunchArgument(
    'simulator',
    default_value='gazebo',
    description='The simulator to use (gazebo or gzserver)')

declare_use_lifecycle_mgr_cmd = DeclareLaunchArgument(
    'use_lifecycle_mgr', default_value='true',
    description='Whether to launch the lifecycle manager')

declare_tb3_map_yaml_cmd = DeclareLaunchArgument(
    'map',
    default_value=os.path.join(bringup_dir, 'maps', 'turtlebot3_world.yaml'),
    description='Full path to map yaml file to load')
