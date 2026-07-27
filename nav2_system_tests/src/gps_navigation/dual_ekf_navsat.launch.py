# Copyright 2018 Open Source Robotics Foundation, Inc.
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

from launch import LaunchDescription
import launch.actions
import launch_ros.actions


def generate_launch_description() -> LaunchDescription:
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    params_file = os.path.join(launch_dir, 'dual_ekf_navsat_params.yaml')
    os.environ['FILE_PATH'] = str(launch_dir)

    # robot_localization nodes are lifecycle nodes on Rolling/Lyrical and newer
    ros_distro = os.environ.get('ROS_DISTRO', '')
    use_lifecycle = ros_distro not in ('jazzy', 'kilted')

    lifecycle_nodes = [
        'ekf_filter_node_odom',
        'ekf_filter_node_map',
    ]

    nodes = [
        launch.actions.DeclareLaunchArgument(
            'output_final_position', default_value='false'
        ),
        launch.actions.DeclareLaunchArgument(
            'output_location', default_value='~/dual_ekf_navsat_example_debug.txt'
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_odom',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}],
            remappings=[('odometry/filtered', 'odometry/local')],
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_map',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}],
            remappings=[('odometry/filtered', 'odometry/global')],
        ),
        launch_ros.actions.Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[params_file, {'use_sim_time': True}],
            remappings=[
                ('imu/data', 'imu/data'),
                ('gps/fix', 'gps/fix'),
                ('gps/filtered', 'gps/filtered'),
                ('odometry/gps', 'odometry/gps'),
                ('odometry/filtered', 'odometry/global'),
            ],
        ),
    ]

    if use_lifecycle:
        nodes.append(
            launch_ros.actions.Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'use_sim_time': True},
                    {'autostart': True},
                    {'bond_timeout': 0.0},
                    {'node_names': lifecycle_nodes},
                ],
            ),
        )

    return LaunchDescription(nodes)
