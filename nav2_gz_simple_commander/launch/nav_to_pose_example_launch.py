# Copyright (C) 2023 Open Source Robotics Foundation

# Licensed under the Apache License, Version 2.0 (the "License")
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

#       http://www.apache.org/licenses/LICENSE-2.0

# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import xacro

def generate_launch_description():
    warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    python_commander_dir = get_package_share_directory('nav2_gz_simple_commander')

    map_yaml_file = os.path.join(warehouse_dir, 'maps', '005', 'map.yaml')
    world = os.path.join(python_commander_dir, 'gz_warehouse.world')

    # Launch gazebo environment
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r -v 4 ' + world]}.items(),
    )

    urdf = os.path.join(nav2_bringup_dir, 'urdf', 'gz_turtlebot3_waffle.urdf')

    doc = xacro.parse(open(urdf))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': doc.toxml()
        }])

    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False'}.items())

    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] += ':' + os.path.join(get_package_share_directory('aws_robomaker_small_warehouse_world'), 'models')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] += ':' + os.path.join(get_package_share_directory('turtlebot3_gazebo'), '..')

    # Clock bridge
    clock_bridge = Node(package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'
    ])

    # lidar bridge
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            ['/scan' + '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']
        ]
    )

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_yaml_file, 'use_sim_time': 'true'}.items())

    start_gazebo_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-entity', 'tb3',
            '-string', doc.toxml(),
            '-x', '3.45', '-y', '2.15', '-z', '0.01',
            '-R', '0', '-P', '0', '-Y', '3.14'
        ]
    )

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )
    load_diffdrive_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'diffdrive_controller'],
        output='screen'
    )

    # start the demo autonomy task
    demo_cmd = Node(
        package='nav2_simple_commander',
        executable='example_nav_to_pose',
        emulate_tty=True,
        output='screen')

    ld = LaunchDescription()
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(clock_bridge)
    ld.add_action(lidar_bridge)
    ld.add_action(load_joint_state_controller)
    ld.add_action(load_diffdrive_controller)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(demo_cmd)

    return ld
