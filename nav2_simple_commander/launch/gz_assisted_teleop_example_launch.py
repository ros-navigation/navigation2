# Copyright (c) 2021 Samsung Research America
# Copyright (c) 2022 Joshua Wallace
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro


def generate_launch_description():
    warehouse_dir = get_package_share_directory('aws_robomaker_small_warehouse_world')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    python_commander_dir = get_package_share_directory('nav2_simple_commander')

    map_yaml_file = os.path.join(warehouse_dir, 'maps', '005', 'map.yaml')
    world = os.path.join(python_commander_dir, 'gz_warehouse.world')

    # Launch configuration variables
    use_rviz = LaunchConfiguration('use_rviz')
    headless = LaunchConfiguration('headless')

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    # Declare the launch arguments
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_simulator_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Whether to execute gzclient)')

    # Launch gazebo environment
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-r -s ' + world]}.items(),
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={'gz_args': ['-g ' + world]}.items(),
        condition=IfCondition(PythonExpression(['not ', headless])),
    )

    urdf = os.path.join(python_commander_dir, 'gz_tb3_waffle.urdf')

    doc = xacro.parse(open(urdf))
    xacro.process_doc(doc)

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': doc.toxml()
        }],
        remappings=remappings
    )

    # start the visualization
    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
        launch_arguments={'namespace': '',
                          'use_namespace': 'False',
                          'use_sim_time': 'true'}.items())

    os.environ['IGN_GAZEBO_RESOURCE_PATH'] = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'models')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] += ':' + os.path.join(
        get_package_share_directory('aws_robomaker_small_warehouse_world'), 'models')
    os.environ['IGN_GAZEBO_RESOURCE_PATH'] += ':' + os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), '..')

    # Clock bridge
    clock_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[
            '/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock']
    )

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

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': True
        }],
        arguments=[['/imu' + '@sensor_msgs/msg/Imu[ignition.msgs.IMU']],
    )

    # start navigation
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={'map': map_yaml_file}.items())

    start_gazebo_spawner_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-entity', 'turtlebot3_waffle',
            '-string', doc.toxml(),
            '-x', '3.45', '-y', '2.15', '-z', '0.1',
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
        executable='example_assisted_teleop',
        emulate_tty=True,
        output='screen')

    ld = LaunchDescription()
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(start_gazebo_server_cmd)
    ld.add_action(start_gazebo_client_cmd)
    ld.add_action(start_gazebo_spawner_cmd)
    ld.add_action(clock_bridge)
    ld.add_action(lidar_bridge)
    ld.add_action(imu_bridge)
    ld.add_action(load_joint_state_controller)
    ld.add_action(load_diffdrive_controller)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(rviz_cmd)
    ld.add_action(bringup_cmd)
    ld.add_action(demo_cmd)
    return ld
