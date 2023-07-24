# Copyright (C) 2023 Open Source Robotics Foundation
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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    use_simulator = LaunchConfiguration('use_simulator')
    use_sim_time = LaunchConfiguration('use_sim_time')
    robot_name = LaunchConfiguration('robot_name')
    robot_sdf = LaunchConfiguration('robot_sdf')
    pose = {'x': LaunchConfiguration('x_pose', default='-2.00'),
            'y': LaunchConfiguration('y_pose', default='-0.50'),
            'z': LaunchConfiguration('z_pose', default='0.01'),
            'R': LaunchConfiguration('roll', default='0.00'),
            'P': LaunchConfiguration('pitch', default='0.00'),
            'Y': LaunchConfiguration('yaw', default='0.00')}

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_robot_name_cmd = DeclareLaunchArgument(
        'robot_name',
        default_value='turtlebot3_waffle',
        description='name of the robot')

    declare_robot_sdf_cmd = DeclareLaunchArgument(
        'robot_sdf',
        default_value=os.path.join(bringup_dir, 'urdf', 'gz_turtlebot3_waffle.urdf'),
        description='Full path to robot sdf file to spawn the robot in gazebo')

    clock_bridge = Node(
        condition=IfCondition(use_simulator),
        package='ros_gz_bridge', executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=['/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock']
    )

    lidar_bridge = Node(
        condition=IfCondition(use_simulator),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='lidar_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[['/scan' + '@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan']],
    )

    imu_bridge = Node(
        condition=IfCondition(use_simulator),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='imu_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[['/imu' + '@sensor_msgs/msg/Imu[ignition.msgs.IMU']],
    )

    odom_bridge = Node(
        condition=IfCondition(use_simulator),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[['/odom' + '@nav_msgs/msg/Odometry[ignition.msgs.Odometry']],
    )

    odom_tf_bridge = Node(
        condition=IfCondition(use_simulator),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='odom_tf_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[['/tf' + '@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V']],
    )

    cm_vel_bridge = Node(
        condition=IfCondition(use_simulator),
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cm_vel_bridge',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        arguments=[['/cmd_vel' + '@geometry_msgs/msg/Twist]ignition.msgs.Twist']],
    )

    # load_joint_state_broadcaster = ExecuteProcess(
    #     condition=IfCondition(use_simulator),
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'joint_state_broadcaster'],
    #     output='screen'
    # )
    #
    # load_diffdrive_controller = ExecuteProcess(
    #     condition=IfCondition(use_simulator),
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #          'diffdrive_controller'],
    #     output='screen'
    # )

    spawn_model = Node(
        condition=IfCondition(use_simulator),
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-entity', robot_name,
            '-string', Command(['xacro', ' ', robot_sdf]),
            '-robot_namespace', namespace,
            '-x', pose['x'], '-y', pose['y'], '-z', pose['z'],
            '-R', pose['R'], '-P', pose['P'], '-Y', pose['Y']]
    )

    env_vars = os.getenv('IGN_GAZEBO_RESOURCE_PATH', default="")
    env_vars += ':' + \
        os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'models') \
        + ':' + \
        os.path.join(get_package_share_directory(
            'turtlebot3_gazebo'), '..')

    set_env_vars_resources = SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', env_vars)

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_robot_name_cmd)
    ld.add_action(declare_robot_sdf_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_use_sim_time_cmd)

    ld.add_action(set_env_vars_resources)

    ld.add_action(clock_bridge)
    ld.add_action(lidar_bridge)
    ld.add_action(imu_bridge)
    ld.add_action(spawn_model)
    ld.add_action(odom_bridge)
    ld.add_action(odom_tf_bridge)
    ld.add_action(cm_vel_bridge)
    # ld.add_action(load_joint_state_broadcaster)
    # ld.add_action(load_diffdrive_controller)
    return ld
