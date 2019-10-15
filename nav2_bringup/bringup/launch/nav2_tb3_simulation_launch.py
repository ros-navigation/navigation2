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

"""This is all-in-one launch script intended for use by nav2 developers."""

import os, sys
sys.path.append(os.path.dirname(__file__))

from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from nav2_common.launch import Node
import nav2_bringup_common as common


def generate_launch_description():
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(common.bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start the simulator')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        # TODO(orduno) Switch back once ROS argument passing has been fixed upstream
        #              https://github.com/ROBOTIS-GIT/turtlebot3_simulations/issues/91
        # default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'),
        #                            'worlds/turtlebot3_worlds/waffle.model'),
        default_value=os.path.join(common.bringup_dir, 'worlds', 'waffle.model'),
        description='Full path to world model file to load')

    # Specify the actions
    start_gazebo_cmd = ExecuteProcess(
        condition=IfCondition(common.use_simulator),
        cmd=[common.simulator, '-s', 'libgazebo_ros_init.so', common.world],
        cwd=[common.launch_dir], output='screen')

    urdf = os.path.join(
        get_package_share_directory('turtlebot3_description'), 'urdf', 'turtlebot3_waffle.urdf')

    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        node_name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': common.use_sim_time}],
        use_remappings=IfCondition(common.use_remappings),
        remappings=common.remappings,
        arguments=[urdf])

    # TODO(orduno) RVIZ crashing if launched as a node: https://github.com/ros2/rviz/issues/442
    #              Launching as node works after applying the change described on the github issue.
    #              Once fixed, launch by providing the remappings:
    # rviz_remappings = [('/tf', 'tf'),
    #                    ('/tf_static', 'tf_static'),
    #                    ('goal_pose', 'goal_pose'),
    #                    ('/clicked_point', 'clicked_point'),
    #                    ('/initialpose', 'initialpose'),
    #                    ('/parameter_events', 'parameter_events'),
    #                    ('/rosout', 'rosout')]

    # start_rviz_cmd = Node(
    #     package='rviz2',
    #     node_executable='rviz2',
    #     node_name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     output='screen',
    #     use_remappings=IfCondition(use_remappings),
    #     remappings=rviz_remappings)

    start_rviz_cmd = ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('rviz2'), 'lib/rviz2/rviz2'),
            ['-d', common.rviz_config_file],
            ['__ns:=/', common.namespace],
            '/tf:=tf',
            '/tf_static:=tf_static',
            '/goal_pose:=goal_pose',
            '/clicked_point:=clicked_point',
            '/initialpose:=initialpose'],
        cwd=[common.launch_dir], output='screen')

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=start_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(common.launch_dir, 'nav2_bringup_launch.py')),
        launch_arguments={'namespace': common.namespace,
                          'map': common.map_yaml_file,
                          'use_sim_time': common.use_sim_time,
                          'params_file': common.params_file,
                          'bt_xml_file': common.bt_xml_file,
                          'autostart': common.autostart,
                          'use_remappings': common.use_remappings}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(common.declare_namespace_cmd)
    ld.add_action(common.declare_map_yaml_cmd)
    ld.add_action(common.declare_use_sim_time_cmd)
    ld.add_action(common.declare_params_file_cmd)
    ld.add_action(common.declare_bt_xml_cmd)
    ld.add_action(common.declare_autostart_cmd)
    ld.add_action(common.declare_use_remappings_cmd)

    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(common.declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add any actions to launch in simulation (conditioned on 'use_simulator')
    ld.add_action(start_gazebo_cmd)

    # Add other nodes and processes we need
    ld.add_action(start_rviz_cmd)
    ld.add_action(exit_event_handler)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(bringup_cmd)

    return ld
