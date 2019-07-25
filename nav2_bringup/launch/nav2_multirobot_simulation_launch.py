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

''' This is an example on how to create a launch file for spawning multiple robots into Gazebo
    and launch multiple instances of the navigation stack, each controlling one robot.
    The robots co-exist on a shared environment and are controlled by independent nav stacks '''

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import LogInfo
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import PushRosNamespace

import launch.actions
import launch_ros.actions


def generate_launch_description():
    # Get the launch directory
    # TODO(orduno) Switch to ThisLaunchFileDir
    launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    # Names and poses of the robots
    # TODO(orduno) provide pose as a number not text and use text substitution instead
    robots = [
        {'name': 'Robot1', 'x_pose': '0.0', 'y_pose': '0.5', 'z_pose': '0.01'},
        {'name': 'Robot2', 'x_pose': '0.0', 'y_pose': '-0.5', 'z_pose': '0.01'}]

    # Create the launch configuration variables
    world = launch.substitutions.LaunchConfiguration('world')
    simulator = launch.substitutions.LaunchConfiguration('simulator')

    map_yaml_file = launch.substitutions.LaunchConfiguration('map_yaml_file')
    params_file = launch.substitutions.LaunchConfiguration('params_file')

    # Declare the launch arguments
    declare_world_cmd = launch.actions.DeclareLaunchArgument(
        'world',
        default_value=[ThisLaunchFileDir(), '/world_only.model'],
        description='Full path to world file to load')

    declare_simulator_cmd = launch.actions.DeclareLaunchArgument(
        'simulator',
        default_value='gazebo',
        description='The simulator to use (gazebo or gzserver)')

    declare_map_yaml_cmd = launch.actions.DeclareLaunchArgument(
        'map_yaml_file',
        default_value=os.path.join(launch_dir, 'turtlebot3_world.yaml'),
        description='Full path to map file to load')

    declare_params_file_cmd = launch.actions.DeclareLaunchArgument(
        'params_file',
        default_value=[ThisLaunchFileDir(), '/nav2_params_namespaced.yaml'],
        description='Full path to the ROS2 parameters file to use for all launched nodes')

   # Start Gazebo with plugin providing the robot spawing service
    start_gazebo_cmd = launch.actions.ExecuteProcess(
        cmd=[simulator, '--verbose', '-s', 'libgazebo_ros_factory.so', world],
        output='screen')

    # Spawn two robots into Gazebo
    spawn_robots_cmds = []
    for robot in robots:
        spawn_robots_cmds.append(
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'spawn_robot_launch.py')),
                launch_arguments={
                    #TODO(orduno) Use text substitution
                                  'x_pose': robot['x_pose'],
                                  'y_pose': robot['y_pose'],
                                  'z_pose': robot['z_pose'],
                                  'robot_name': robot['name']
                                  }.items()))

    simulation_instances_cmds = []
    for robot in robots:
        group = GroupAction([
            # TODO(orduno) Each action.Node has two versions one with the required remaps and one
            #              without. The `use_remappings` flag specifies which runs.
            #              A better mechanism would be to have a PushNodeRemapping() action:
            #              https://github.com/ros2/launch_ros/issues/56
            #              For more on why we're remapping topics, see below
            # PushNodeRemapping(remappings)

            # Instances use the robot's name for namespace
            PushRosNamespace(robot['name']),
            launch.actions.IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(launch_dir, 'nav2_simulation_launch.py')),
                #TODO(orduno) pass the rviz config file
                launch_arguments={
                                  #TODO(orduno) might not be necessary to pass the robot name
                                  'robot_name': robot['name'],
                                  'use_simulator': 'False',
                                  'map_yaml_file': map_yaml_file,
                                  'params_file': params_file,
                                  'use_remappings': 'True'}.items())
        ])

        simulation_instances_cmds.append(group)

    # A note on the `remappings` variable defined above and the fact it's passed as a node arg.
    # A few topics have fully qualified names (have a leading '/'), these need to be remapped
    # to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # for multi-robot transforms:
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Add the actions to start gazebo, robots and simulations
    ld.add_action(start_gazebo_cmd)

    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    for simulation_instance_cmd in simulation_instances_cmds:
        ld.add_action(simulation_instance_cmd)

    return ld
