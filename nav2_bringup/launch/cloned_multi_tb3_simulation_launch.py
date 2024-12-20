# Copyright (c) 2023 LG Electronics.
# Copyright (c) 2024 Open Navigation LLC
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
from pathlib import Path
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext, LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, GroupAction,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
import yaml


def parse_robots_argument(robots_arg: str):
    robots_dict = {}
    if robots_arg:
        robots_list = robots_arg.split(';')
        for robot in robots_list:
            key, value = robot.split('=')
            key = key.strip()
            value = yaml.safe_load(value.strip())
            value.setdefault('x', 0.0)
            value.setdefault('y', 0.0)
            value.setdefault('z', 0.0)
            value.setdefault('roll', 0.0)
            value.setdefault('pitch', 0.0)
            value.setdefault('yaw', 0.0)
            robots_dict[key] = value
    return robots_dict


def launch_setup(context: LaunchContext, *args, **kwargs):
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Simulation settings. On this example all robots are launched with the same settings
    world = LaunchConfiguration('world')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config')
    use_robot_state_pub = LaunchConfiguration('use_robot_state_pub')
    use_rviz = LaunchConfiguration('use_rviz')
    log_settings = LaunchConfiguration('log_settings', default='true')

    # Start Gazebo with plugin providing the robot spawning service
    world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    world_sdf_xacro = ExecuteProcess(
        cmd=['xacro', '-o', world_sdf, ['headless:=', 'False'], world])
    start_gazebo_cmd = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_sdf],
        output='screen',
    )

    remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
        on_shutdown=[
            OpaqueFunction(function=lambda _: os.remove(world_sdf))
        ]))

    robots_string = LaunchConfiguration('robots').perform(context)
    robots_list = parse_robots_argument(robots_string)

    # Define commands for launching the navigation instances
    bringup_cmd_group = []
    for robot_name in robots_list:
        init_pose = robots_list[robot_name]
        group = GroupAction(
            [
                LogInfo(
                    msg=[
                        'Launching namespace=',
                        robot_name,
                        ' init_pose=',
                        str(init_pose),
                    ]
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(launch_dir, 'rviz_launch.py')
                    ),
                    condition=IfCondition(use_rviz),
                    launch_arguments={
                        'namespace': TextSubstitution(text=robot_name),
                        'rviz_config': rviz_config_file,
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(bringup_dir, 'launch',
                                     'tb3_simulation_launch.py')
                    ),
                    launch_arguments={
                        'namespace': robot_name,
                        'map': map_yaml_file,
                        'use_sim_time': 'True',
                        'params_file': params_file,
                        'autostart': autostart,
                        'use_rviz': 'False',
                        'use_simulator': 'False',
                        'headless': 'False',
                        'use_robot_state_pub': use_robot_state_pub,
                        'x_pose': TextSubstitution(text=str(init_pose['x'])),
                        'y_pose': TextSubstitution(text=str(init_pose['y'])),
                        'z_pose': TextSubstitution(text=str(init_pose['z'])),
                        'roll': TextSubstitution(text=str(init_pose['roll'])),
                        'pitch': TextSubstitution(text=str(init_pose['pitch'])),
                        'yaw': TextSubstitution(text=str(init_pose['yaw'])),
                        'robot_name': TextSubstitution(text=robot_name),
                    }.items(),
                ),
            ]
        )

        bringup_cmd_group.append(group)

    launch_actions = []

    # Add the actions to start gazebo, robots and simulations
    launch_actions.append(world_sdf_xacro)
    launch_actions.append(start_gazebo_cmd)
    launch_actions.append(remove_temp_sdf_file)

    launch_actions.append(
        LogInfo(msg=['number_of_robots=', str(len(robots_list))]))

    launch_actions.append(
        LogInfo(condition=IfCondition(log_settings),
                msg=['map yaml: ', map_yaml_file])
    )
    launch_actions.append(
        LogInfo(condition=IfCondition(log_settings),
                msg=['params yaml: ', params_file])
    )
    launch_actions.append(
        LogInfo(
            condition=IfCondition(log_settings),
            msg=['rviz config file: ', rviz_config_file],
        )
    )
    launch_actions.append(
        LogInfo(
            condition=IfCondition(log_settings),
            msg=['using robot state pub: ', use_robot_state_pub],
        )
    )
    launch_actions.append(
        LogInfo(condition=IfCondition(log_settings),
                msg=['autostart: ', autostart])
    )

    for cmd in bringup_cmd_group:
        launch_actions.append(cmd)

    return launch_actions


def generate_launch_description():
    """
    Bring up the multi-robots with given launch arguments.

    Launch arguments consist of robot name(which is namespace) and pose for initialization.
    Keep general yaml format for pose information.
    ex) robots:='robot1={x: 1.0, y: 1.0, yaw: 1.5707}; robot2={x: 1.0, y: 1.0, yaw: 1.5707}'
    ex) robots:='robot3={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707};
                 robot4={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}'
    """
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(
                sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
            description='Full path to world file to load',
        ),
        DeclareLaunchArgument(
            'robots',
            description='Robot namespaces and poses (required)'
        ),
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(
                bringup_dir, 'maps', 'tb3_sandbox.yaml'),
            description='Full path to map file to load',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(
                bringup_dir, 'params', 'nav2_params.yaml'
            ),
            description='Full path to the ROS2 parameters file to use for all launched nodes',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='false',
            description='Automatically startup the stacks',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                bringup_dir, 'rviz', 'nav2_default_view.rviz'),
            description='Full path to the RVIZ config file to use.',
        ),
        DeclareLaunchArgument(
            'use_robot_state_pub',
            default_value='True',
            description='Whether to start the robot state publisher',
        ),
        DeclareLaunchArgument(
            'use_rviz', default_value='True', description='Whether to start RVIZ'
        ),
        # Set the path to the simulation resources
        AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH', os.path.join(sim_dir, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(
            Path(os.path.join(sim_dir)).parent.resolve())),
        OpaqueFunction(function=launch_setup)
    ])
