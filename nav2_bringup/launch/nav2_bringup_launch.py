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

import os

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

import launch.actions


def generate_launch_description():

    # Configuration parameters for the launch

    use_gui = launch.substitutions.LaunchConfiguration('use_gui')
    use_simulation = launch.substitutions.LaunchConfiguration('use_simulation')
    simulator = launch.substitutions.LaunchConfiguration('simulator')
    world = launch.substitutions.LaunchConfiguration('world')
    params_file = launch.substitutions.LaunchConfiguration('params',
            default=[launch.substitutions.ThisLaunchFileDir(), '/nav2_params.yaml'])

    declare_use_gui_cmd = launch.actions.DeclareLaunchArgument(
            'use_gui', condition=IfCondition('True'),
            default_value='True', description='Whether to bring up the GUI interface')

    declare_use_simulation_cmd = launch.actions.DeclareLaunchArgument(
            'use_simulation', condition=IfCondition('True'),
            default_value='True', description='Whether to run in simulation')

    declare_simulator_cmd = launch.actions.DeclareLaunchArgument(
            'simulator', 
            default_value='gzserver', description='The simulator to use (gazebo or gzserver)')

    declare_world_cmd = launch.actions.DeclareLaunchArgument(
            'world',
            default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'worlds/turtlebot3.world'),
            description='Full path to world file to load')

    declare_params_file_cmd = launch.actions.DeclareLaunchArgument(
            'params_file',
            description='The ROS2 parameters file to use')

    launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    gz = launch.substitutions.LaunchConfiguration('gz', default=['gzserver'])

    # Specify the actions

    start_gazebo_cmd = launch.actions.ExecuteProcess(
        condition=IfCondition(use_simulation),
        cmd=[simulator, '-s', 'libgazebo_ros_init.so', world, ['__params:=', params_file]],
        cwd=[launch_dir], output='screen')

    start_robot_state_publisher_cmd = launch.actions.ExecuteProcess(
        condition=IfCondition(use_simulation),
        cmd=[
            os.path.join(
                get_package_prefix('robot_state_publisher'),
                'lib/robot_state_publisher/robot_state_publisher'),
            os.path.join(
                get_package_share_directory('turtlebot3_description'),
                'urdf', 'turtlebot3_burger.urdf'),
            ['__params:=', params_file]],
        cwd=[launch_dir],
        output='screen')

    start_map_server_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_map_server'),
                'lib/nav2_map_server/map_server'),
            ['__params:=', params_file]],
        cwd=[launch_dir], output='screen')

    start_localizer_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_amcl'),
                'lib/nav2_amcl/amcl'),
            ['__params:=', params_file]],
        cwd=[launch_dir], output='screen')

    start_world_model_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_world_model'),
                'lib/nav2_world_model/world_model'),
            ['__params:=', params_file]],
        cwd=[launch_dir],
        output='screen')

    start_dwb_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('dwb_controller'),
                'lib/dwb_controller/dwb_controller'),
            ['__params:=', params_file]],
        cwd=[launch_dir],
        output='screen')

    start_planner_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_navfn_planner'),
                'lib/nav2_navfn_planner/navfn_planner'),
            ['__params:=', params_file]],
        cwd=[launch_dir],
        output='screen')

    start_navigator_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_simple_navigator'),
                'lib/nav2_simple_navigator/simple_navigator'),
            ['__params:=', params_file]],
        cwd=[launch_dir],
        output='screen')

    start_controller_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_controller'),
                'lib/nav2_controller/nav2_controller'),
            ['__params:=', params_file]],
        cwd=[launch_dir],
        output='screen')

    start_cli_cmd = launch.actions.ExecuteProcess(
        condition=UnlessCondition(use_gui),
        cmd=[
            os.path.join(
                get_package_prefix('nav2_controller'),
                'lib/nav2_controller/nav2_cli'),
            'startup',
            ['__params:=', params_file]],
        cwd=[launch_dir],
        output='screen')

    cli_exit_event_handler = launch.actions.RegisterEventHandler(
        condition=UnlessCondition(use_gui),
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=start_cli_cmd,
            on_exit=launch.actions.EmitEvent(event=launch.events.Shutdown(reason='Done!'))))

    start_gui_cmd = launch.actions.ExecuteProcess(
        condition=IfCondition(use_gui),
        cmd=[os.path.join(get_package_prefix('nav2_controller'), 'bin/gui/nav2_gui')],
        cwd=[os.path.join(get_package_prefix('nav2_controller'), 'bin/gui')],
        output='screen')

    gui_exit_event_handler = launch.actions.RegisterEventHandler(
        condition=IfCondition(use_gui),
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=start_gui_cmd,
            on_exit=launch.actions.EmitEvent(event=launch.events.Shutdown(reason='Done!'))))

    # Compose the launch description

    ld = launch.LaunchDescription()

    ld.add_action(declare_use_gui_cmd)
    ld.add_action(declare_use_simulation_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_localizer_cmd)
    ld.add_action(start_world_model_cmd)
    ld.add_action(start_dwb_cmd)
    ld.add_action(start_planner_cmd)
    ld.add_action(start_navigator_cmd)
    ld.add_action(start_controller_cmd)
    ld.add_action(start_cli_cmd)
    ld.add_action(cli_exit_event_handler)
    ld.add_action(start_gui_cmd)
    ld.add_action(gui_exit_event_handler)

    return ld
