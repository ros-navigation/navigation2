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
import sys
import rclpy
import time
import asyncio

from ament_index_python.packages import get_package_prefix
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition

import launch.actions
import subprocess
import termios
import threading
import tty

from std_srvs.srv import Empty


class KeyboardController():
    def __init__(self, context):
        self.node = rclpy.create_node('kbd_controller_client')
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        self.context = context
        tty.setcbreak(self.fd)

    def __del__(self):
        self.node.destroy_node()
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def call_service(self, service_name):
        cli = self.node.create_client(Empty, service_name)
        req = Empty.Request()
        while not cli.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('service not available, waiting again...')
        future = cli.call_async(req)
        # rclpy.spin_until_future_complete(self.node, future)

    def monitor_input(self):
        while True:
            ch = sys.stdin.read(1)
            if (ch == "u"):
                self.call_service('startup')
            elif (ch == "p"):
                self.call_service('pause')
            elif (ch == "r"):
                self.call_service('resume')
            elif (ch == "d"):
                self.call_service('shutdown')
                time.sleep(1)
                loop = asyncio.new_event_loop()
                task = loop.create_task(self.context.emit_event(launch.events.Shutdown(reason="Done")))
                loop.run_until_complete(task)
                return


class StartKeyboardController(launch.Action):
    def execute(self, context: launch.LaunchContext):
        kb = KeyboardController(context)
        thread = threading.Thread(target=kb.monitor_input)
        thread.start()


def generate_launch_description():
    use_gui = launch.substitutions.LaunchConfiguration('use_gui')
    use_simulation = launch.substitutions.LaunchConfiguration('use_simulation')
    simulator = launch.substitutions.LaunchConfiguration('simulator')
    world = launch.substitutions.LaunchConfiguration('world')
    params_file = launch.substitutions.LaunchConfiguration(
        'params', default=[launch.substitutions.ThisLaunchFileDir(), '/nav2_params.yaml'])

    declare_use_gui_cmd = launch.actions.DeclareLaunchArgument(
        'use_gui', condition=IfCondition('True'),
        default_value='False', description='Whether to bring up the GUI interface')

    declare_use_simulation_cmd = launch.actions.DeclareLaunchArgument(
        'use_simulation', condition=IfCondition('True'),
        default_value='True', description='Whether to run in simulation')

    declare_simulator_cmd = launch.actions.DeclareLaunchArgument(
        'simulator',
        default_value='gzserver', description='The simulator to use (gazebo or gzserver)')

    declare_world_cmd = launch.actions.DeclareLaunchArgument(
        'world',
        default_value=os.path.join(
            get_package_share_directory('turtlebot3_gazebo'), 'worlds/turtlebot3.world'),
        description='Full path to world file to load')

    declare_params_file_cmd = launch.actions.DeclareLaunchArgument(
        'params_file',
        description='Full path to the ROS2 parameters file to use for all launched nodes')

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
        cwd=[launch_dir], output='screen')

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
        cwd=[launch_dir], output='screen')

    start_dwb_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('dwb_controller'),
                'lib/dwb_controller/dwb_controller'),
            ['__params:=', params_file]],
        cwd=[launch_dir], output='screen')

    start_planner_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_navfn_planner'),
                'lib/nav2_navfn_planner/navfn_planner'),
            ['__params:=', params_file]],
        cwd=[launch_dir], output='screen')

    start_navigator_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_simple_navigator'),
                'lib/nav2_simple_navigator/simple_navigator'),
            ['__params:=', params_file]],
        cwd=[launch_dir], output='screen')

    start_controller_cmd = launch.actions.ExecuteProcess(
        cmd=[
            os.path.join(
                get_package_prefix('nav2_controller'),
                'lib/nav2_controller/nav2_controller'),
            ['__params:=', params_file]],
        cwd=[launch_dir], output='screen')

    start_gui_cmd = launch.actions.ExecuteProcess(
        cmd=[os.path.join(get_package_prefix('nav2_controller'), 'bin/gui/nav2_gui')],
        cwd=[os.path.join(get_package_prefix('nav2_controller'), 'bin/gui')],
        output='screen')

    gui_exit_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=start_gui_cmd,
            on_exit=launch.actions.EmitEvent(event=launch.events.Shutdown(reason='Done!'))))

    # Create a group for the GUI-related actions, conditioned on the 'use_gui' launch option
    gui_group = launch.actions.GroupAction(
        condition=IfCondition(use_gui),
        actions=[start_gui_cmd, gui_exit_event_handler])

    # Create a group for the CLI-related actions, used when !use_gui
    cmdline_group = launch.actions.GroupAction(
        condition=UnlessCondition(use_gui),
        actions=[StartKeyboardController()])

    # Create the launch description and populate
    ld = launch.LaunchDescription()

    # First, declare the launch options
    ld.add_action(declare_use_gui_cmd)
    ld.add_action(declare_use_simulation_cmd)
    ld.add_action(declare_simulator_cmd)
    ld.add_action(declare_world_cmd)

    # Then, add the actions to launch the simulator-related nodes (conditioned on 'use_simulation')
    ld.add_action(start_gazebo_cmd)
    ld.add_action(start_robot_state_publisher_cmd)

    # Next, add the actions to launch all of the navigation nodes
    ld.add_action(start_map_server_cmd)
    ld.add_action(start_localizer_cmd)
    ld.add_action(start_world_model_cmd)
    ld.add_action(start_dwb_cmd)
    ld.add_action(start_planner_cmd)
    ld.add_action(start_navigator_cmd)
    ld.add_action(start_controller_cmd)

    # Finally, launch an interface to the controller node, either a GUI or CLI
    ld.add_action(gui_group)
    ld.add_action(cmdline_group)

    return ld
