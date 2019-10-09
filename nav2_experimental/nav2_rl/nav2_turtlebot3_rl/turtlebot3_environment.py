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

from gazebo_interface import GazeboInterface

from time import sleep

import rclpy
from rclpy.qos import qos_profile_sensor_data

import parameters

from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import GetEntityState

class Turtlebot3Environment(GazeboInterface):
    def __init__(self):
        super().__init__()
        self.Turtlebot3Environment = Turtlebot3Environment
        self.act = 0
        self.done = False
        self.actions = self.get_actions()

        self.collision = False
        self.collision_tol = 0.125
        self.laser_scan_range = [0] * 360
        self.states_input = [3.5] * 8
        self.zero_div_tol = 0.01
        self.range_min = 0.0
        self.states = []

        self.current_pose = Pose()
        self.goal_pose = Pose()

        self.pub_cmd_vel = self.node_.create_publisher(Twist, 'cmd_vel', 1)
        self.sub_scan = self.node_.create_subscription(LaserScan, '/turtlebot3_laserscan/out', self.scan_callback,
                                                       qos_profile_sensor_data)
        self.scan_msg_received = False

    def scan_callback(self, LaserScan):
        self.scan_msg_received = True
        self.laser_scan_range.clear()
        self.laser_scan_range = []
        self.range_min = LaserScan.range_min
        range_max = LaserScan.range_max
        for i in range(len(LaserScan.ranges)):
            if LaserScan.ranges[i] == float('Inf'):
                self.laser_scan_range.append(range_max)
            elif LaserScan.ranges[i] < self.range_min + self.collision_tol - 0.05:
                self.laser_scan_range.append(self.range_min + self.collision_tol)
            else:
                self.laser_scan_range.append(LaserScan.ranges[i])

        self.states_input.clear()
        self.states_input = []
        for i in range(8):
            step = int(len(LaserScan.ranges) / 8)
            self.states_input.append(min(self.laser_scan_range[i * step:(i + 1) * step],
                                     default=0))
        if self.check_collision():
            self.collision = True
            self.done = True

    def check_collision(self):
        if min(self.states_input) < self.range_min + self.collision_tol:
            print("Colistion proximity... " + str(min(self.laser_scan_range)))
            return True
        return False

    def action_space_size(self):
        return len(self.actions)

    def observation_space_size(self):
        return len(self.states)

    def stop_action(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        self.pub_cmd_vel.publish(vel_cmd)

    def get_robot_pose(self):
        while not self.get_entity_state.wait_for_service(timeout_sec=1.0):
            print('get entity state service is not available...')
        req = GetEntityState.Request()
        req.name = 'turtlebot3_waffle'
        future = self.get_entity_state.call_async(req)

        while not future.done() and rclpy.ok():
            sleep(0.01 / self.time_factor)

        self.current_pose.position = future.result().state.pose.position
        self.current_pose.orientation = future.result().state.pose.orientation

    def reset_tb3_env(self):
        self.unpause_gazebo_world()
        self.stop_action()
        self.reset_gazebo_world()

        self.time_factor = self.get_time_factor()
        self.scan_msg_received = False

        self.stop_action()

        self.laser_scan_range = [0] * 360
        self.states_input = [3.5] * 8
        while not self.scan_msg_received and rclpy.ok():
            sleep(0.1 / self.time_factor)
        self.collision = False
        self.done = False

    def get_velocity_cmd(self):
        """gets the velocity cmd from action

        # Argument
            None

        # Returns
            Select velocity cmd's from the action list
        """
        raise NotImplementedError()

    def step(self, action):
        # Unpause environment
        self.unpause_gazebo_world()
        vel_cmd = Twist()
        self.act = action
        vel_cmd.linear.x, vel_cmd.linear.y, vel_cmd.angular.z = self.get_velocity_cmd(action)
        self.pub_cmd_vel.publish(vel_cmd)
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        sleep(parameters.LOOP_RATE / self.time_factor)
        get_reward = self.compute_reward()
        # Pause environment
        self.pause_gazebo_world()

        return self.observation(), get_reward[0], self.done, {}
