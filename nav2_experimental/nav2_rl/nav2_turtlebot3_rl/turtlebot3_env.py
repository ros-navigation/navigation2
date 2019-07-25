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

from time import sleep
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor

import numpy as np
import math
import random
import pandas
import parameters

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from std_msgs.msg import String
from gazebo_msgs.srv import GetEntityState, SetEntityState


class TurtlebotEnv():
    def __init__(self):
        self.node_ = rclpy.create_node('turtlebot3_env')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node_)
        self.act = 0
        self.done = False
        self.actions = [[parameters.LINEAR_FWD_VELOCITY, parameters.ANGULAR_FWD_VELOCITY],
                        [parameters.LINEAR_STR_VELOCITY, parameters.ANGULAR_VELOCITY],
                        [parameters.LINEAR_STR_VELOCITY, -parameters.ANGULAR_VELOCITY]]
        self.bonous_reward = 0

        self.collision = False
        self.collision_tol = 0.125
        self.laser_scan_range = [0] * 360
        self.states_input = [3.5] * 8
        self.zero_div_tol = 0.01
        self.range_min = 0.0

        self.pub_cmd_vel = self.node_.create_publisher(Twist, 'cmd_vel', 1)
        self.sub_scan = self.node_.create_subscription(LaserScan, 'scan', self.scan_callback,
                                                       qos_profile_sensor_data)

        self.reset_simulation = self.node_.create_client(Empty, 'reset_simulation')
        self.reset_world = self.node_.create_client(Empty, 'reset_world')
        self.unpause_proxy = self.node_.create_client(Empty, 'unpause_physics')
        self.pause_proxy = self.node_.create_client(Empty, 'pause_physics')
        self.get_entity_state = self.node_.create_client(GetEntityState, 'get_entity_state')
        self.set_entity_state = self.node_.create_client(SetEntityState, 'set_entity_state')
        self.scan_msg_received = False
        self.t = Thread(target=self.executor.spin)
        self.t.start()

    def cleanup(self):
        self.t.join()

    def get_reward(self):
        reward = 0
        if self.collision is True:
            reward = -10
            self.done = True
            return reward, self.done
        elif self.collision is False and self.act is 0:
            if abs(min(self.states_input)) >= self.zero_div_tol:
                reward = 0.08 - (1 / (min(self.states_input)**2)) * 0.005
            else:
                reward = -10
            if reward > 0:
                self.bonous_reward += reward
                reward = self.bonous_reward
        else:
            bonous_discount_factor = 0.6
            self.bonous_reward *= bonous_discount_factor
            if abs(min(self.states_input)) >= self.zero_div_tol:
                reward = 0.02 - (1 / min(self.states_input)) * 0.005
            else:
                reward = -10
        return reward, self.done

    def scan_callback(self, LaserScan):
        self.scan_msg_received = True
        self.laser_scan_range = []
        self.range_min = LaserScan.range_min
        range_max = LaserScan.range_max

        for i in range(len(LaserScan.ranges)):
            if LaserScan.ranges[i] == float('Inf'):
                self.laser_scan_range.append(range_max)
            elif LaserScan.ranges[i] < self.range_min:
                self.laser_scan_range.append(self.range_min + self.collision_tol)
            else:
                self.laser_scan_range.append(LaserScan.ranges[i])
        if self.check_collision():
            self.collision = True
            self.done = True
        self.states_input = []
        for i in range(8):
            step = int(len(LaserScan.ranges) / 8)
            self.states_input.append(min(self.laser_scan_range[i * step:(i + 1) * step],
                                     default=0))

    def action_space(self):
        return len(self.actions)

    def step(self, action):
        vel_cmd = Twist()
        self.act = action
        vel_cmd.linear.x = self.actions[action][0]
        vel_cmd.angular.z = self.actions[action][1]
        self.pub_cmd_vel.publish(vel_cmd)
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        get_reward = self.get_reward()
        return self.states_input, get_reward[0], self.done

    def check_collision(self):
        if min(self.laser_scan_range) < self.range_min + self.collision_tol:
            print("Near collision detected... " + str(min(self.laser_scan_range)))
            return True
        return False

    def stop_action(self):
        vel_cmd = Twist()
        vel_cmd.linear.x = 0.0
        vel_cmd.angular.z = 0.0
        self.pub_cmd_vel.publish(vel_cmd)

    def reset(self):
        self.scan_msg_received = False
        self.stop_action
        while not self.reset_world.wait_for_service(timeout_sec=1.0):
            print('Reset world service is not available...')
        self.reset_world.call_async(Empty.Request())

        while not self.reset_simulation.wait_for_service(timeout_sec=1.0):
            print('Reset simulation service is not available...')
        self.reset_simulation.call_async(Empty.Request())

        self.laser_scan_range = [0] * 360
        self.states_input = [3.5] * 8
        while not self.scan_msg_received and rclpy.ok():
            sleep(0.1)
        self.collision = False
        self.done = False
        self.bonous_reward = 0
        return self.states_input
