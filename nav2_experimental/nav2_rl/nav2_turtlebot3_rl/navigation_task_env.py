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

from turtlebot3_environment import Turtlebot3Environment

import numpy as np
from math import pi, atan2, sin, cos
import math
import random

from geometry_msgs.msg import Twist, Pose

class NavigationTaskEnv(Turtlebot3Environment):
    def __init__(self):
        super().__init__()
        self.NavigationTaskEnv = NavigationTaskEnv

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

    def get_actions(self):
        """Defines the actions that the environment can have

        # Argument
            None

        # Returns
            The list of possible actions
        """
        raise NotImplementedError()

    def compute_reward(self):

        reward = 0.0
        goal_dist_sq = self.sq_distance_to_goal()

        obstacle_reward = - (1 / (min(self.states_input)**2)) * 0.05

        if goal_dist_sq > 0.25:
            distance_reward = -(goal_dist_sq)
            heading_reward = -0.5 * self.get_heading()**2
        else:
            distance_reward = 1000
            heading_reward = 1000
            self.done = True
            print("Goal Reached")

        heading_reward = -0.5 * self.get_heading()**2

        reward += distance_reward
        reward += heading_reward
        reward += obstacle_reward

        if self.collision:
            reward = -500
            self.done = True
        return reward, self.done

    def set_random_robot_pose(self):
        self.set_entity_state_pose('turtlebot3_waffle', self.get_random_pose())

    def set_random_goal_pose(self):
        self.goal_pose = self.get_random_pose()
        self.set_entity_state_pose('goal_pose', self.goal_pose)

    def get_random_pose(self):
        random_pose = Pose()
        yaw = random.uniform(0, pi * 2)

        random_pose.position.x = random.uniform(-3, 3)
        random_pose.position.y = random.uniform(-3, 3)
        random_pose.position.z = 0.0
        random_pose.orientation.x = 0.0
        random_pose.orientation.y = 0.0
        random_pose.orientation.z = sin(yaw * 0.5)
        random_pose.orientation.w = cos(yaw * 0.5)
        return random_pose

    def sq_distance_to_goal(self):
        self.get_robot_pose()
        dx = self.goal_pose.position.x - self.current_pose.position.x
        dy = self.goal_pose.position.y - self.current_pose.position.y
        return dx * dx + dy * dy

    def get_heading(self):
        goal_angle = math.atan2(self.goal_pose.position.y - self.current_pose.position.y,
                                self.goal_pose.position.x - self.current_pose.position.x)

        current_yaw = self.get_yaw(self.current_pose)
        heading = goal_angle - current_yaw

        if heading > pi:
            heading -= 2 * pi

        elif heading < -pi:
            heading += 2 * pi

        return heading

    def get_yaw(self, q):
        yaw = atan2(2.0 * (q.orientation.x * q.orientation.y + q.orientation.w * q.orientation.z),
                    q.orientation.w * q.orientation.w + q.orientation.x * q.orientation.x -
                    q.orientation.y * q.orientation.y - q.orientation.z * q.orientation.z)
        return yaw

    def observation(self):
        self.get_robot_pose()
        self.states.clear()
        self.states = [0] * (len(self.states_input) + 6)
        self.states[:8] = self.states_input[:]

        sq_dist = self.sq_distance_to_goal()
        heading = self.get_heading()

        self.states[8] = sq_dist
        self.states[9] = heading
        self.states[10] = float(self.current_pose.position.x)
        self.states[11] = float(self.current_pose.position.y)
        self.states[12] = float(self.goal_pose.position.x)
        self.states[13] = float(self.goal_pose.position.y)

        return self.states

    def reset(self):
        self.reset_tb3_env()
        self.set_random_robot_pose()
        self.set_random_goal_pose()
        return self.observation()