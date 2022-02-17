#! /usr/bin/env python3
# Copyright 2022 Samsung Research America
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

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterValue, ParameterType, Parameter
from rcl_interfaces.srv import SetParameters
from ament_index_python.packages import get_package_share_directory
import time

import math
import os 
import pickle 
import numpy as np

from random import seed
from random import randint
from random import uniform


def getPlannerResults(navigator, initial_pose, goal_pose, planners):
    results = []
    for planner in planners:
        path = navigator.getPath(initial_pose, goal_pose, planner)
        if path is not None:
            results.append(path)
    return results

def getRandomStart(costmap, max_cost, side_buffer, time_stamp, res):
    start = PoseStamped()
    start.header.frame_id = 'map'
    start.header.stamp = time_stamp
    while True:
        row = randint(side_buffer, costmap.shape[0]-side_buffer)
        col = randint(side_buffer, costmap.shape[1]-side_buffer)

        if costmap[row, col] < max_cost:
            start.pose.position.x = col*res
            start.pose.position.y = row*res

            yaw = uniform(0,1) * 2*math.pi
            quad = get_quaternion_from_euler(0.0, 0.0, yaw)
            start.pose.orientation.x = quad[0] 
            start.pose.orientation.y = quad[1]
            start.pose.orientation.z = quad[2]
            start.pose.orientation.w = quad[3]
            break
    return start

def getRandomGoal(costmap, start, max_cost, side_buffer, time_stamp, res):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = time_stamp
    while True:
        row = randint(side_buffer, costmap.shape[0]-side_buffer)
        col = randint(side_buffer, costmap.shape[1]-side_buffer)

        start_x = start.pose.position.x 
        start_y = start.pose.position.y
        goal_x = col*res
        goal_y = row*res
        x_diff = goal_x - start_x
        y_diff = goal_y - start_y
        dist = math.sqrt( x_diff ** 2 + y_diff ** 2)

        if costmap[row, col] < max_cost and dist > 3.0:
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y

            yaw = uniform(0,1) * 2*math.pi
            quad = get_quaternion_from_euler(0.0, 0.0, yaw)
            goal.pose.orientation.x = quad[0] 
            goal.pose.orientation.y = quad[1]
            goal.pose.orientation.z = quad[2]
            goal.pose.orientation.w = quad[3]
            break
    return goal

def get_quaternion_from_euler(roll, pitch, yaw):
    quad = []
    """
    Convert an Euler angle to a quaternion.
    
    Input
        :param roll: The roll (rotation around x-axis) angle in radians.
        :param pitch: The pitch (rotation around y-axis) angle in radians.
        :param yaw: The yaw (rotation around z-axis) angle in radians.
    
    Output
        :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
    """
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    quad = [qx, qy, qz, qw]
    return quad


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our experiment's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 1.0
    initial_pose.pose.position.y = 1.0
    initial_pose.pose.orientation.z = 0.999913
    initial_pose.pose.orientation.w = 0.0131738
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate
    navigator.waitUntilNav2Active()

    #Get the costmap for plotting and start/goal validation
    costmap_msg = navigator.getGlobalCostmap()
    costmap = np.asarray(costmap_msg.data)
    costmap.resize(costmap_msg.metadata.size_y, costmap_msg.metadata.size_x)

    # planners = ['NavFn', 'SmacLattice', 'Smac2D', 'SmacHybrid']
    planners = ['SmacLattice']
    max_cost = 210
    side_buffer = 10
    time_stamp = navigator.get_clock().now().to_msg()
    results = []
    for i in range(100):
        start = getRandomStart(costmap, max_cost, side_buffer, time_stamp, costmap_msg.metadata.resolution)
        goal = getRandomGoal(costmap, start, max_cost, side_buffer, time_stamp, costmap_msg.metadata.resolution)
        result = getPlannerResults(navigator, start, goal, planners)
        if len(result) == len(planners):  
            results.append(result)
        else:
            print("One of the planners was invalid")

    print("Write Results...")
    nav2_planner_metrics_dir = get_package_share_directory('nav2_planner_metrics')
    with open(os.path.join(nav2_planner_metrics_dir, 'results.pickle'),'wb') as f:
        pickle.dump(results, f, pickle.HIGHEST_PROTOCOL)

    with open(os.path.join(nav2_planner_metrics_dir, 'costmap.pickle'), 'wb') as f:
        pickle.dump(costmap, f, pickle.HIGHEST_PROTOCOL)

    with open(os.path.join(nav2_planner_metrics_dir, 'planners.pickle'), 'wb') as f:
        pickle.dump(planners, f, pickle.HIGHEST_PROTOCOL)
    print("Write Complete")

    navigator.lifecycleShutdown()
    exit(0)

if __name__ == '__main__':
    main()