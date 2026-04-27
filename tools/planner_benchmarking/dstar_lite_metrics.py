#! /usr/bin/env python3
# Copyright 2024 Nav2 Contributors
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

"""Benchmark DStarLite planner via Nav2 Python API.

Measures planning time for first-plan and incremental replan scenarios,
comparing against other planners.
"""

import glob
import math
import os
import pickle
from random import randint, seed, uniform
import sys
import time

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import numpy as np
import rclpy
from transforms3d.euler import euler2quat


def get_planner_results(
        navigator: BasicNavigator, initial_pose: PoseStamped,
        goal_pose: PoseStamped, planners: list) -> list:
    results = []
    for planner in planners:
        path = navigator._getPathImpl(initial_pose, goal_pose, planner, use_start=True)
        if path is not None and path.error_code == 0:
            results.append(path)
        else:
            print(planner, 'planner failed to produce the path')
            return results
    return results


def get_random_start(costmap, max_cost, side_buffer, time_stamp, res):
    start = PoseStamped()
    start.header.frame_id = 'map'
    start.header.stamp = time_stamp
    while True:
        row = randint(side_buffer, costmap.shape[0] - side_buffer)
        col = randint(side_buffer, costmap.shape[1] - side_buffer)
        if costmap[row, col] < max_cost:
            start.pose.position.x = col * res
            start.pose.position.y = row * res
            yaw = uniform(0, 1) * 2 * math.pi
            quad = euler2quat(0.0, 0.0, yaw)
            start.pose.orientation.w = quad[0]
            start.pose.orientation.x = quad[1]
            start.pose.orientation.y = quad[2]
            start.pose.orientation.z = quad[3]
            break
    return start


def get_random_goal(costmap, start, max_cost, side_buffer, time_stamp, res):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.header.stamp = time_stamp
    while True:
        row = randint(side_buffer, costmap.shape[0] - side_buffer)
        col = randint(side_buffer, costmap.shape[1] - side_buffer)
        goal_x = col * res
        goal_y = row * res
        x_diff = goal_x - start.pose.position.x
        y_diff = goal_y - start.pose.position.y
        dist = math.sqrt(x_diff ** 2 + y_diff ** 2)
        if costmap[row, col] < max_cost and dist > 3.0:
            goal.pose.position.x = goal_x
            goal.pose.position.y = goal_y
            yaw = uniform(0, 1) * 2 * math.pi
            quad = euler2quat(0.0, 0.0, yaw)
            goal.pose.orientation.w = quad[0]
            goal.pose.orientation.x = quad[1]
            goal.pose.orientation.y = quad[2]
            goal.pose.orientation.z = quad[3]
            break
    return goal


def get_path_length(path):
    if path is None or len(path.poses) < 2:
        return 0.0
    length = 0.0
    x_prev = path.poses[0].pose.position.x
    y_prev = path.poses[0].pose.position.y
    for i in range(1, len(path.poses)):
        x_curr = path.poses[i].pose.position.x
        y_curr = path.poses[i].pose.position.y
        length += math.sqrt((x_curr - x_prev) ** 2 + (y_curr - y_prev) ** 2)
        x_prev = x_curr
        y_prev = y_curr
    return length


def get_planning_time(result):
    """Extract planning time in seconds from a ComputePathToPose result."""
    return result.planning_time.nanosec / 1e09 + result.planning_time.sec


def main():
    rclpy.init()

    navigator = BasicNavigator()
    map_path = os.getcwd() + '/' + glob.glob('**/100by100_20.yaml', recursive=True)[0]
    navigator.changeMap(map_path)
    time.sleep(2)

    costmap_msg = navigator.getGlobalCostmap()
    costmap = np.asarray(costmap_msg.data)
    costmap.resize(costmap_msg.metadata.size_y, costmap_msg.metadata.size_x)

    planners = ['Navfn', 'Smac2d', 'DStarLite']
    max_cost = 210
    side_buffer = 100
    time_stamp = navigator.get_clock().now().to_msg()
    results = []
    first_times = []   # first plan times per planner
    seed(33)
    random_pairs = 50
    res = costmap_msg.metadata.resolution

    i = 0
    while len(results) != random_pairs:
        print(f'Cycle: {i + 1} / {random_pairs}')
        start = get_random_start(costmap, max_cost, side_buffer, time_stamp, res)
        goal = get_random_goal(costmap, start, max_cost, side_buffer, time_stamp, res)
        result = get_planner_results(navigator, start, goal, planners)

        if len(result) == len(planners):
            pair_times = []
            for r in result:
                pair_times.append(get_planning_time(r))
            first_times.append(pair_times)
            results.append(result)
            i += 1
        else:
            print('  One of the planners was invalid, skipping pair')

    print('Write Results...')
    with open(os.getcwd() + '/dstar_lite_results.pickle', 'wb+') as f:
        pickle.dump(results, f, pickle.HIGHEST_PROTOCOL)
    with open(os.getcwd() + '/dstar_lite_costmap.pickle', 'wb+') as f:
        pickle.dump(costmap_msg, f, pickle.HIGHEST_PROTOCOL)
    with open(os.getcwd() + '/dstar_lite_planners.pickle', 'wb+') as f:
        pickle.dump(planners, f, pickle.HIGHEST_PROTOCOL)
    with open(os.getcwd() + '/dstar_lite_first_times.pickle', 'wb+') as f:
        pickle.dump(first_times, f, pickle.HIGHEST_PROTOCOL)
    print('Write Complete')
    sys.exit(0)


if __name__ == '__main__':
    main()
