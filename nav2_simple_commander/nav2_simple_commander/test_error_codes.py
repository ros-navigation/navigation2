#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from nav2_msgs.action import ComputePathThroughPoses, ComputePathToPose

"""
Basic navigation demo to go to poses.
"""


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()

    # Initial pose does not respect the orientation... not sure why
    initial_pose.pose.position.x = -2.00
    initial_pose.pose.position.y = -0.50
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    navigator.setInitialPose(initial_pose)

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    # Initial pose does not respect the orientation... not sure why
    goal_pose.pose.position.x = -1.00
    goal_pose.pose.position.y = -0.50
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    compute_path_to_pose = {
        'unknown': ComputePathToPose.Goal().UNKNOWN,
        'tf_error': ComputePathToPose.Goal().TF_ERROR,
        'start_outside_map': ComputePathToPose.Goal().START_OUTSIDE_MAP,
        'goal_outside_map': ComputePathToPose.Goal().GOAL_OUTSIDE_MAP,
        'start_occupied': ComputePathToPose.Goal().START_OCCUPIED,
        'goal_occupied': ComputePathToPose.Goal().GOAL_OCCUPIED,
        'timeout': ComputePathToPose.Goal().TIMEOUT,
        'no_valid_path': ComputePathToPose.Goal().NO_VALID_PATH}

    for planner, error_code in compute_path_to_pose.items():
        result = navigator._getPathImpl(initial_pose, goal_pose, planner)
        assert result.error_code == error_code
        # print("Planner")

   # Check the compute to pose error codes

    # Check the compute through error codes

    # Check the follow path error codes

    navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()
