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

import sys
import threading
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import (
    ComputePathThroughPoses,
    ComputePathToPose,
    FollowPath,
    SmoothPath,
)
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
import rclpy


def main(argv=sys.argv[1:]):
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()

    # Initial pose
    initial_pose.pose.position.x = -2.00
    initial_pose.pose.position.y = -0.50
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0

    # Follow path error codes
    path = Path()

    goal_pose = initial_pose
    goal_pose.pose.position.x += 0.25

    goal_pose1 = goal_pose
    goal_pose1.pose.position.x += 0.25

    path.poses.append(initial_pose)
    path.poses.append(goal_pose)
    path.poses.append(goal_pose1)

    navigator._waitForNodeToActivate('controller_server')
    follow_path = {
        'unknown': FollowPath.Result().UNKNOWN,
        'invalid_controller': FollowPath.Result().INVALID_CONTROLLER,
        'tf_error': FollowPath.Result().TF_ERROR,
        'invalid_path': FollowPath.Result().INVALID_PATH,
        'patience_exceeded': FollowPath.Result().PATIENCE_EXCEEDED,
        'failed_to_make_progress': FollowPath.Result().FAILED_TO_MAKE_PROGRESS,
        'no_valid_control': FollowPath.Result().NO_VALID_CONTROL,
    }

    for controller, error_code in follow_path.items():
        success = navigator.followPath(path, controller)

        if success:
            while not navigator.isTaskComplete():
                time.sleep(0.5)

            assert (
                navigator.result_future.result().result.error_code == error_code
            ), 'Follow path error code does not match'

        else:
            assert False, 'Follow path was rejected'

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()

    # Check compute path to pose error codes
    goal_pose.pose.position.x = -1.00
    goal_pose.pose.position.y = -0.50
    goal_pose.pose.orientation.z = 0.0
    goal_pose.pose.orientation.w = 1.0

    navigator._waitForNodeToActivate('planner_server')
    compute_path_to_pose = {
        'unknown': ComputePathToPose.Result().UNKNOWN,
        'invalid_planner': ComputePathToPose.Result().INVALID_PLANNER,
        'tf_error': ComputePathToPose.Result().TF_ERROR,
        'start_outside_map': ComputePathToPose.Result().START_OUTSIDE_MAP,
        'goal_outside_map': ComputePathToPose.Result().GOAL_OUTSIDE_MAP,
        'start_occupied': ComputePathToPose.Result().START_OCCUPIED,
        'goal_occupied': ComputePathToPose.Result().GOAL_OCCUPIED,
        'timeout': ComputePathToPose.Result().TIMEOUT,
        'no_valid_path': ComputePathToPose.Result().NO_VALID_PATH,
    }

    for planner, error_code in compute_path_to_pose.items():
        result = navigator._getPathImpl(initial_pose, goal_pose, planner)
        assert (
            result.error_code == error_code
        ), 'Compute path to pose error does not match'

    def cancel_task():
        time.sleep(1)
        navigator.goal_handle.cancel_goal_async()

    # Check compute path to pose cancel
    threading.Thread(target=cancel_task).start()
    result = navigator._getPathImpl(initial_pose, goal_pose, 'cancelled')
    assert (
        navigator.getResult() == TaskResult.CANCELED
    ), 'Compute path to pose cancel failed'

    # Check compute path through error codes
    goal_pose1 = goal_pose
    goal_poses = [goal_pose, goal_pose1]

    compute_path_through_poses = {
        'unknown': ComputePathThroughPoses.Result().UNKNOWN,
        'invalid_planner': ComputePathThroughPoses.Result().INVALID_PLANNER,
        'tf_error': ComputePathThroughPoses.Result().TF_ERROR,
        'start_outside_map': ComputePathThroughPoses.Result().START_OUTSIDE_MAP,
        'goal_outside_map': ComputePathThroughPoses.Result().GOAL_OUTSIDE_MAP,
        'start_occupied': ComputePathThroughPoses.Result().START_OCCUPIED,
        'goal_occupied': ComputePathThroughPoses.Result().GOAL_OCCUPIED,
        'timeout': ComputePathThroughPoses.Result().TIMEOUT,
        'no_valid_path': ComputePathThroughPoses.Result().NO_VALID_PATH,
        'no_viapoints_given': ComputePathThroughPoses.Result().NO_VIAPOINTS_GIVEN,
    }

    for planner, error_code in compute_path_through_poses.items():
        result = navigator._getPathThroughPosesImpl(initial_pose, goal_poses, planner)
        assert (
            result.error_code == error_code
        ), 'Compute path through pose error does not match'
    # Check compute path to pose cancel
    threading.Thread(target=cancel_task).start()
    result = navigator._getPathThroughPosesImpl(initial_pose, goal_poses, 'cancelled')
    assert (
        navigator.getResult() == TaskResult.CANCELED
    ), 'Compute path through poses cancel failed'

    # Check compute path to pose error codes
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()

    pose.pose.position.x = -1.00
    pose.pose.position.y = -0.50
    pose.pose.orientation.z = 0.0
    pose.pose.orientation.w = 1.0

    pose1 = pose
    pose1.pose.position.x = -0.5

    a_path = Path()
    a_path.poses.append(pose)
    a_path.poses.append(pose1)

    navigator._waitForNodeToActivate('smoother_server')
    smoother = {
        'invalid_smoother': SmoothPath.Result().INVALID_SMOOTHER,
        'unknown': SmoothPath.Result().UNKNOWN,
        'timeout': SmoothPath.Result().TIMEOUT,
        'smoothed_path_in_collision': SmoothPath.Result().SMOOTHED_PATH_IN_COLLISION,
        'failed_to_smooth_path': SmoothPath.Result().FAILED_TO_SMOOTH_PATH,
        'invalid_path': SmoothPath.Result().INVALID_PATH,
    }

    for smoother, error_code in smoother.items():
        result = navigator._smoothPathImpl(a_path, smoother)
        assert result.error_code == error_code, 'Smoother error does not match'

    navigator.lifecycleShutdown()
    rclpy.shutdown()
    exit(0)


if __name__ == '__main__':
    main()
