#! /usr/bin/env python3
# Copyright 2018 Intel Corporation.
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

import math
import sys
import time

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
import rclpy
from rclpy.node import Node


class NavTester(Node):

    def __init__(self):
        super().__init__('nav2_tester')
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      '/initialpose')
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose')

        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                       '/amcl_pose', self.poseCallback)
        self.initial_pose_received = False

    def setInitialPose(self, pose):
        self.initial_pose = pose
        self.currentPose = pose
        self.publishInitialPose(pose)

    def publishInitialPose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = 'map'
        self.get_logger().info('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)

    def setGoalPose(self, pose):
        self.goal_pose = pose
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose = pose
        self.goal_pub.publish(msg)

    def poseCallback(self, msg):
        self.get_logger().info('Received amcl_pose')
        self.current_pose = msg.pose.pose
        self.initial_pose_received = True

    def reachesGoal(self, timeout, distance):
        goalReached = False
        start_time = time.time()

        while not goalReached:
            rclpy.spin_once(self, timeout_sec=1)
            if self.distanceFromGoal() < distance:
                goalReached = True
                self.get_logger().info('*** GOAL REACHED ***')
                return True
            elif timeout is not None:
                if (time.time() - start_time) > timeout:
                    self.get_logger().error('Robot timed out reaching its goal!')
                    return False

    def distanceFromGoal(self):
        d_x = self.current_pose.position.x - self.goal_pose.position.x
        d_y = self.current_pose.position.y - self.goal_pose.position.y
        distance = math.sqrt(d_x * d_x + d_y * d_y)
        self.get_logger().info('Distance from goal is: ' + str(distance))
        return distance

    def wait_for_node_active(self, node):
        # wait for the bt_navigator to be in active state
        node_service = '/' + node + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            print(node_service + ' service not available, waiting...')
        req = GetState.Request()  # empty request
        state = 'UNKNOWN'
        while (state != 'active'):
            self.get_logger().info('Getting ' + node + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.get_logger().info('Result of get_state: %s' % state)
            else:
                self.get_logger().error('Exception while calling service: %r' % future.exception())
            time.sleep(5)


def test_InitialPose(test_robot, timeout, retries):
    # Set initial pose to the Turtlebot3 starting position -2, 0, 0, facing towards positive X
    initial_pose = Pose()
    initial_pose.position.x = -2.0
    initial_pose.position.y = -0.5
    initial_pose.position.z = 0.01
    initial_pose.orientation.x = 0.0
    initial_pose.orientation.y = 0.0
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 1.0
    test_robot.initial_pose_received = False
    retry_count = 1
    while not test_robot.initial_pose_received and retry_count <= retries:
        retry_count += 1
        test_robot.get_logger().info('Setting initial pose')
        test_robot.setInitialPose(initial_pose)
        test_robot.get_logger().info('Waiting for amcl_pose to be received')
        rclpy.spin_once(test_robot, timeout_sec=timeout)  # wait for poseCallback

    if (test_robot.initial_pose_received):
        test_robot.get_logger().info('test_InitialPose PASSED')
    else:
        test_robot.get_logger().info('test_InitialPose FAILED')
    return test_robot.initial_pose_received


def test_RobotMovesToGoal(test_robot):
    goal_pose = Pose()
    goal_pose.position.x = 0.0
    goal_pose.position.y = 2.0
    goal_pose.position.z = 0.01
    goal_pose.orientation.x = 0.0
    goal_pose.orientation.y = 0.0
    goal_pose.orientation.z = 0.0
    goal_pose.orientation.w = 1.0
    test_robot.get_logger().info('Setting goal pose')
    test_robot.setGoalPose(goal_pose)
    test_robot.get_logger().info('Waiting 60 seconds for robot to reach goal')
    return test_robot.reachesGoal(timeout=60, distance=0.5)


def test_all(test_robot):
    # set transforms to use_sim_time
    result = True
    if (result):
        test_robot.wait_for_node_active('amcl')
        result = test_InitialPose(test_robot, timeout=1, retries=10)
    if (result):
        test_robot.wait_for_node_active('bt_navigator')
    if (result):
        result = test_RobotMovesToGoal(test_robot)

    # Add more tests here if desired
    return result


def main(argv=sys.argv[1:]):
    rclpy.init()

    test_robot = NavTester()
    test_robot.get_logger().info('Starting test_system_node')

    # wait a few seconds to make sure entire stack is up and running
    test_robot.get_logger().info('Waiting for a few seconds for all nodes to initialize')
    time.sleep(5)

    # run tests
    result = test_all(test_robot)
    if (result):
        test_robot.get_logger().info('Test PASSED')
        return
    else:
        test_robot.get_logger().error('Test FAILED')
        exit(1)


if __name__ == '__main__':
    main()
