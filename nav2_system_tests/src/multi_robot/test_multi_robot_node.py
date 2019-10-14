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

import argparse
import math
import sys
import time
from typing import Optional

from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState

import rclpy

from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class NavTester(Node):

    def __init__(
        self,
        initial_pose: Pose,
        goal_pose: Pose,
        namespace: str = ''  # TODO(orduno) try '/'
    ):
        super().__init__(node_name='nav2_tester', namespace=namespace)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped,
                                                      'initialpose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

        pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
          history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
          depth=1)

        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped,
                                                       'amcl_pose', self.poseCallback, pose_qos)
        self.initial_pose_received = False
        # self.namespace = '/' if namespace is None else namespace
        self.initial_pose = initial_pose
        self.goal_pose = goal_pose

    def info_msg(self, msg: str):
        self.get_logger().info('\033[1;37;44m' + msg + '\033[0m')

    def info_warn(self, msg: str):
        self.get_logger().warn('\033[1;37;43m' + msg + '\033[0m')

    def info_error(self, msg: str):
        self.get_logger().error('\033[1;37;41m' + msg + '\033[0m')

    def setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        self.info_msg('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        self.currentPose = self.initial_pose

    def setGoalPose(self, pose: Optional[Pose] = None):
        self.goal_pose = pose if pose is not None else self.goal_pose
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose = self.goal_pose
        self.goal_pub.publish(msg)

    def poseCallback(self, msg):
        self.info_msg('Received amcl_pose')
        self.current_pose = msg.pose.pose
        self.initial_pose_received = True

    def reachesGoal(self, timeout, distance):
        goalReached = False
        start_time = time.time()

        while not goalReached:
            rclpy.spin_once(self, timeout_sec=1)
            if self.distanceFromGoal() < distance:
                goalReached = True
                self.info_msg('*** GOAL REACHED ***')
                return True
            elif timeout is not None:
                if (time.time() - start_time) > timeout:
                    self.error_msg('Robot timed out reaching its goal!')
                    return False

    def distanceFromGoal(self):
        d_x = self.current_pose.position.x - self.goal_pose.position.x
        d_y = self.current_pose.position.y - self.goal_pose.position.y
        distance = math.sqrt(d_x * d_x + d_y * d_y)
        self.info_msg('Distance from goal is: ' + str(distance))
        return distance

    def wait_for_node_active(self, node_name: str):
        # Waits for the node within the tester namespace to become active
        self.info_msg('Waiting for ' + node_name + ' to become active')
        node_service = node_name + '/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(node_service + ' service not available, waiting...')
        req = GetState.Request()  # empty request
        state = 'UNKNOWN'
        while (state != 'active'):
            self.info_msg('Getting ' + node_name + ' state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.info_msg('Result of get_state: %s' % state)
            else:
                self.error_msg('Exception while calling service: %r' % future.exception())
            time.sleep(5)


def test_InitialPose(robot_tester, timeout, retries):
    robot_tester.initial_pose_received = False
    retry_count = 1
    while not robot_tester.initial_pose_received and retry_count <= retries:
        retry_count += 1
        robot_tester.info_msg('Setting initial pose')
        robot_tester.setInitialPose()
        robot_tester.info_msg('Waiting for amcl_pose to be received')
        rclpy.spin_once(robot_tester, timeout_sec=timeout)  # wait for poseCallback

    if (robot_tester.initial_pose_received):
        robot_tester.info_msg('test_InitialPose PASSED')
    else:
        robot_tester.info_msg('test_InitialPose FAILED')
    return robot_tester.initial_pose_received


def test_RobotMovesToGoal(robot_tester):
    robot_tester.info_msg('Setting goal pose')
    robot_tester.setGoalPose()
    robot_tester.info_msg('Waiting 60 seconds for robot to reach goal')
    return robot_tester.reachesGoal(timeout=60, distance=0.5)


def run_all_tests(robot_tester):
    # set transforms to use_sim_time
    result = True
    if (result):
        robot_tester.wait_for_node_active('amcl')
        result = test_InitialPose(robot_tester, timeout=1, retries=10)
    if (result):
        robot_tester.wait_for_node_active('bt_navigator')
    if (result):
        result = test_RobotMovesToGoal(robot_tester)

    # Add more tests here if desired

    if (result):
        robot_tester.info_msg('Test PASSED')
    else:
        robot_tester.error_msg('Test FAILED')

    return result


def fwd_pose(x=0.0, y=0.0, z=0.01):
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z
    initial_pose.orientation.x = 0.0
    initial_pose.orientation.y = 0.0
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 1.0
    return initial_pose


def main(argv=sys.argv[1:]):
    # TODO(orduno) Run python linter

    # Get input arguments
    parser = argparse.ArgumentParser(description='System-level navigation tester node')
    parser.add_argument('-r', '--robots', action='append', nargs=5,
                        metavar=('name', 'init_x', 'init_y', 'final_x', 'final_y'),
                        help="The robot's namespace and starting and final positions. " +
                             'Repeating the argument for multiple robots is supported.')

    args, unknown = parser.parse_known_args()

    rclpy.init()

    # Create testers for each robot
    testers = []
    for robot in args.robots:
        namespace, init_x, init_y, final_x, final_y = robot
        tester = NavTester(
            namespace=namespace,
            initial_pose=fwd_pose(float(init_x), float(init_y)),
            goal_pose=fwd_pose(float(final_x), float(final_y)))
        tester.info_msg(
            'Starting tester for ' + namespace +
            ' going from ' + init_x + ', ' + init_y +
            ' to ' + final_x + ', ' + final_y)
        testers.append(tester)

    # wait a few seconds to make sure entire stack is up
    time.sleep(20)

    # run tests on each robot
    for tester in testers:
        passed = run_all_tests(tester)
        if not passed:
            exit(1)


if __name__ == '__main__':
    main()
