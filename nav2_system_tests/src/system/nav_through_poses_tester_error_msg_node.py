#! /usr/bin/env python3
# Copyright 2018 Intel Corporation.
# Copyright 2020 Florian Gramss
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
import sys
import time

from typing import Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.srv import SetParameters

import rclpy

from rclpy.action.client import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile

from std_msgs.msg import String


class NavTester(Node):

    def __init__(self, initial_pose: Pose, goal_pose: Pose, namespace: str = ''):
        super().__init__(node_name='nav2_tester', namespace=namespace)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )

        checker_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.goal_checker_selector_pub = self.create_publisher(
            String, 'goal_checker_selector', checker_qos)

        self.progress_checker_selector_pub = self.create_publisher(
            String, 'progress_checker_selector', checker_qos)

        pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.model_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.poseCallback, pose_qos
        )
        self.initial_pose_received = False
        self.initial_pose = initial_pose
        self.goal_pose = goal_pose
        self.action_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses'
        )

        self.controller_param_cli = self.create_client(
            SetParameters, '/controller_server/set_parameters'
        )

    def info_msg(self, msg: str):
        self.get_logger().info('\033[1;37;44m' + msg + '\033[0m')

    def warn_msg(self, msg: str):
        self.get_logger().warning('\033[1;37;43m' + msg + '\033[0m')

    def error_msg(self, msg: str):
        self.get_logger().error('\033[1;37;41m' + msg + '\033[0m')

    def setInitialPose(self):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        self.info_msg('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        self.currentPose = self.initial_pose

    def setGoalChecker(self, name):
        msg = String()
        msg.data = name
        self.goal_checker_selector_pub.publish(msg)

    def setProgressChecker(self, name):
        msg = String()
        msg.data = name
        self.progress_checker_selector_pub.publish(msg)

    def setControllerParam(self, name, parameter_type, value):
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name, parameter_type, value).to_parameter_msg()
        ]
        future = self.controller_param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def getStampedPoseMsg(self, pose: Pose):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose = pose
        return msg

    def runNavigateAction(self,
                          goal_pose: Optional[Pose] = None,
                          behavior_tree: Optional[str] = None,
                          expected_error_code: Optional[int] = None,
                          expected_error_msg: Optional[str] = None):
        # Sends a `NavToPose` action request and waits for completion
        self.info_msg("Waiting for 'NavigateThroughPoses' action server")
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.info_msg(
                "'NavigateThroughPoses' action server not available, waiting..."
            )

        self.goal_pose = goal_pose if goal_pose is not None else self.goal_pose
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses.header.frame_id = 'map'
        goal_msg.poses.header.stamp = self.get_clock().now().to_msg()
        goal_msg.poses.poses = [
            self.getStampedPoseMsg(self.goal_pose),
            self.getStampedPoseMsg(self.goal_pose),
        ]
        goal_msg.behavior_tree = behavior_tree

        self.info_msg('Sending goal request...')
        send_goal_future = self.action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        get_result_future = goal_handle.get_result_async()

        self.info_msg("Waiting for 'NavigateToPose' action to complete")
        rclpy.spin_until_future_complete(self, get_result_future)
        status = get_result_future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            result = get_result_future.result().result
            if (result.error_code == expected_error_code and
               result.error_msg == expected_error_msg):
                self.info_msg(f'Goal failed as expected with status code: {status}'
                              f' error code:{result.error_code}'
                              f' error msg:{result.error_msg}')
                return True
            else:
                self.error_msg(f'Goal failed unexpectedly with status code: {status}'
                               f' Expected error_code:{expected_error_code},'
                               f' Got error_code:{result.error_code},'
                               f' Expected error_msg:{expected_error_msg},'
                               f' Got error_msg:{result.error_msg}')
                return False

        self.info_msg('Goal succeeded!')
        return True

    def poseCallback(self, msg):
        self.info_msg('Received amcl_pose')
        self.current_pose = msg.pose.pose
        self.initial_pose_received = True

    def wait_for_node_active(self, node_name: str):
        # Waits for the node within the tester namespace to become active
        self.info_msg(f'Waiting for {node_name} to become active')
        node_service = f'{node_name}/get_state'
        state_client = self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(f'{node_service} service not available, waiting...')
        req = GetState.Request()  # empty request
        state = 'UNKNOWN'
        while state != 'active':
            self.info_msg(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label
                self.info_msg(f'Result of get_state: {state}')
            else:
                self.error_msg(
                    f'Exception while calling service: {future.exception()!r}'
                )
            time.sleep(5)

    def shutdown(self):
        self.info_msg('Shutting down')
        self.action_client.destroy()

        transition_service = 'lifecycle_manager_navigation/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(f'{transition_service} service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            self.info_msg('Shutting down navigation lifecycle manager...')
            rclpy.spin_until_future_complete(self, future)
            future.result()
            self.info_msg('Shutting down navigation lifecycle manager complete.')
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')
        transition_service = 'lifecycle_manager_localization/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(f'{transition_service} service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request.SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            self.info_msg('Shutting down localization lifecycle manager...')
            rclpy.spin_until_future_complete(self, future)
            future.result()
            self.info_msg('Shutting down localization lifecycle manager complete')
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')

    def wait_for_initial_pose(self):
        self.initial_pose_received = False
        while not self.initial_pose_received:
            self.info_msg('Setting initial pose')
            self.setInitialPose()
            self.info_msg('Waiting for amcl_pose to be received')
            rclpy.spin_once(self, timeout_sec=1)


def run_all_tests(robot_tester):
    pose_out_of_bounds = Pose()
    pose_out_of_bounds.position.x = 2000.0
    pose_out_of_bounds.position.y = 4000.0
    pose_out_of_bounds.position.z = 0.0
    pose_out_of_bounds.orientation.x = 0.0
    pose_out_of_bounds.orientation.y = 0.0
    pose_out_of_bounds.orientation.z = 0.0
    pose_out_of_bounds.orientation.w = 1.0

    reasonable_pose = Pose()
    reasonable_pose.position.x = -1.0
    reasonable_pose.position.y = 0.0
    reasonable_pose.position.z = 0.0
    reasonable_pose.orientation.x = 0.0
    reasonable_pose.orientation.y = 0.0
    reasonable_pose.orientation.z = 0.0
    reasonable_pose.orientation.w = 1.0

    robot_tester.wait_for_node_active('amcl')
    robot_tester.wait_for_initial_pose()
    robot_tester.wait_for_node_active('bt_navigator')
    robot_tester.setGoalChecker('general_goal_checker')
    robot_tester.setProgressChecker('progress_checker')

    result = True
    if result:
        robot_tester.info_msg('Test non existing behavior_tree xml file')
        result = robot_tester.runNavigateAction(
            goal_pose=pose_out_of_bounds,
            behavior_tree='behavior_tree_that_does_not_exist.xml',
            expected_error_code=NavigateThroughPoses.Result.FAILED_TO_LOAD_BEHAVIOR_TREE,
            expected_error_msg=('Error loading XML file: behavior_tree_that_does_not_exist.xml. '
                                'Navigation canceled.'))

    if result:
        robot_tester.info_msg('Test goal out of bounds')
        result = robot_tester.runNavigateAction(
            goal_pose=pose_out_of_bounds,
            behavior_tree='',
            expected_error_code=304,
            expected_error_msg=('GridBasedplugin failed to plan from '
                                '(-2.00, -0.50) to (2000.00, 4000.00): '
                                '"Goal Coordinates of(2000.000000, 4000.000000) '
                                'was outside bounds"'))

    if result:
        robot_tester.info_msg('Test for unknown goal checker')
        robot_tester.setGoalChecker('junk_goal_checker')
        result = robot_tester.runNavigateAction(
            goal_pose=reasonable_pose,
            behavior_tree='',
            expected_error_code=100,
            expected_error_msg=('Failed to find goal checker name: junk_goal_checker'))
        robot_tester.setGoalChecker('general_goal_checker')

    if result:
        robot_tester.info_msg('Test for unknown progress checker')
        robot_tester.setProgressChecker('junk_progress_checker')
        result = robot_tester.runNavigateAction(
            goal_pose=reasonable_pose,
            behavior_tree='',
            expected_error_code=100,
            expected_error_msg=('Failed to find progress checker name: junk_progress_checker'))
        robot_tester.setProgressChecker('progress_checker')

    if result:
        robot_tester.info_msg('Test for impossible to achieve progress parameters')
        robot_tester.setControllerParam(
            'progress_checker.movement_time_allowance',
            Parameter.Type.DOUBLE,
            0.1)
        robot_tester.setControllerParam(
            'progress_checker.required_movement_radius',
            Parameter.Type.DOUBLE,
            10.0)
        # Limit controller to generate very slow velocities
        # Note assumes nav2_dwb_controller dwb_core::DWBLocalPlanner
        robot_tester.setControllerParam(
            'FollowPath.max_vel_x',
            Parameter.Type.DOUBLE,
            0.0001)
        result = robot_tester.runNavigateAction(
            goal_pose=reasonable_pose,
            behavior_tree='',
            expected_error_code=105,
            expected_error_msg=('Failed to make progress'))

    # Add more tests here if desired
    if result:
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


def get_testers(args):
    testers = []

    init_x, init_y, final_x, final_y = args.robot[0]
    tester = NavTester(
        initial_pose=fwd_pose(float(init_x), float(init_y)),
        goal_pose=fwd_pose(float(final_x), float(final_y)),
    )
    tester.info_msg(
        'Starting tester, robot going from '
        + init_x
        + ', '
        + init_y
        + ' to '
        + final_x
        + ', '
        + final_y
        + '.'
    )
    testers.append(tester)
    return testers


def main(argv=sys.argv[1:]):
    # The robot(s) positions from the input arguments
    parser = argparse.ArgumentParser(description='System-level navigation tester node')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        '-r',
        '--robot',
        action='append',
        nargs=4,
        metavar=('init_x', 'init_y', 'final_x', 'final_y'),
        help='The robot starting and final positions.',
    )

    args, _ = parser.parse_known_args()

    rclpy.init()

    # Create testers for each robot
    testers = get_testers(args)

    # wait a few seconds to make sure entire stacks are up
    time.sleep(10)

    passed = False
    for tester in testers:
        passed = run_all_tests(tester)
        if not passed:
            break

    for tester in testers:
        # stop and shutdown the nav stack to exit cleanly
        tester.shutdown()

    testers[0].info_msg('Done Shutting Down.')

    if not passed:
        testers[0].info_msg('Exiting failed')
        exit(1)
    else:
        testers[0].info_msg('Exiting passed')
        exit(0)


if __name__ == '__main__':
    main()
