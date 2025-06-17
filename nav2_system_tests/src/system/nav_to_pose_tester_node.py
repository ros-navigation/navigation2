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
import json
import math
import os
import struct
import sys
import time
from typing import Any, Optional

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
import rclpy
from rclpy.action import ActionClient  # type: ignore[attr-defined]
from rclpy.client import Client
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
import zmq


class NavTester(Node):

    def __init__(self, initial_pose: Pose, goal_pose: Pose, namespace: str = ''):
        super().__init__(node_name='nav2_tester', namespace=namespace)
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)

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
        self.set_initial_pose_timeout = 15
        self.action_client: ActionClient[
            NavigateToPose.Goal,
            NavigateToPose.Result,
            NavigateToPose.Feedback
        ] = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def info_msg(self, msg: str) -> None:
        self.get_logger().info('\033[1;37;44m' + msg + '\033[0m')

    def warn_msg(self, msg: str) -> None:
        self.get_logger().warn('\033[1;37;43m' + msg + '\033[0m')

    def error_msg(self, msg: str) -> None:
        self.get_logger().error('\033[1;37;41m' + msg + '\033[0m')

    def setInitialPose(self) -> None:
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = self.initial_pose
        msg.header.frame_id = 'map'
        self.info_msg('Publishing Initial Pose')
        self.initial_pose_pub.publish(msg)
        self.currentPose = self.initial_pose

    def getStampedPoseMsg(self, pose: Pose) -> PoseStamped:
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose = pose
        return msg

    def publishGoalPose(self, goal_pose: Optional[Pose] = None) -> None:
        self.goal_pose = goal_pose if goal_pose is not None else self.goal_pose
        self.goal_pub.publish(self.getStampedPoseMsg(self.goal_pose))

    def runNavigateAction(self, goal_pose: Optional[Pose] = None) -> bool:
        # Sends a `NavToPose` action request and waits for completion
        self.info_msg("Waiting for 'NavigateToPose' action server")
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.info_msg("'NavigateToPose' action server not available, waiting...")

        if os.getenv('GROOT_MONITORING') == 'True':
            if not self.grootMonitoringGetStatus():
                self.error_msg('Behavior Tree must not be running already!')
                self.error_msg('Are you running multiple goals/bts..?')
                return False

        self.goal_pose = goal_pose if goal_pose is not None else self.goal_pose
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.getStampedPoseMsg(self.goal_pose)

        self.info_msg('Sending goal request...')
        send_goal_future = self.action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        get_result_future = goal_handle.get_result_async()

        future_return = True
        if os.getenv('GROOT_MONITORING') == 'True':
            try:
                if not self.grootMonitoringReloadTree():
                    self.error_msg('Failed GROOT_BT - Reload Tree from ZMQ Server')
                    future_return = False
                if not self.grootMonitoringSetBreakpoint():
                    self.error_msg('Failed GROOT_BT - Set Breakpoint from ZMQ Publisher')
                    future_return = False
            except Exception as e:  # noqa: B902
                self.error_msg(f'Failed GROOT_BT - ZMQ Tests: {e}')
                future_return = False

        self.info_msg("Waiting for 'NavigateToPose' action to complete")
        rclpy.spin_until_future_complete(self, get_result_future)
        status = get_result_future.result().status  # type: ignore[union-attr]
        if status != GoalStatus.STATUS_SUCCEEDED:
            result = get_result_future.result().result  # type: ignore[union-attr]
            self.info_msg(f'Goal failed with status code: {status}'
                          f' error code:{result.error_code}'
                          f' error msg:{result.error_msg}')
            return False

        if not future_return:
            return False

        self.info_msg('Goal succeeded!')
        return True

    def grootMonitoringReloadTree(self) -> bool:
        # ZeroMQ Context
        context = zmq.Context()

        sock = context.socket(zmq.REQ)
        port = 1667  # default server port for groot monitoring
        # # Set a Timeout so we do not spin till infinity
        sock.setsockopt(zmq.RCVTIMEO, 1000)
        # sock.setsockopt(zmq.LINGER, 0)

        sock.connect(f'tcp://localhost:{port}')
        self.info_msg(f'ZMQ Server Port:{port}')

        # this should fail
        try:
            sock.recv()
            self.error_msg('ZMQ Reload Tree Test 1/3 - This should have failed!')
            # Only works when ZMQ server receives a request first
            sock.close()
            return False
        except zmq.error.ZMQError:
            self.info_msg('ZMQ Reload Tree Test 1/3: Check')
        try:
            # request tree from server
            request_header = struct.pack('!BBI', 2, ord('T'), 12345)
            sock.send(request_header)
            # receive tree from server as flat_buffer
            sock.recv_multipart()
            self.info_msg('ZMQ Reload Tree Test 2/3: Check')
        except zmq.error.Again:
            self.info_msg('ZMQ Reload Tree Test 2/3 - Failed to load tree')
            sock.close()
            return False

        # this should fail
        try:
            sock.recv()
            self.error_msg('ZMQ Reload Tree Test 3/3 - This should have failed!')
            # Tree should only be loadable ONCE after ZMQ server received a request
            sock.close()
            return False
        except zmq.error.ZMQError:
            self.info_msg('ZMQ Reload Tree Test 3/3: Check')

        return True

    def grootMonitoringSetBreakpoint(self) -> bool:
        # ZeroMQ Context
        context = zmq.Context()
        # Define the socket using the 'Context'
        sock = context.socket(zmq.REQ)
        # Set a Timeout so we do not spin till infinity
        sock.setsockopt(zmq.RCVTIMEO, 2000)
        # sock.setsockopt(zmq.LINGER, 0)

        port = 1667  # default publishing port for groot monitoring
        sock.connect(f'tcp://127.0.0.1:{port}')
        self.info_msg(f'ZMQ Publisher Port:{port}')

        # Create header for the request
        request_header = struct.pack('!BBI', 2, ord('I'), 12345)  # HOOK_INSERT
        # Create JSON for the hook
        hook_data = {
            'enabled': True,
            'uid': 9,  # Node ID
            'mode': 0,  # 0 = BREAKPOINT, 1 = REPLACE
            'once': False,
            'desired_status': 'SUCCESS',  # Desired status
            'position': 0,  # 0 = PRE, 1 = POST
        }
        hook_json = json.dumps(hook_data)

        # Send the request
        try:
            sock.send_multipart([request_header, hook_json.encode('utf-8')])
            reply = sock.recv_multipart()
            if len(reply[0]) < 2:
                self.error_msg('ZMQ - Incomplete reply received')
                sock.close()
                return False
        except Exception as e:
            self.error_msg(f'ZMQ - Error during request: {e}')
            sock.close()
            return False
        self.info_msg('ZMQ - HOOK_INSERT request sent')
        return True

    def grootMonitoringGetStatus(self) -> bool:
        # ZeroMQ Context
        context = zmq.Context()

        sock = context.socket(zmq.REQ)
        port = 1667  # default server port for groot monitoring
        # # Set a Timeout so we do not spin till infinity
        sock.setsockopt(zmq.RCVTIMEO, 1000)
        # sock.setsockopt(zmq.LINGER, 0)

        sock.connect(f'tcp://localhost:{port}')
        self.info_msg(f'ZMQ Server Port:{port}')

        for request in range(3):
            try:
                # request tree from server
                request_header = struct.pack('!BBI', 2, ord('S'), 12345)
                sock.send(request_header)
                # receive tree from server as flat_buffer
                reply = sock.recv_multipart()
                if len(reply[0]) < 6:
                    self.error_msg('ZMQ - Incomplete reply received')
                    sock.close()
                    return False
                # Decoding payload
                payload = reply[1]
                node_states = []
                offset = 0
                while offset < len(payload):
                    node_uid, node_status = struct.unpack_from('!HB', payload, offset)
                    offset += 3  # 2 bytes for UID, 1 byte for status
                    node_states.append((node_uid, node_status))
                # Get the status of the first node
                node_uid, node_status = node_states[0]
                if node_status != 0:
                    self.error_msg('ZMQ - BT Not running')
                    return False
            except zmq.error.Again:
                self.error_msg('ZMQ - Did not receive any status')
                sock.close()
                return False
        self.info_msg('ZMQ - Did receive status')
        return True

    def poseCallback(self, msg: PoseWithCovarianceStamped) -> None:
        self.info_msg('Received amcl_pose')
        self.current_pose = msg.pose.pose
        self.initial_pose_received = True

    def reachesGoal(self, timeout: float, distance: float) -> bool:
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

        self.info_msg('Robot reached its goal!')
        return True

    def distanceFromGoal(self) -> float:
        d_x = self.current_pose.position.x - self.goal_pose.position.x
        d_y = self.current_pose.position.y - self.goal_pose.position.y
        distance = math.sqrt(d_x * d_x + d_y * d_y)
        self.info_msg(f'Distance from goal is: {distance}')
        return distance

    def wait_for_node_active(self, node_name: str) -> None:
        # Waits for the node within the tester namespace to become active
        self.info_msg(f'Waiting for {node_name} to become active')
        node_service = f'{node_name}/get_state'
        state_client: Client[GetState.Request, GetState.Response] = \
            self.create_client(GetState, node_service)
        while not state_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(f'{node_service} service not available, waiting...')
        req = GetState.Request()  # empty request
        state = 'UNKNOWN'
        while state != 'active':
            self.info_msg(f'Getting {node_name} state...')
            future = state_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            if future.result() is not None:
                state = future.result().current_state.label  # type: ignore[union-attr]
                self.info_msg(f'Result of get_state: {state}')
            else:
                self.error_msg(
                    f'Exception while calling service: {future.exception()!r}'
                )
            time.sleep(5)

    def shutdown(self) -> None:
        self.info_msg('Shutting down')
        self.action_client.destroy()

        transition_service = 'lifecycle_manager_navigation/manage_nodes'
        mgr_client: Client[ManageLifecycleNodes.Request, ManageLifecycleNodes.Response] = \
            self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(f'{transition_service} service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
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
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            self.info_msg('Shutting down localization lifecycle manager...')
            rclpy.spin_until_future_complete(self, future)
            future.result()
            self.info_msg('Shutting down localization lifecycle manager complete')
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')

    def wait_for_initial_pose(self) -> bool:
        self.initial_pose_received = False
        # If the initial pose is not received within 100 seconds, return False
        # this is because when setting a wrong initial pose, amcl_pose is not received
        # and the test will hang indefinitely
        start_time = time.time()
        duration = 0.0
        while not self.initial_pose_received:
            self.info_msg('Setting initial pose')
            self.setInitialPose()
            self.info_msg('Waiting for amcl_pose to be received')
            duration = time.time() - start_time
            if duration > self.set_initial_pose_timeout:
                self.error_msg('Timeout waiting for initial pose to be set')
                return False
            rclpy.spin_once(self, timeout_sec=1)
        return True


def test_RobotMovesToGoal(robot_tester: NavTester) -> bool:
    robot_tester.info_msg('Setting goal pose')
    robot_tester.publishGoalPose()
    robot_tester.info_msg('Waiting 60 seconds for robot to reach goal')
    return robot_tester.reachesGoal(timeout=60, distance=0.5)


def run_all_tests(robot_tester: NavTester) -> bool:
    # set transforms to use_sim_time
    result = True
    if result:
        robot_tester.wait_for_node_active('amcl')
        result = robot_tester.wait_for_initial_pose()
    if result:
        robot_tester.wait_for_node_active('bt_navigator')
        result = robot_tester.runNavigateAction()

    if result:
        result = test_RobotMovesToGoal(robot_tester)

    # Add more tests here if desired

    if result:
        robot_tester.info_msg('Test PASSED')
    else:
        robot_tester.error_msg('Test FAILED')

    return result


def fwd_pose(x: float = 0.0, y: float = 0.0, z: float = 0.01) -> Pose:
    initial_pose = Pose()
    initial_pose.position.x = x
    initial_pose.position.y = y
    initial_pose.position.z = z
    initial_pose.orientation.x = 0.0
    initial_pose.orientation.y = 0.0
    initial_pose.orientation.z = 0.0
    initial_pose.orientation.w = 1.0
    return initial_pose


def get_testers(args: argparse.Namespace) -> list[NavTester]:
    testers = []

    if args.robot:
        # Requested tester for one robot
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

    # Requested tester for multiple robots
    for robot in args.robots:
        namespace, init_x, init_y, final_x, final_y = robot
        tester = NavTester(
            namespace=namespace,
            initial_pose=fwd_pose(float(init_x), float(init_y)),
            goal_pose=fwd_pose(float(final_x), float(final_y)),
        )
        tester.info_msg(
            'Starting tester for '
            + namespace
            + ' going from '
            + init_x
            + ', '
            + init_y
            + ' to '
            + final_x
            + ', '
            + final_y
        )
        testers.append(tester)
    return testers


def check_args(expect_failure: str) -> Any:
    # Check if --expect_failure is True or False
    if expect_failure != 'True' and expect_failure != 'False':
        print(
            '\033[1;37;41m' + ' -e flag must be set to True or False only. ' + '\033[0m'
        )
        exit(1)
    else:
        return eval(expect_failure)


def main(argv: list[str] = sys.argv[1:]):  # type: ignore[no-untyped-def]
    # The robot(s) positions from the input arguments
    parser = argparse.ArgumentParser(description='System-level navigation tester node')
    parser.add_argument('-e', '--expect_failure')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument(
        '-r',
        '--robot',
        action='append',
        nargs=4,
        metavar=('init_x', 'init_y', 'final_x', 'final_y'),
        help='The robot starting and final positions.',
    )
    group.add_argument(
        '-rs',
        '--robots',
        action='append',
        nargs=5,
        metavar=('name', 'init_x', 'init_y', 'final_x', 'final_y'),
        help="The robot's namespace and starting and final positions."
        + 'Repeating the argument for multiple robots is supported.',
    )

    args, unknown = parser.parse_known_args()

    expect_failure = check_args(args.expect_failure)

    rclpy.init()

    # Create testers for each robot
    testers = get_testers(args)

    # wait a few seconds to make sure entire stacks are up
    time.sleep(10)

    for tester in testers:
        passed = run_all_tests(tester)
        if passed != expect_failure:
            break

    for tester in testers:
        # stop and shutdown the nav stack to exit cleanly
        tester.shutdown()

    testers[0].info_msg('Done Shutting Down.')

    if passed != expect_failure:
        testers[0].info_msg('Exiting failed')
        exit(1)
    else:
        testers[0].info_msg('Exiting passed')
        exit(0)


if __name__ == '__main__':
    main()
