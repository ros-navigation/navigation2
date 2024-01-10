#! /usr/bin/env python3
# Copyright 2019 Samsung Research America
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
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import ComputePathToPose, FollowWaypoints
from nav2_msgs.srv import ManageLifecycleNodes
from rcl_interfaces.srv import SetParameters

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class WaypointFollowerTest(Node):

    def __init__(self):
        super().__init__(node_name='nav2_waypoint_tester', namespace='')
        self.waypoints = None
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'initialpose', 10
        )
        self.initial_pose_received = False
        self.goal_handle = None
        self.action_result = None

        pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.model_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self.poseCallback, pose_qos
        )
        self.param_cli = self.create_client(
            SetParameters, '/waypoint_follower/set_parameters'
        )

    def setInitialPose(self, pose):
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.pose.pose.position.x = pose[0]
        self.init_pose.pose.pose.position.y = pose[1]
        self.init_pose.header.frame_id = 'map'
        self.publishInitialPose()
        time.sleep(5)

    def poseCallback(self, msg):
        self.info_msg('Received amcl_pose')
        self.initial_pose_received = True

    def setWaypoints(self, waypoints):
        self.waypoints = []
        for wp in waypoints:
            msg = PoseStamped()
            msg.header.frame_id = 'map'
            msg.pose.position.x = wp[0]
            msg.pose.position.y = wp[1]
            msg.pose.orientation.w = 1.0
            self.waypoints.append(msg)

    def run(self, block, cancel):
        # if not self.waypoints:
        #     rclpy.error_msg('Did not set valid waypoints before running test!')
        #     return False

        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.info_msg("'follow_waypoints' action server not available, waiting...")

        action_request = FollowWaypoints.Goal()
        action_request.poses = self.waypoints

        self.info_msg('Sending goal request...')
        send_goal_future = self.action_client.send_goal_async(action_request)
        try:
            rclpy.spin_until_future_complete(self, send_goal_future)
            self.goal_handle = send_goal_future.result()
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')

        if not self.goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        if not block:
            return True

        get_result_future = self.goal_handle.get_result_async()
        if cancel:
            time.sleep(2)
            self.cancel_goal()

        self.info_msg("Waiting for 'follow_waypoints' action to complete")
        try:
            rclpy.spin_until_future_complete(self, get_result_future)
            status = get_result_future.result().status
            result = get_result_future.result().result
            self.action_result = result
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.info_msg(f'Goal failed with status code: {status}')
            return False
        if len(self.action_result.missed_waypoints) > 0:
            self.info_msg(
                'Goal failed to process all waypoints,'
                ' missed {0} wps.'.format(len(self.action_result.missed_waypoints))
            )
            return False

        self.info_msg('Goal succeeded!')
        return True

    def publishInitialPose(self):
        self.initial_pose_pub.publish(self.init_pose)

    def setStopFailureParam(self, value):
        req = SetParameters.Request()
        req.parameters = [
            Parameter('stop_on_failure', Parameter.Type.BOOL, value).to_parameter_msg()
        ]
        future = self.param_cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def shutdown(self):
        self.info_msg('Shutting down')

        self.action_client.destroy()
        self.info_msg('Destroyed follow_waypoints action client')

        transition_service = 'lifecycle_manager_navigation/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(f'{transition_service} service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future)
            future.result()
        except Exception as e:  # noqa: B902
            self.error_msg(f'{transition_service} service call failed {e!r}')

        self.info_msg(f'{transition_service} finished')

        transition_service = 'lifecycle_manager_localization/manage_nodes'
        mgr_client = self.create_client(ManageLifecycleNodes, transition_service)
        while not mgr_client.wait_for_service(timeout_sec=1.0):
            self.info_msg(f'{transition_service} service not available, waiting...')

        req = ManageLifecycleNodes.Request()
        req.command = ManageLifecycleNodes.Request().SHUTDOWN
        future = mgr_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, future)
            future.result()
        except Exception as e:  # noqa: B902
            self.error_msg(f'{transition_service} service call failed {e!r}')

        self.info_msg(f'{transition_service} finished')

    def cancel_goal(self):
        cancel_future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)

    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)


def main(argv=sys.argv[1:]):
    rclpy.init()

    # wait a few seconds to make sure entire stacks are up
    time.sleep(10)

    wps = [[-0.52, -0.54], [0.58, -0.55], [1.78, -0.57]]
    starting_pose = [-2.0, -0.5]

    test = WaypointFollowerTest()
    test.setWaypoints(wps)

    retry_count = 0
    retries = 2
    while not test.initial_pose_received and retry_count <= retries:
        retry_count += 1
        test.info_msg('Setting initial pose')
        test.setInitialPose(starting_pose)
        test.info_msg('Waiting for amcl_pose to be received')
        rclpy.spin_once(test, timeout_sec=1.0)  # wait for poseCallback

    result = test.run(True, False)
    assert result

    # preempt with new point
    test.setWaypoints([starting_pose])
    result = test.run(False, False)
    time.sleep(2)
    test.setWaypoints([wps[1]])
    result = test.run(False, False)

    # cancel
    time.sleep(2)
    test.cancel_goal()

    # set waypoint outside of map
    time.sleep(2)
    test.setWaypoints([[100.0, 100.0]])
    result = test.run(True, False)
    assert not result
    result = not result
    assert (
        test.action_result.missed_waypoints[0].error_code
        == ComputePathToPose.Result().GOAL_OUTSIDE_MAP
    )

    # stop on failure test with bogous waypoint
    test.setStopFailureParam(True)
    bwps = [[-0.52, -0.54], [100.0, 100.0], [0.58, 0.52]]
    test.setWaypoints(bwps)
    result = test.run(True, False)
    assert not result
    result = not result
    mwps = test.action_result.missed_waypoints
    result = (len(mwps) == 1) & (mwps[0] == 1)
    test.setStopFailureParam(False)

    # Zero goal test
    test.setWaypoints([])
    result = test.run(True, False)

    # Cancel test
    test.setWaypoints(wps)
    result = test.run(True, True)
    assert not result
    result = not result

    test.shutdown()
    test.info_msg('Done Shutting Down.')

    if not result:
        test.info_msg('Exiting failed')
        exit(1)
    else:
        test.info_msg('Exiting passed')
        exit(0)


if __name__ == '__main__':
    main()
