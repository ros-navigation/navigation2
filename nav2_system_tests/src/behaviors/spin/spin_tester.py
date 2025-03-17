#! /usr/bin/env python3
# Copyright 2024 Open Navigation LLC
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
from geometry_msgs.msg import Point32, PolygonStamped
from nav2_msgs.action import Spin
from nav2_msgs.msg import Costmap
from nav2_msgs.srv import ManageLifecycleNodes

import rclpy

from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy
from rclpy.qos import QoSProfile


class SpinTest(Node):

    def __init__(self):
        super().__init__(node_name='spin_tester', namespace='')
        self.costmap_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.action_client = ActionClient(self, Spin, 'spin')
        self.costmap_pub = self.create_publisher(
            Costmap, 'local_costmap/costmap_raw', self.costmap_qos)
        self.footprint_pub = self.create_publisher(
            PolygonStamped, 'local_costmap/published_footprint', 10)
        self.goal_handle = None
        self.action_result = None

    def sendCommand(self, command):
        self.info_msg('Sending goal request...')
        self.goal_future = self.action_client.send_goal_async(command)
        try:
            rclpy.spin_until_future_complete(self, self.goal_future)
            self.goal_handle = self.goal_future.result()
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')

        if not self.goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        self.result_future = self.goal_handle.get_result_async()

        self.info_msg("Waiting for 'spin' action to complete")
        try:
            rclpy.spin_until_future_complete(self, self.result_future)
            status = self.result_future.result().status
            result = self.result_future.result().result
            self.action_result = result
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.info_msg(f'Goal failed with status code: {status}')
            return False
        if self.action_result.error_code == 0:
            self.info_msg('Spin was successful!')
            return True
        self.info_msg('Spin failed to meet target!')
        return False

    def sendAndPreemptWithInvertedCommand(self, command):
        # Send initial goal
        self.info_msg('Sending goal request...')
        self.goal_future = self.action_client.send_goal_async(command)
        try:
            rclpy.spin_until_future_complete(self, self.goal_future)
            self.goal_handle = self.goal_future.result()
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')

        if not self.goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        self.result_future = self.goal_handle.get_result_async()

        # Now preempt it
        time.sleep(0.5)
        self.info_msg('Sending preemption request...')
        command.target_yaw = -command.target_yaw
        self.goal_future = self.action_client.send_goal_async(command)
        try:
            rclpy.spin_until_future_complete(self, self.goal_future)
            self.goal_handle = self.goal_future.result()
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')

        if not self.goal_handle.accepted:
            self.error_msg('Preemption rejected')
            return False

        self.info_msg('Preemption accepted')
        self.result_future = self.goal_handle.get_result_async()

        # Wait for new goal to complete
        self.info_msg("Waiting for 'spin' action Preemption to complete")
        try:
            rclpy.spin_until_future_complete(self, self.result_future)
            status = self.result_future.result().status
            result = self.result_future.result().result
            self.action_result = result
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.info_msg(f'Goal failed with status code: {status}')
            return False
        if self.action_result.error_code == 0:
            self.info_msg('Spin was successful!')
            return True
        self.info_msg('Spin failed to meet target!')
        return False

    def sendAndCancelCommand(self, command):
        self.info_msg('Sending goal request...')
        self.goal_future = self.action_client.send_goal_async(command)
        try:
            rclpy.spin_until_future_complete(self, self.goal_future)
            self.goal_handle = self.goal_future.result()
        except Exception as e:  # noqa: B902
            self.error_msg(f'Service call failed {e!r}')

        if not self.goal_handle.accepted:
            self.error_msg('Goal rejected')
            return False

        self.info_msg('Goal accepted')
        self.result_future = self.goal_handle.get_result_async()

        # Now cancel it
        time.sleep(0.5)
        cancel_future = self.goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future)
        rclpy.spin_until_future_complete(self, self.result_future)
        status = self.result_future.result().status
        if status != GoalStatus.STATUS_CANCELED:
            self.info_msg(f'Goal failed with status code: {status}')
            return False
        else:
            self.info_msg('Goal was canceled successfully')
            return True

    def sendFreeCostmap(self):
        costmap_msg = Costmap()
        costmap_msg.header.frame_id = 'odom'
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.metadata.resolution = 0.05
        costmap_msg.metadata.size_x = 100
        costmap_msg.metadata.size_y = 100
        costmap_msg.metadata.origin.position.x = -2.5
        costmap_msg.metadata.origin.position.y = -2.5
        costmap_msg.data = [0] * (costmap_msg.metadata.size_x * costmap_msg.metadata.size_y)
        self.costmap_pub.publish(costmap_msg)

        footprint_msg = PolygonStamped()
        footprint_msg.header.frame_id = 'odom'
        footprint_msg.header.stamp = self.get_clock().now().to_msg()
        footprint_msg.polygon.points = [
            Point32(x=0.1, y=0.1, z=0.0),
            Point32(x=0.1, y=-0.1, z=0.0),
            Point32(x=-0.1, y=-0.1, z=0.0),
            Point32(x=-0.1, y=0.1, z=0.0)
        ]
        self.footprint_pub.publish(footprint_msg)

    def sendOccupiedCostmap(self):
        costmap_msg = Costmap()
        costmap_msg.header.frame_id = 'odom'
        costmap_msg.header.stamp = self.get_clock().now().to_msg()
        costmap_msg.metadata.resolution = 0.05
        costmap_msg.metadata.size_x = 100
        costmap_msg.metadata.size_y = 100
        costmap_msg.metadata.origin.position.x = -2.5
        costmap_msg.metadata.origin.position.y = -2.5
        costmap_msg.data = [254] * (costmap_msg.metadata.size_x * costmap_msg.metadata.size_y)
        self.costmap_pub.publish(costmap_msg)

        footprint_msg = PolygonStamped()
        footprint_msg.header.frame_id = 'odom'
        footprint_msg.header.stamp = self.get_clock().now().to_msg()
        footprint_msg.polygon.points = [
            Point32(x=0.1, y=0.1, z=0.0),
            Point32(x=0.1, y=-0.1, z=0.0),
            Point32(x=-0.1, y=-0.1, z=0.0),
            Point32(x=-0.1, y=0.1, z=0.0)
        ]
        self.footprint_pub.publish(footprint_msg)

    def run(self):
        while not self.action_client.wait_for_server(timeout_sec=1.0):
            self.info_msg("'spin' action server not available, waiting...")

        # Test A: Send without valid costmap
        action_request = Spin.Goal()
        action_request.target_yaw = 0.349066  # 20 deg
        action_request.time_allowance = Duration(seconds=5).to_msg()
        cmd1 = self.sendCommand(action_request)

        if cmd1:
            self.error_msg('Test A failed: Passed while costmap was not available!')
            return not cmd1
        else:
            self.info_msg('Test A passed')

        # Test B: Send with valid costmap and spin a couple of times
        self.sendFreeCostmap()
        time.sleep(1)
        cmd1 = self.sendCommand(action_request)
        action_request.target_yaw = -3.49066  # -200 deg
        cmd2 = self.sendCommand(action_request)

        if not cmd1 or not cmd2:
            self.error_msg('Test B failed: Failed to spin with valid costmap!')
            return not cmd1 or not cmd2

        action_request.target_yaw = 0.349066  # 20 deg
        cmd_preempt = self.sendAndPreemptWithInvertedCommand(action_request)
        if not cmd_preempt:
            self.error_msg('Test B failed: Failed to preempt and invert command!')
            return not cmd_preempt

        cmd_cancel = self.sendAndCancelCommand(action_request)
        if not cmd_cancel:
            self.error_msg('Test B failed: Failed to cancel command!')
            return not cmd_cancel
        else:
            self.info_msg('Test B passed')

        # Test C: Send with impossible command in time allowance
        action_request.time_allowance = Duration(seconds=0.1).to_msg()
        cmd3 = self.sendCommand(action_request)
        if cmd3:
            self.error_msg('Test C failed: Passed while impoossible timing requested!')
            return not cmd3
        else:
            self.info_msg('Test C passed')

        # Test D: Send with lethal costmap and spin
        action_request.time_allowance = Duration(seconds=5).to_msg()
        self.sendOccupiedCostmap()
        time.sleep(1)
        cmd4 = self.sendCommand(action_request)
        if cmd4:
            self.error_msg('Test D failed: Passed while costmap was not lethal!')
            return not cmd4
        else:
            self.info_msg('Test D passed')
        return True

    def shutdown(self):
        self.info_msg('Shutting down')

        self.action_client.destroy()
        self.info_msg('Destroyed backup action client')

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

    def info_msg(self, msg: str):
        self.get_logger().info(msg)

    def warn_msg(self, msg: str):
        self.get_logger().warn(msg)

    def error_msg(self, msg: str):
        self.get_logger().error(msg)


def main(argv=sys.argv[1:]):
    rclpy.init()
    time.sleep(10)
    test = SpinTest()
    result = test.run()
    test.shutdown()

    if not result:
        test.info_msg('Exiting failed')
        exit(1)
    else:
        test.info_msg('Exiting passed')
        exit(0)


if __name__ == '__main__':
    main()
