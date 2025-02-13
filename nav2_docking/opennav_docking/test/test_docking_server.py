# Copyright (c) 2024 Open Navigation LLC
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

from math import acos, cos, sin
import time
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import TransformStamped, Twist, TwistStamped
from launch import LaunchDescription
# from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.markers
import launch_testing.util
from nav2_msgs.action import DockRobot, NavigateToPose, UndockRobot
import pytest
import rclpy
from rclpy.action.client import ActionClient
from rclpy.action.server import ActionServer
from sensor_msgs.msg import BatteryState
from tf2_ros import TransformBroadcaster


# This test can be run standalone with:
# python3 -u -m pytest test_docking_server.py -s

# If python3-flaky is installed, you can run the test multiple times to
# try to identify flaky ness.
# python3 -u -m pytest --force-flaky --min-passes 3 --max-runs 5 -s -v test_docking_server.py

@pytest.mark.rostest
# @pytest.mark.flaky
# @pytest.mark.flaky(max_runs=5, min_passes=3)
def generate_test_description():

    return LaunchDescription([
        # SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        Node(
            package='opennav_docking',
            executable='opennav_docking',
            name='docking_server',
            parameters=[{'wait_charge_timeout': 1.0,
                         'controller': {
                             'use_collision_detection': False,
                             'transform_tolerance': 0.5,
                         },
                         'dock_plugins': ['test_dock_plugin'],
                         'test_dock_plugin': {
                             'plugin': 'opennav_docking::SimpleChargingDock',
                             'use_battery_status': True},
                         'docks': ['test_dock'],
                         'test_dock': {
                             'type': 'test_dock_plugin',
                             'frame': 'odom',
                             'pose': [10.0, 0.0, 0.0]
                         }}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['docking_server']}]
        ),
        launch_testing.actions.ReadyToTest(),
    ])


class TestDockingServer(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        # Create a ROS node for tests
        # Latest odom -> base_link
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        # Track charge state
        self.is_charging = False
        # Latest command velocity
        self.command = Twist()
        self.node = rclpy.create_node('test_docking_server')

    def tearDown(self):
        self.node.destroy_node()

    def command_velocity_callback(self, msg):
        self.node.get_logger().info('Command: %f %f' % (msg.twist.linear.x, msg.twist.angular.z))
        self.command = msg.twist

    def timer_callback(self):
        # Propagate command
        period = 0.05
        self.x += cos(self.theta) * self.command.linear.x * period
        self.y += sin(self.theta) * self.command.linear.x * period
        self.theta += self.command.angular.z * period
        # Need to publish updated TF
        self.publish()

    def publish(self):
        # Publish base->odom transform
        t = TransformStamped()
        t.header.stamp = self.node.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = sin(self.theta / 2.0)
        t.transform.rotation.w = cos(self.theta / 2.0)
        self.tf_broadcaster.sendTransform(t)
        # Publish battery state
        b = BatteryState()
        if self.is_charging:
            b.current = 1.0
        else:
            b.current = -1.0
        self.battery_state_pub.publish(b)

    def action_feedback_callback(self, msg):
        # Force the docking action to run a full recovery loop and then
        # make contact with the dock (based on pose of robot) before
        # we report that the robot is charging
        if msg.feedback.num_retries > 0 and \
                msg.feedback.state == msg.feedback.WAIT_FOR_CHARGE:
            self.is_charging = True

    def nav_execute_callback(self, goal_handle):
        goal = goal_handle.request
        self.x = goal.pose.pose.position.x - 0.05
        self.y = goal.pose.pose.position.y + 0.05
        self.theta = 2.0 * acos(goal.pose.pose.orientation.w)
        self.node.get_logger().info('Navigating to %f %f %f' % (self.x, self.y, self.theta))
        goal_handle.succeed()
        self.publish()

        result = NavigateToPose.Result()
        result.error_code = 0
        result.error_msg = ''
        return result

    def test_docking_server(self):
        # Publish TF for odometry
        self.tf_broadcaster = TransformBroadcaster(self.node)

        # Create a timer to run "control loop" at 20hz
        self.timer = self.node.create_timer(0.05, self.timer_callback)

        # Create action client
        self.dock_action_client = ActionClient(self.node, DockRobot, 'dock_robot')
        self.undock_action_client = ActionClient(self.node, UndockRobot, 'undock_robot')

        # Subscribe to command velocity
        self.node.create_subscription(
            TwistStamped,
            'cmd_vel',
            self.command_velocity_callback,
            10
        )

        # Create publisher for battery state message
        self.battery_state_pub = self.node.create_publisher(
            BatteryState,
            'battery_state',
            10
        )

        # Mock out navigation server (just teleport the robot)
        self.action_server = ActionServer(
            self.node,
            NavigateToPose,
            'navigate_to_pose',
            self.nav_execute_callback)

        # Publish transform
        self.publish()

        # Run for 1 seconds to allow tf to propagate
        for _ in range(10):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Test docking action
        self.action_result = []
        assert self.dock_action_client.wait_for_server(timeout_sec=5.0), \
               'dock_robot service not available'

        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = 'test_dock'
        future = self.dock_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle is not None, 'goal_handle should not be None'
        assert self.goal_handle.accepted, 'goal_handle not accepted'
        result_future_original = self.goal_handle.get_result_async()

        # Run for 2 seconds
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Send another goal to preempt the first
        future = self.dock_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle is not None, 'goal_handle should not be None'
        assert self.goal_handle.accepted, 'goal_handle not accepted'
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.action_result.append(result_future.result())

        rclpy.spin_until_future_complete(self.node, result_future_original)
        self.action_result.append(result_future_original.result())

        # First is aborted due to preemption
        self.assertEqual(self.action_result[0].status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.action_result[0].result.success)

        self.node.get_logger().info('Goal preempted')

        # Run for 0.5 seconds
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Second is aborted due to preemption during main loop (takes down all actions)
        self.assertEqual(self.action_result[1].status, GoalStatus.STATUS_ABORTED)
        self.assertFalse(self.action_result[1].result.success)

        # Resend the goal
        self.node.get_logger().info('Sending goal again')
        future = self.dock_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle is not None, 'goal_handle should not be None'
        assert self.goal_handle.accepted, 'goal_handle not accepted'
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.action_result.append(result_future.result())

        self.assertEqual(self.action_result[2].status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(self.action_result[2].result.success)
        self.assertEqual(self.action_result[2].result.num_retries, 1)

        # Test undocking action
        self.is_charging = False
        self.undock_action_client.wait_for_server(timeout_sec=5.0)
        goal = UndockRobot.Goal()
        goal.dock_type = 'test_dock_plugin'
        future = self.undock_action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle is not None, 'goal_handle should not be None'
        assert self.goal_handle.accepted, 'goal_handle not accepted'
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.action_result.append(result_future.result())

        self.assertEqual(self.action_result[3].status, GoalStatus.STATUS_SUCCEEDED)
        self.assertTrue(self.action_result[3].result.success)


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info):
        # Check that all processes in the launch exit with code 0
        launch_testing.asserts.assertExitCodes(proc_info)
