# Copyright (c) 2024 Open Navigation LLC
# Copyright (c) 2024 Alberto J. Tudela Roldán
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

from math import cos, sin
import os
import threading
import time
from typing import Callable
import unittest

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, TransformStamped, Twist, TwistStamped
from launch import LaunchDescription
from launch_ros.actions import Node
import launch_testing
import launch_testing.actions
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import FollowObject
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.client import Client
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
import tf2_ros
from tf2_ros import TransformBroadcaster

# This test can be run standalone with:
# python3 -u -m pytest test_following_server.py -s

# If python3-flaky is installed, you can run the test multiple times to
# try to identify flaky ness.
# python3 -u -m pytest --force-flaky --min-passes 3 --max-runs 5 -s -v test_following_server.py


@pytest.mark.rostest
# @pytest.mark.flaky
# @pytest.mark.flaky(max_runs=5, min_passes=3)
def generate_test_description() -> LaunchDescription:

    return LaunchDescription([
        # SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        # SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        Node(
            package='opennav_following',
            executable='opennav_following',
            name='following_server',
            parameters=[{'desired_distance': 0.5,
                         'detection_timeout': 0.75,
                         'linear_tolerance': 0.05,
                         'angular_tolerance': 0.05,
                         'transform_tolerance': 0.5,
                         'static_object_timeout': 0.5,
                         'search_by_rotating': True,
                         'controller': {
                             'use_collision_detection': False,
                             'rotate_to_heading_max_angular_accel': 25.0,
                             'transform_tolerance': 0.5,
                         }}],
            output='screen',
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['following_server']}]
        ),
        launch_testing.actions.ReadyToTest(),  # type: ignore[no-untyped-call]
    ])


class ObjectPublisher:
    """
    A class that continuously publishes the tested object pose or frame at a fixed rate.

    The publisher runs in a separate thread and stops publishing when the provided
    at_distance_getter callable returns True.
    """

    def __init__(
        self,
        topic_name: str,
        frame_name: str,
        rate_hz: float,
        at_distance_getter: Callable[[], bool],
        mode: str = 'topic',
    ):
        # Create a dedicated node and executor so timers and clocks behave correctly
        self._node = rclpy.create_node('test_object_pose_publisher')
        self._pub = self._node.create_publisher(PoseStamped, topic_name, 10)
        self._at_distance_getter = at_distance_getter
        self._mode = mode
        self._frame_name = frame_name

        # Timer will drive publishing for both modes
        self._timer = self._node.create_timer(1.0 / rate_hz, self._timer_cb)

        # Use a single-threaded executor to spin this node in the background
        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self._node)

        # If in frame mode, create a TransformBroadcaster for publishing TF
        self._tf_broadcaster: TransformBroadcaster | None = None
        if self._mode == 'frame':
            self._tf_broadcaster = TransformBroadcaster(self._node)

        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._spin, daemon=True)
        self._thread.start()

    def _timer_cb(self) -> None:
        # Called in executor context at the configured rate
        if self._at_distance_getter():
            return
        if self._mode == 'topic':
            p = PoseStamped()
            p.header.stamp = self._node.get_clock().now().to_msg()
            p.header.frame_id = 'map'
            p.pose.position.x = 1.75
            p.pose.position.y = 0.0
            self._pub.publish(p)
            self._node.get_logger().debug('Publishing pose')
        elif self._mode == 'frame':
            # Publish a TF map -> object_frame
            t = TransformStamped()
            t.header.stamp = self._node.get_clock().now().to_msg()
            t.header.frame_id = 'map'
            t.child_frame_id = self._frame_name
            t.transform.translation.x = 1.75
            t.transform.translation.y = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            if self._tf_broadcaster is not None:
                self._tf_broadcaster.sendTransform(t)

    def _spin(self) -> None:
        # Spin until stop event is set
        try:
            while not self._stop_event.is_set():
                self._executor.spin_once(timeout_sec=0.1)
        except Exception:
            # Ensure we don't crash the main test thread
            pass

    def shutdown(self) -> None:
        self._stop_event.set()
        # Allow the executor loop to finish
        self._thread.join(timeout=1.0)
        try:
            self._executor.remove_node(self._node)
        except Exception:
            pass
        try:
            self._node.destroy_node()
        except Exception:
            pass


class TestFollowingServer(unittest.TestCase):

    @classmethod
    def setUpClass(cls) -> None:
        rclpy.init()

    @classmethod
    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        # Create a ROS node for tests
        # Latest odom -> base_link
        self.x: float = 0.0
        self.y: float = 0.0
        self.theta: float = 0.0
        # Track states
        self.at_distance: bool = False
        self.retry_state: bool = False
        # Latest command velocity
        self.command: Twist = Twist()
        self.node = rclpy.create_node('test_following_server')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self.node)
        # Determine test mode from environment: 'topic, 'frame' or 'search'
        mode_env = os.getenv('FOLLOWING_MODE', 'topic')
        # In 'search' mode stop publishing once the robot reaches 0.75m so the server must
        # perform its recovery behavior (object lost)
        if mode_env == 'search':
            def at_distance_getter() -> bool:
                return bool(((self.x ** 2 + self.y ** 2) ** 0.5) >= 0.75)
            pub_mode = 'topic'
        else:
            def at_distance_getter() -> bool:
                return self.at_distance
            pub_mode = mode_env

        self.object_publisher = ObjectPublisher(
            'tested_pose',
            'object_frame',
            20.0,
            at_distance_getter,
            pub_mode,
        )

    def wait_for_node_to_be_active(self, node_name: str, timeout_sec: float = 10.0) -> None:
        """Wait for a managed node to become active."""
        client: Client[GetState.Request, GetState.Response] = (  # type: ignore[name-defined]
            self.node.create_client(GetState, f'{node_name}/get_state')  # type: ignore[arg-type]
        )
        if not client.wait_for_service(timeout_sec=2.0):
            self.fail(f'Service get_state for {node_name} not available.')

        start_time = time.time()
        while time.time() - start_time < timeout_sec:
            req = GetState.Request()  # empty request
            future = client.call_async(req)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
            result = future.result()
            if result is not None and result.current_state.id == 3:  # 3 = ACTIVE
                self.node.get_logger().info(f'Node {node_name} is active.')
                return
            time.sleep(0.5)
        # raises AssertionError
        self.fail(f'Node {node_name} did not become active within {timeout_sec} seconds.')

    def tearDown(self) -> None:
        self.object_publisher.shutdown()
        self.node.destroy_node()

    def command_velocity_callback(self, msg: TwistStamped) -> None:
        self.node.get_logger().info(f'Command: {msg.twist.linear.x:f} {msg.twist.angular.z:f}')
        self.command = msg.twist

    def timer_callback(self) -> None:
        # Propagate command
        period = 0.05
        self.x += cos(self.theta) * self.command.linear.x * period
        self.y += sin(self.theta) * self.command.linear.x * period
        self.theta += self.command.angular.z * period
        # Need to publish updated TF
        self.publish()

    def publish(self) -> None:
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
        # Also publish map->odom transform so object pose in 'map' can be transformed
        m = TransformStamped()
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.child_frame_id = 'odom'
        m.transform.translation.x = 0.0
        m.transform.translation.y = 0.0
        m.transform.rotation.x = 0.0
        m.transform.rotation.y = 0.0
        m.transform.rotation.z = 0.0
        m.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(m)

    def action_feedback_callback(
        self,
        msg: FollowObject.Feedback  # type: ignore[name-defined]
    ) -> None:
        # Force the following action to run a full recovery loop when
        # the robot is at distance
        if msg.feedback.state == msg.feedback.STOPPING:
            self.at_distance = True
        elif msg.feedback.state == msg.feedback.RETRY:
            self.at_distance = False
            self.retry_state = True

    def test_following_server(self) -> None:
        # Publish TF for odometry
        self.tf_broadcaster = TransformBroadcaster(self.node)
        time.sleep(0.5)

        # Create a timer to run "control loop" at 20hz
        self.timer = self.node.create_timer(0.05, self.timer_callback)

        # Create action client
        self.follow_action_client: ActionClient[  # type: ignore[name-defined]
            FollowObject.Goal, FollowObject.Result, FollowObject.Feedback
        ] = ActionClient(self.node, FollowObject, 'follow_object')  # type: ignore[arg-type]

        # Subscribe to command velocity
        self.node.create_subscription(
            TwistStamped,
            'cmd_vel',
            self.command_velocity_callback,
            10
        )

        # Publish transform
        self.publish()

        # Wait until the transform is available.
        self.node.get_logger().info('Waiting for TF odom->base_link to be available...')
        start_time = time.time()
        timeout = 10.0
        while not self.tf_buffer.can_transform('odom', 'base_link', rclpy.time.Time()):
            if time.time() - start_time > timeout:
                self.fail('TF transform odom->base_link not available after 10s')
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)
        self.node.get_logger().info('TF is ready, proceeding with test.')

        # Wait until the following server is active.
        self.wait_for_node_to_be_active('following_server')

        # Test follow action with an object at 1.75m in front of the robot
        self.action_result = []
        assert self.follow_action_client.wait_for_server(timeout_sec=5.0), \
            'follow_object service not available'

        # Create the goal
        goal = FollowObject.Goal()
        if os.getenv('FOLLOWING_MODE') == 'topic' or os.getenv('FOLLOWING_MODE') == 'search':
            goal.pose_topic = 'tested_pose'
            goal.max_duration = Duration(seconds=10.0).to_msg()
        elif os.getenv('FOLLOWING_MODE') == 'frame':
            goal.tracked_frame = 'object_frame'
            goal.max_duration = Duration(seconds=4.0).to_msg()

        # Send a goal
        self.node.get_logger().info('Sending first goal')
        future = self.follow_action_client.send_goal_async(
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
        self.node.get_logger().info('Preempting with a new goal')
        future = self.follow_action_client.send_goal_async(
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
        self.assertIsNotNone(self.action_result[0])
        if self.action_result[0] is not None:
            self.assertEqual(self.action_result[0].status, GoalStatus.STATUS_ABORTED)
            self.assertTrue(self.action_result[0].result, FollowObject.Result.NONE)
            self.assertFalse(self.at_distance)

        self.node.get_logger().info('Goal preempted')

        # Run for 0.5 seconds
        for _ in range(5):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            time.sleep(0.1)

        # Second is aborted due to preemption during main loop (takes down all actions)
        self.assertIsNotNone(self.action_result[1])
        if self.action_result[1] is not None:
            self.assertEqual(self.action_result[1].status, GoalStatus.STATUS_ABORTED)
            self.assertTrue(self.action_result[1].result, FollowObject.Result.NONE)
            self.assertFalse(self.at_distance)

        # Resend the goal
        self.node.get_logger().info('Sending goal again')
        future = self.follow_action_client.send_goal_async(
            goal, feedback_callback=self.action_feedback_callback)
        rclpy.spin_until_future_complete(self.node, future)
        self.goal_handle = future.result()
        assert self.goal_handle is not None, 'goal_handle should not be None'
        assert self.goal_handle.accepted, 'goal_handle not accepted'
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.action_result.append(result_future.result())

        self.assertIsNotNone(self.action_result[2])
        if self.action_result[2] is not None:
            mode = os.getenv('FOLLOWING_MODE')
            if mode != 'skip_pose' and mode != 'search':
                self.assertEqual(self.action_result[2].status, GoalStatus.STATUS_SUCCEEDED)
                self.assertEqual(self.action_result[2].result.num_retries, 0)
                self.assertTrue(self.at_distance)


@launch_testing.post_shutdown_test()  # type: ignore[no-untyped-call]
class TestProcessOutput(unittest.TestCase):

    def test_exit_code(self, proc_info: launch_testing.ProcInfoHandler) -> None:
        # Check that all processes in the launch exit with code 0
        launch_testing.asserts.assertExitCodes(proc_info)  # type: ignore[no-untyped-call]
