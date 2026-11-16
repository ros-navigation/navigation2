# Copyright (c) 2026 Oke Iyanuoluwa E.
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

import threading
import time
from typing import Generator, Optional

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateThroughPoses
from nav2_simple_commander.robot_navigator import BasicNavigator, RunningTask
from nav_msgs.msg import Goals
import pytest
import rclpy
from rclpy.action import ActionServer
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class FakeNavigateThroughPoses(Node):
    """NavigateThroughPoses server stand-in that records the received goal."""

    def __init__(self) -> None:
        super().__init__('test_bt_navigator')
        self.received: Optional[Goals] = None
        self.server = ActionServer(
            self, NavigateThroughPoses, 'navigate_through_poses', self.execute)

    def execute(self, goal_handle: ServerGoalHandle) -> NavigateThroughPoses.Result:
        self.received = goal_handle.request.poses
        goal_handle.succeed()
        return NavigateThroughPoses.Result()


def make_pose(x: float, y: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    return pose


@pytest.fixture
def server() -> Generator[FakeNavigateThroughPoses, None, None]:
    rclpy.init()
    node = FakeNavigateThroughPoses()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    yield node
    executor.shutdown()
    spin_thread.join(timeout=5.0)
    node.destroy_node()
    rclpy.try_shutdown()


def send_and_wait(navigator: BasicNavigator, poses: object) -> None:
    task = navigator.goThroughPoses(poses)
    assert task == RunningTask.NAVIGATE_THROUGH_POSES
    deadline = time.monotonic() + 10.0
    while not navigator.isTaskComplete(task=task):
        assert time.monotonic() < deadline, 'NavigateThroughPoses did not complete'


def test_go_through_poses_accepts_pose_list(server: FakeNavigateThroughPoses) -> None:
    navigator = BasicNavigator()
    try:
        send_and_wait(navigator, [make_pose(1.0, 2.0), make_pose(3.0, 4.0)])
        assert server.received is not None
        assert server.received.header.frame_id == 'map'
        assert [p.pose.position.x for p in server.received.goals] == [1.0, 3.0]
    finally:
        navigator.destroy_node()


def test_go_through_poses_accepts_goals_msg(server: FakeNavigateThroughPoses) -> None:
    navigator = BasicNavigator()
    try:
        goals = Goals()
        goals.header.frame_id = 'odom'
        goals.goals = [make_pose(5.0, 6.0)]
        send_and_wait(navigator, goals)
        assert server.received is not None
        assert server.received.header.frame_id == 'odom'
        assert [p.pose.position.x for p in server.received.goals] == [5.0]
    finally:
        navigator.destroy_node()
