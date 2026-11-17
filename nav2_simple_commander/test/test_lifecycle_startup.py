# Copyright (c) 2026 blessedefe4real
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
from typing import Generator

from nav2_msgs.srv import ManageLifecycleNodes
from nav2_simple_commander.robot_navigator import BasicNavigator
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class SlowLifecycleManager(Node):
    """Lifecycle manager stand-in that takes a while to answer STARTUP."""

    def __init__(self) -> None:
        super().__init__('test_lifecycle_manager')
        self.startup_done = False
        self.srv = self.create_service(
            ManageLifecycleNodes, '~/manage_nodes', self.callback)

    def callback(
        self, request: ManageLifecycleNodes.Request,
        response: ManageLifecycleNodes.Response
    ) -> ManageLifecycleNodes.Response:
        time.sleep(1.0)
        self.startup_done = True
        response.success = True
        return response


@pytest.fixture
def ros() -> Generator[None, None, None]:
    rclpy.init()
    yield
    rclpy.try_shutdown()


def test_lifecycle_startup_waits_for_response(ros: None) -> None:
    manager = SlowLifecycleManager()
    executor = SingleThreadedExecutor()
    executor.add_node(manager)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    navigator = BasicNavigator()
    # Skip the initial pose fallback, there is no localization here
    navigator.initial_pose_received = True

    try:
        # Wait for the service to be discovered
        deadline = time.monotonic() + 10.0
        while not any(
            types[0] == 'nav2_msgs/srv/ManageLifecycleNodes'
            for _, types in navigator.get_service_names_and_types()
        ):
            assert time.monotonic() < deadline, 'lifecycle manager service not discovered'
            time.sleep(0.1)

        navigator.lifecycleStartup()

        # lifecycleStartup must only return once STARTUP has been answered
        assert manager.startup_done
    finally:
        executor.shutdown()
        spin_thread.join(timeout=5.0)
        navigator.destroy_node()
        manager.destroy_node()
