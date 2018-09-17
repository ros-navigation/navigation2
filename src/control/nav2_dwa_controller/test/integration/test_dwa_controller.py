#!/usr/bin/env python3

# Copyright (c) 2018 Intel Corporation
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

import os
import signal
import subprocess
import sys

from nav2_tasks.msg import Path
import pytest
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class DwaControllerTestNode(Node):

    def __init__(self):
        super().__init__('dwa_controller_test')
        self.path_publisher_ = self.create_publisher(
            Path,
            'DwaController_command')
        self.result_subscription_ = self.create_subscription(
            Empty,
            'DwaController_result',
            self.result_callback)
        self.result_received = False

    def result_callback(self, msg):
        self.result_received = True


@pytest.fixture(scope='module')
def plannerNode():
    launchFile = os.path.join(os.getenv('TEST_LAUNCH_DIR'), 'dwa_controller_node.launch.py')
    p = subprocess.Popen('ros2 launch ' + launchFile, shell=True, preexec_fn=os.setsid)
    yield
    os.killpg(p.pid, signal.SIGINT)


@pytest.fixture()
def testNode(plannerNode):
    rclpy.init()
    node = DwaControllerTestNode()
    yield node
    node.destroy_node()
    rclpy.shutdown()


def test_result_returned(testNode):
    while(testNode.count_subscribers('DwaController_command') < 1):
        rclpy.spin_once(testNode, timeout_sec=0.1)

    testNode.path_publisher_.publish(Path())

    while(not testNode.result_received):
        rclpy.spin_once(testNode, timeout_sec=0.1)
    assert(True)

def test_passing():
    assert(True)
