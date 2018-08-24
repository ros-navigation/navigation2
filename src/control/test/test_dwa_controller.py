#!/usr/bin/env python3
import os
import time
import sys
import pytest
import subprocess

import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Path
from std_msgs.msg import Empty

class DwaControllerTestNode(Node):
    def __init__(self):
        super().__init__("dwa_controller_test")
        self.path_publisher_ = self.create_publisher(Path, "DwaController_command")
        self.result_subscription_ = self.create_subscription(Empty, "DwaController_result", self.result_callback)
        self.result_received = False

    def result_callback(self, msg):
        self.result_received = True

@pytest.fixture(scope="module")
def plannerNode():
    p = subprocess.Popen("ros2 run control dwa_controller", shell=True)
    yield
    p.terminate()

@pytest.fixture()
def testNode(plannerNode):
    rclpy.init()
    node = DwaControllerTestNode()

    yield node

    node.destroy_node()
    rclpy.shutdown()


def test_result_returned(testNode):
    while(testNode.count_subscribers("DwaController_command") < 1):
        rclpy.spin_once(testNode)

    testNode.path_publisher_.publish(Path())

    while(not testNode.result_received):
        rclpy.spin_once(testNode)
    assert(True)
