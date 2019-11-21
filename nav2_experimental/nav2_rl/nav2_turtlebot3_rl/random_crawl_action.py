# Copyright (c) 2019 Intel Corporation
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

import os.path
from ament_index_python.packages import get_package_share_directory
from turtlebot3_env import TurtlebotEnv
import numpy as np
import rclpy
import parameters

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from time import sleep
from keras.models import load_model

import time

from nav2_msgs.action import RandomCrawl
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading


class RandomCrawlActionServer(Node):

    def __init__(self):
        super().__init__('action_server')
        self.env = TurtlebotEnv()
        self.action_size = self.env.action_space()
        print(self.action_size)
        self.state = self.env.reset()
        self.observation_space = len(self.state)
        self.state = np.reshape(self.state, [1, self.observation_space])
        pkg_share_directory = get_package_share_directory('nav2_turtlebot3_rl')
        path = os.path.join(pkg_share_directory, "saved_models/random_crawl_waffle.h5")
        self.model = load_model(path)
        q = self.model.predict(self.state)
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            RandomCrawl,
            'random_crawl',
            execute_callback=self.execute_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        self.env.cleanup()
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        while not goal_handle.is_cancel_requested and rclpy.ok():
            q_values = self.model.predict(self.state)
            action = np.argmax(q_values)
            next_state, reward, terminal = self.env.step(action)
            next_state = np.reshape(next_state, [1, self.observation_space])
            self.state = next_state
            sleep(parameters.LOOP_RATE)
        goal_handle.succeed()
        self.env.stop_action()
        result = RandomCrawl.Result()
        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = RandomCrawlActionServer()
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)

    action_server.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
