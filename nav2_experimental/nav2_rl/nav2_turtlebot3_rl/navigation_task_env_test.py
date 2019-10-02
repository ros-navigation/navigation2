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

from navigation_task_env import NavigationTaskEnv

import numpy as np
from math import pi, atan2, sin, cos
import math
import random
from time import sleep

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
import nav2_msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist, Pose


class Test(NavigationTaskEnv):
    def __init__(self):
        super().__init__()

    def get_actions(self):
        pass

    def main(self, args=None):
        '''
        Call reset and check if new path is received
        '''
        #rclpy.init(args=args)

        for _ in range(1):
            self.reset()
            print("Received path length is : " + str(len(self.path.poses)))

if __name__=="__main__":
    test = Test()
    test.main()
