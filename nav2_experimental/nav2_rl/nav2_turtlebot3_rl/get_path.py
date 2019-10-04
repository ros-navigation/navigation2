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


import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from nav2_msgs.action import ComputePathToPose
import nav2_msgs
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus


class RLPathClient(Node):
    '''
    This class is designed to use the navigation stack to publish the global path.
    Functions implemented will send the current pose, and goal pose to the global planner
    using action interface, and get the path so that Reinforcemnet Learning algorithms
    can be used to follow the path.

    It uses ComputePathToPose action.

    This is a part of experimental package.
    '''
    def __init__(self):
        super().__init__('rl_path_client')
        #self.action_client_ = ActionClient(self, NavigateToPose, 'NavigateToPose')
        self.action_client_ = ActionClient(self, ComputePathToPose, 'ComputePathToPose')
        self.result_path = None
        
    def send_goal(self, goal_pose):
        goal_msg = nav2_msgs.action.ComputePathToPose.Goal()
        goal_msg.pose = goal_pose

        self.action_client_.wait_for_server()
        print("Server ready : "+ str(self.action_client_.server_is_ready()) + str('. Goal Sent!'))

        self._send_goal_future = self.action_client_.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print('Goal rejected :(')
            return

        print('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            print('Goal succeeded! Received path of length ' + str(len(result.path.poses)) + str('.'))
        else:
            print("Failed to get path :S")

        self.result_path = result.path.poses

    def feedback_callback(self, feedback_msg):
        print("Feedback not implemented?")


def test(args=None):
    rclpy.init(args=args)

    test_goal = PoseStamped()
    test_goal.pose.position.x = 0.0
    test_goal.pose.position.y = 2.0
    test_goal.pose.position.z = 0.0
    
    RLaction_client = RLPathClient()
    RLaction_client.send_goal(test_goal)
    
    if RLaction_client.result_path!=None:
        print(RLaction_client.result_path)

    executor = MultiThreadedExecutor()
    rclpy.spin(RLaction_client, executor=executor)
    #rclpy.shutdown()


if __name__ == '__main__':
    test()
