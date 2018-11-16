#! /usr/bin/env python3
# Copyright 2018 Intel Corporation.
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
import copy
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from rcl_interfaces.srv._set_parameters import SetParameters
from rclpy.parameter import Parameter
from rcl_interfaces.srv._set_parameters__request import SetParameters_Request
from time import sleep

class NavTester(Node):
    def __init__(self):
        super().__init__("navtester")
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'initialpose')
        self.goal_pub = self.create_publisher(PoseStamped, 'move_base_simple/goal')
        
        # Can't get/set model state from Gazebo yet -see https://github.com/ros-simulation/gazebo_ros_pkgs/issues/838
        # self.model_pose_sub = self.create_subscription(ModelStates, '/gazebo/model_states', self.modelStateCallback)
        # self.setPoseClient = self.create_client(SetModelState, "/gazebo/set_model_state")
        # So for now we subscribe to amcl_pose instead
        self.model_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.modelStateCallback)

        self.tfParamClient = self.create_client(SetParameters, "/static_transform_publisher/set_parameters") # TODO: remove this when TB transforms are fixed
        self.robotModel = 'turtlebot3' # TODO: make robotModel a param

    def setInitialPose(self, pose):
        self.initialPose = pose
        self.currentPose = pose
        self.setNavstackInitialPose(pose)
        # setGazeboToInitialPose() doesn't work because of this gazebo_ros issue: https://github.com/ros-simulation/gazebo_ros_pkgs/issues/838
        # --- the /gazebo/set_model_state service is not present, running Gazebo 9, Ubuntu 18.04
        # Luckily, we don't need this if the initial pose already is matched
        #self.setGazeboToInitialPose(pose)

    def setNavstackInitialPose(self, pose):
        msg = PoseWithCovarianceStamped()
        msg.pose.pose = pose
        msg.header.frame_id = "map"
        print("Publishing Initial Pose")
        self.initial_pose_pub.publish(msg)

    def setGazeboToInitialPose(self, pose):
        modelState = SetModelState.Request()
        modelState.model_state.model_name = self.robotModel
        modelState.model_state.pose = pose
        modelState.model_state.reference_frame = 'world'
        while not self.setPoseClient.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('/gazebo/set_model_state service not available, waiting again...')
        future = self.setPoseClient.call_async(modelState)
        rclpy.spin_until_future_complete(self, future)

    def setGoalPose(self, pose):
        self.goalPose = pose
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.pose = pose
        self.goal_pub.publish(msg)

    def modelStateCallback(self, msg):
        #index = msg.name.index(self.robotModel)
        #self.currentPose = msg.pose[index]
        print("Recieved amcl_pose")
        self.currentPose = msg.pose.pose

    def reachesGoal(self, timeout):
        goalReached = False
        startTime = time.time()

        while not goalReached:
            rclpy.spin_once(self, timeout_sec=1)
            if self.distanceFromGoal() < 1: # just get within one meter for now
                goalReached = True
                print("*** GOAL REACHED ***")
                return True
            elif timeout is not None:
                if (time.time() - startTime) > timeout:
                    print("Robot timed out reaching its goal!")
                    return False

    def distanceFromGoal(self):
        d_x = self.currentPose.position.x - self.goalPose.position.x
        d_y = self.currentPose.position.y - self.goalPose.position.y
        distance = math.sqrt(d_x*d_x + d_y*d_y)
        print ("Distance from goal is: ", distance)
        return distance
    
    def setSimTime(self):
        print("Setting transforms to use sim time from gazebo")
        from subprocess import call
        # loop through the problematic nodes
        for nav2_node in ("/static_transform_publisher", "/amcl", "/map_server"):
            while (call(["ros2", "param", "set", nav2_node, "use_sim_time", "True"])):
                print("Error couldn't set use_sim_time param on: ", nav2_node, " retrying...")


def test_whenGivenAGoalPose_RobotMovesToGoal(testRobot):
    # set transforms to use_sim_time
    testRobot.setSimTime()
    print ("Waiting for time to reset")
    sleep(5) # give time for everything to reset to sim time
    
    # Set initial pose to the Turtlebot3 starting position -2, 0, 0, facing towards positive X
    initialPose = Pose()
    initialPose.position.x = -2.0
    initialPose.position.y =  0.0
    initialPose.position.z =  0.0
    initialPose.orientation.x = 0.0
    initialPose.orientation.y = 0.0
    initialPose.orientation.z = 0.0
    initialPose.orientation.w = 1.0
    # Debug
    print("Setting initial pose")
    testRobot.setInitialPose(initialPose)
    
    print("Waiting for pose to propagate to nodes")
    sleep(5)
    
    goalPose = copy.deepcopy(initialPose)
    goalPose.position.x = 0.0
    goalPose.position.y = 2.0
    print("Setting goal pose")
    testRobot.setGoalPose(goalPose)
    
    print("Waiting 30 seconds for robot to reach goal")
    return testRobot.reachesGoal(timeout=30)

def test_all():
    testRobot = NavTester()
    result = test_whenGivenAGoalPose_RobotMovesToGoal(testRobot)
    # Add more tests here if desired
    return result

def main(argv=sys.argv[1:]):
    print("Starting test_system_node")
    rclpy.init()
    # wait a few seconds to make sure entire stack is up and running
    print("Waiting for a few seconds for all nodes to initialize")
    sleep(5)
    # run tests
    result =  test_all()
    if (result):
        print("Test PASSED")
    else:
        raise Exception("Test FAILED") # raise Exception so that colcon test will fail
    
if __name__ == '__main__':
    main()
