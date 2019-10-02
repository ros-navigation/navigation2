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

from core_env import Env

from time import sleep
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from std_srvs.srv import Empty
from gazebo_msgs.srv import GetEntityState, SetEntityState


class GazeboInterface(Env):
    def __init__(self):
        super()
        self.GazeboInterface = GazeboInterface
        rclpy.init()
        self.node_ = rclpy.create_node('gazebo_interface')
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node_)
        self.node_.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.reset_simulation = self.node_.create_client(Empty, 'reset_simulation')
        self.reset_world = self.node_.create_client(Empty, 'reset_world')
        self.unpause_proxy = self.node_.create_client(Empty, 'unpause_physics')
        self.pause_proxy = self.node_.create_client(Empty, 'pause_physics')
        self.get_entity_state = self.node_.create_client(GetEntityState, 'get_entity_state')
        self.set_entity_state = self.node_.create_client(SetEntityState, 'set_entity_state')
        self.t = Thread(target=self.executor.spin)
        self.t.start()
        self.time_factor = 1.0
        self.time_to_sample = 1.0

    def get_robot_pose(self):
        """Gets the robot pose with respect to the world

        Argument:
            None

        Returns:
            The robot pose
        """
        raise NotImplementedError()

    def cleanup(self):
        self.t.join()

    # Rate object is not yet available in rclpy. Thus, we created this method to calculate the
    # difference between simulation time and system time
    def get_time_factor(self):
        sim_time_start = self.node_._clock.now()
        sleep(self.time_to_sample)
        sim_time_end = self.node_._clock.now()
        sim_time_dif = (sim_time_end.nanoseconds - sim_time_start.nanoseconds) / 1e9
        return sim_time_dif / self.time_to_sample

    def set_entity_state_pose(self, entity_name, entity_pose):
        while not self.set_entity_state.wait_for_service(timeout_sec=1.0):
            print('Set entity state service is not available...')

        req = SetEntityState.Request()
        req.state.name = entity_name
        req.state.pose.position = entity_pose.position
        req.state.pose.position.z = 0.0
        req.state.pose.orientation = entity_pose.orientation
        future = self.set_entity_state.call_async(req)
        while not future.done() and rclpy.ok():
            sleep(0.1)
        sleep(0.5)

    def pause_gazebo_world(self):
        while not self.pause_proxy.wait_for_service(timeout_sec=1.0 / self.time_factor):
            print('Pause Environment service is not available...')
        self.pause_proxy.call_async(Empty.Request())

    def unpause_gazebo_world(self):
        while not self.unpause_proxy.wait_for_service(timeout_sec=1.0 / self.time_factor):
            print('Unpause Environment service is not available...')
        self.unpause_proxy.call_async(Empty.Request())

    def reset_gazebo_world(self):
        while not self.reset_world.wait_for_service(timeout_sec=1.0):
            print('Reset world service is not available...')
        self.reset_world.call_async(Empty.Request())

    def reset_gazebo_simulation(self):
        while not self.reset_simulation.wait_for_service(timeout_sec=1.0):
            print('Reset simulation service is not available...')
        self.reset_simulation.call_async(Empty.Request())
