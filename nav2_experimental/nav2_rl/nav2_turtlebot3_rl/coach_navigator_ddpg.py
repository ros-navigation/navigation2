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

import rclpy
from rclpy.node import Node
from time import sleep

import sys
import numpy as np
import gym
from gym import spaces

import parameters
import tensorflow as tf

from rl_coach.environments.gym_environment import GymEnvironment, GymVectorEnvironment
from rl_coach.filters.filter import NoInputFilter, NoOutputFilter
from rl_coach.agents.ddpg_agent import DDPGAgentParameters
from rl_coach.architectures.layers import Dense
from rl_coach.base_parameters import VisualizationParameters, EmbedderScheme
from rl_coach.base_parameters import PresetValidationParameters, TaskParameters
from rl_coach.core_types import TrainingSteps, EnvironmentEpisodes, EnvironmentSteps
from rl_coach.environments.environment import SingleLevelSelection
from rl_coach.filters.filter import InputFilter
from rl_coach.filters.reward.reward_rescale_filter import RewardRescaleFilter
from rl_coach.graph_managers.basic_rl_graph_manager import BasicRLGraphManager
from rl_coach.graph_managers.graph_manager import ScheduleParameters


class TB3EnvironmentDDPG(NavigationTaskEnv):
    def __init__(self):
        super().__init__()
        self.default_input_filter = NoInputFilter()
        self.default_output_filter = NoOutputFilter()
        self.action_space = spaces.Box(low=-0.36,
                                       high=0.36,
                                       shape=(2,),
                                       dtype=np.float32)
        self.spec = None

    def __del__(self):
        self.cleanup()

    @property
    def unwrapped(self):
        """Completely unwrap this env.
        Returns:
            gym.Env: The base non-wrapped gym.Env instance
        """
        return self

    def get_actions(self):
        return [0.0, 0.0]

    def get_velocity_cmd(self, action):
        self.act = action
        x_vel = float(action[0])
        y_vel = 0.0
        z_vel = float(action[1])
        return x_vel, y_vel, z_vel


class NavigatorDDPG():

    def __init__(self):

        # Resetting tensorflow graph as the network has changed.
        tf.reset_default_graph()

        # Graph Scheduling
        schedule_params = ScheduleParameters()
        schedule_params.improve_steps = TrainingSteps(1000000000)
        # how many episodes to train before calling evaluation
        schedule_params.steps_between_evaluation_periods = EnvironmentEpisodes(100)
        schedule_params.evaluation_steps = EnvironmentEpisodes(2)  # how many times to evaluate
        schedule_params.heatup_steps = EnvironmentSteps(50)  # how many episodes to heatup

        # Agent
        agent_params = DDPGAgentParameters()

        # Actor Model
        agent_params.network_wrappers['actor'].\
            input_embedders_parameters['observation'].scheme = [Dense(32)]
        agent_params.network_wrappers['actor'].\
            middleware_parameters.scheme = [Dense(32), Dense(32)]
        agent_params.network_wrappers['actor'].middleware_parameters.activation_function = 'relu'

        # Critic Model
        agent_params.network_wrappers['critic'].\
            input_embedders_parameters['observation'].scheme = [Dense(64)]
        agent_params.network_wrappers['critic'].\
            middleware_parameters.scheme = [Dense(64), Dense(64)]

        # Environment
        self.env_params = GymVectorEnvironment(level='coach_navigator_ddpg:TB3EnvironmentDDPG')

        self.graph_manager = BasicRLGraphManager(agent_params=agent_params,
                                                 env_params=self.env_params,
                                                 schedule_params=schedule_params,
                                                 vis_params=VisualizationParameters())

        # Resources path was defined at the top of this notebook
        resources_path = '/home/mohammad/OTC_workDir/navigation2/temp'
        self.my_checkpoint_dir = resources_path + '/checkpoints'

    def train_model(self):
        # Checkpoints
        task_parameters1 = TaskParameters()
        task_parameters1.checkpoint_save_dir = self.my_checkpoint_dir
        task_parameters1.checkpoint_save_secs = 3600
        self.graph_manager.create_graph(task_parameters1)

        # Train
        self.graph_manager.heatup(EnvironmentSteps(5))
        for episode in range(40000):
            self.graph_manager.train_and_act(EnvironmentSteps(400))
            print("Training episode:{}".format(episode))

    def load_model(self):
        task_parameters2 = TaskParameters()
        task_parameters2.checkpoint_restore_path = self.my_checkpoint_dir
        self.graph_manager.create_graph(task_parameters2)

        # Inference
        env = GymEnvironment(**self.env_params.__dict__,
                             visualization_parameters=VisualizationParameters())

        response = env.reset_internal_state()
        for steps in range(10000):
            action_info = self.graph_manager.get_agent().choose_action(response.next_state)
            response = env.step(action_info.action)
            if steps % 400 == 0:
                print("Reward:{}".format(response.reward))
                print("State:{}, Action:{}".format(response.next_state, action_info.action))
            if response.game_over is True:
                print("step:{}".format(steps))
                response = env.reset_internal_state()


def main(args=None):
    rclpy.init(args=args)
    ddpg_agent = NavigatorDDPG()

    # Ctrl-C doesn't make rclpy.ok() to return false. Thus, we catch the exception with
    # `finally` to shutdown ros and terminate the background thread cleanly.
    try:
        if len(sys.argv) > 1:
            ddpg_agent.train_model()
        else:
            ddpg_agent.load_model()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
    return

if __name__ == "__main__":
    main()
