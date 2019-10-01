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

import parameters

import gym
from keras.models import Sequential
from keras.layers import Dense, Activation, Flatten
from keras.optimizers import Adam
from rl.agents.dqn import DQNAgent
from rl.policy import LinearAnnealedPolicy, BoltzmannQPolicy, EpsGreedyQPolicy
from rl.memory import SequentialMemory


class TB3EnvironmentDQN(NavigationTaskEnv):
    def get_actions(self):
        actions = [[parameters.ZERO, parameters.ZERO],  # Stop
                   [parameters.ZERO, -parameters.SPIN_VELOCITY],  # SR
                   [parameters.FWD_VELOCITY, parameters.ZERO],  # FWD
                   [parameters.ZERO, parameters.SPIN_VELOCITY],  # SL
                   [-parameters.FWD_VELOCITY, -parameters.ZERO]]  # BWD

        return actions

    def get_velocity_cmd(self, action):
        self.act = action
        x_vel = self.actions[action][0]
        y_vel = 0.0
        z_vel = self.actions[action][1]
        return x_vel, y_vel, z_vel


class NavigatorDQN():
    def __init__(self, env):
        self.state = env.reset()
        self.observation_space = env.observation_space()
        self.build_model(env)

    def build_model(self, env):

        nb_actions = env.action_space()
        model = Sequential()

        # DQN Model
        model.add(Flatten(input_shape=(1,) + (self.observation_space,)))
        model.add(Dense(16))
        model.add(Activation('relu'))
        model.add(Dense(16))
        model.add(Activation('relu'))
        model.add(Dense(16))
        model.add(Activation('relu'))
        model.add(Dense(nb_actions))
        model.add(Activation('linear'))
        print(model.summary())

        memory = SequentialMemory(limit=40000, window_length=1)
        policy = LinearAnnealedPolicy(EpsGreedyQPolicy(),
                                      attr='eps',
                                      value_max=1.,
                                      value_min=.1,
                                      value_test=.05,
                                      nb_steps=10000)

        # Build Agent
        self.agent = DQNAgent(model=model,
                              nb_actions=nb_actions,
                              memory=memory,
                              nb_steps_warmup=100,
                              target_model_update=500,
                              policy=policy)

        self.agent.compile(Adam(lr=1e-3), metrics=['mae'])

    def train_model(self, env, action_size):
        self.agent.fit(env, nb_steps=40000, visualize=True, verbose=1, nb_max_episode_steps=300)
        self.agent.save_weights('dqn_weights.h5f', overwrite=True)
        self.agent.test(env, nb_episodes=5, visualize=True, nb_max_episode_steps=300)

    def load_model(self, env, action_size):
        self.agent.load_weights('dqn_weights.h5f')
        self.agent.test(env, nb_episodes=5, visualize=True, nb_max_episode_steps=300)


def main(args=None):
    rclpy.init(args=args)
    env = TB3EnvironmentDQN()
    action_size = env.action_space()
    dqn_agent = NavigatorDQN(env)

    # Ctrl-C doesn't make rclpy.ok() to return false. Thus, we catch the exception with
    # `finally` to shutdown ros and terminate the background thread cleanly.
    try:
        if len(sys.argv) > 1:
            dqn_agent.train_model(env, action_size)
        else:
            dqn_agent.load_model(env, action_size)
    except KeyboardInterrupt:
        pass
    finally:
        env.stop_action()
        rclpy.shutdown()
        env.cleanup()
    return

if __name__ == "__main__":
    main()
