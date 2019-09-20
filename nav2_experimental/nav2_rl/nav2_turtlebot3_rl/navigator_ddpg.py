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

from keras.models import Sequential, Model
from keras.layers import Dense, Activation, Flatten, Input, Concatenate
from keras.optimizers import Adam

from rl.processors import WhiteningNormalizerProcessor
from rl.agents import DDPGAgent
from rl.memory import SequentialMemory
from rl.random import OrnsteinUhlenbeckProcess


class TB3Processor(WhiteningNormalizerProcessor):
    def process_action(self, action):
        return np.clip(action, -0.2, 0.2)


class TB3NavigationEnvironmentDDPG(NavigationTaskEnv):
    def get_actions(self):
        return [0.0, 0.0]

    def get_velocity_cmd(self, action):
        self.act = action
        x_vel = float(action[0])
        y_vel = 0.0
        z_vel = float(action[1])
        return x_vel, y_vel, z_vel

class NavigatorDDPG():
    def __init__(self, env):
        self.state = env.reset()
        self.observation_space = env.observation_space()
        self.build_model(env)

    def build_model(self, env):

        nb_actions = env.action_space()
        actor = Sequential()

        # Actor Model
        actor.add(Flatten(input_shape=(1,) + (self.observation_space,)))
        actor.add(Dense(32))
        actor.add(Activation('relu'))
        actor.add(Dense(32))
        actor.add(Activation('relu'))
        actor.add(Dense(32))
        actor.add(Activation('relu'))
        actor.add(Dense(nb_actions))
        actor.add(Activation('tanh'))
        print(actor.summary())

        # Critic Model
        action_input = Input(shape=(nb_actions,), name='action_input')
        observation_input = Input(shape=(1,) + (self.observation_space,), name='observation_input')
        flattened_observation = Flatten()(observation_input)
        x = Concatenate()([action_input, flattened_observation])
        x = Dense(64)(x)
        x = Activation('relu')(x)
        x = Dense(64)(x)
        x = Activation('relu')(x)
        x = Dense(64)(x)
        x = Activation('relu')(x)
        x = Dense(1)(x)
        x = Activation('linear')(x)
        critic = Model(inputs=[action_input, observation_input], outputs=x)
        print(critic.summary())

        memory = SequentialMemory(limit=400000, window_length=1)
        random_process = OrnsteinUhlenbeckProcess(size=nb_actions, theta=.15, mu=0., sigma=.3)

        # Build Agent
        self.agent = DDPGAgent(nb_actions=nb_actions,
                               actor=actor,
                               critic=critic,
                               critic_action_input=action_input,
                               memory=memory,
                               nb_steps_warmup_critic=100,
                               nb_steps_warmup_actor=100,
                               random_process=random_process,
                               gamma=.99,
                               target_model_update=100,
                               processor=TB3Processor())

        self.agent.compile(Adam(lr=.001, clipnorm=1.), metrics=['mae'])

    def train_model(self, env, action_size):
        self.agent.fit(env, nb_steps=800000, visualize=True, verbose=1, nb_max_episode_steps=300)
        self.agent.save_weights('ddpg_weights.h5f', overwrite=True)
        self.agent.test(env, nb_episodes=10, visualize=True, nb_max_episode_steps=300)

    def load_model(self, env, action_size):
        self.agent.load_weights('ddpg_weights.h5f')
        self.agent.test(env, nb_episodes=5, visualize=True, nb_max_episode_steps=300)


def main(args=None):
    rclpy.init(args=args)
    env = TB3NavigationEnvironmentDDPG()
    action_size = env.action_space()
    ddpg_agent = NavigatorDDPG(env)

    # Ctrl-C doesn't make rclpy.ok() to return false. Thus, we catch the exception with
    # `finally` to shutdown ros and terminate the background thread cleanly.
    try:
        if len(sys.argv) > 1:
            ddpg_agent.train_model(env, action_size)
        else:
            ddpg_agent.load_model(env, action_size)
    except KeyboardInterrupt:
        pass
    finally:
        env.stop_action()
        rclpy.shutdown()
        env.cleanup()
    return

if __name__ == "__main__":
    main()
