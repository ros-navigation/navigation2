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

from turtlebot3_env import TurtlebotEnv
from dqn import DQN

import random
import numpy as np
import tensorflow as tf

import rclpy
from rclpy.node import Node
from time import sleep
import parameters

def trainModel(env, action_size):
    state = env.reset()
    observation_space = len(state)
    agent = DQN(observation_space, action_size)
    target_model_update_counter = 0
    agent.step = 0
    for _ in range(parameters.EPISODES):
        print("Episode number: " + str(_))
        state = env.reset()
        observation_size = len(state)
        state = np.reshape(state, [1, observation_size])
        done = False
        while not done and rclpy.ok():
            agent.step += 1
            target_model_update_counter += 1
            if target_model_update_counter%parameters.TARGET_MODEL_UPDATE_STEP == 0:
                agent.save_load_model_weights()
                target_model_update_counter = 0
            action = agent.get_action(state)
            next_state, reward, done = env.step(action)
            next_state = np.reshape(next_state, [1, observation_space])
            agent.save_to_memory(state, action, reward, next_state, done)
            state = next_state
            if not done: 
                agent.experience_replay()
            sleep(parameters.LOOP_RATE)
        agent.model.save('random_crawl_model.h5')

def main(args=None):
    rclpy.init(args=args)
    env = TurtlebotEnv()
    action_size = env.action_space()
    
    # Ctrl-C doesn't make rclpy.ok() to return false. Thus, we catch the exception with
    # `finally` to shutdown ros and terminate the background thread cleanly.
    try:
        trainModel(env, action_size)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        env.cleanup()
    return

if __name__ == "__main__":
    main()

