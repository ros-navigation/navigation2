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

import random
import numpy as np
import tensorflow as tf
import parameters

from collections import deque
from keras.models import Sequential
from keras.layers import Dense
from keras.optimizers import Adam
from keras.models import load_model

class DQN:
    def __init__(self, observation_space, action_size):
        self.action_size = action_size
        self.memory = deque(maxlen=parameters.MEMORY_SIZE)
        self.build_model(observation_space, False)
        self.target_model = self.model
        self.step = 0

    def build_model(self, observation_space, load):
        if load:
            self.model = load_model('random_crawl_model.h5')
        else:
            self.model = Sequential()
            self.model.add(Dense(8, input_shape=(observation_space,), activation="relu"))
            self.model.add(Dense(8, activation="relu"))
            self.model.add(Dense(8, activation="relu"))
            self.model.add(Dense(self.action_size, activation="linear"))
            self.model.compile(loss="mse", optimizer=Adam(lr=parameters.LEARNING_RATE))

    def save_to_memory(self, state, action, reward, next_state, done):
        self.memory.append((state, action, reward, next_state, done))
   
    def get_linear_decay_epsilon(self, step):
        if step <= parameters.EXPLORATION_TARGET_STEP:
            epsilon = parameters.EXPLORATION_MAX - step * ((parameters.EXPLORATION_MAX -\
                parameters.EXPLORATION_MIN)/parameters.EXPLORATION_TARGET_STEP)
        else:
           epsilon = parameters.EXPLORATION_MIN
        return epsilon

    def get_action(self, state):
        exploration_rate = self.get_linear_decay_epsilon(self.step)
        # Explore
        if np.random.rand() <= exploration_rate:
            action = random.randrange(self.action_size)
            return action
        # Exploit
        q_values = self.model.predict(state,batch_size=1)
        action = np.argmax(q_values)
        return action

    # Train with previous experiences
    def experience_replay(self):
        if len(self.memory) < parameters.BATCH_SIZE:
            return
        mini_batch = random.sample(self.memory, parameters.BATCH_SIZE)
        for state, action, reward, next_state, done in mini_batch:
            q_target = reward if done else reward +\
                parameters.GAMMA * np.amax(self.target_model.predict(next_state,batch_size=1))
            
            q_values = self.model.predict(state,batch_size=1)
            q_values[0][action] = q_target
            
            self.model.fit(state, q_values, verbose=0)

    # update target model every TARGET_MODEL_UPDATE_STEP steps
    def save_load_model_weights(self):
        self.model.save_weights('target_model.h5')
        self.target_model.load_weights('target_model.h5', by_name=False)