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

from time import sleep
from keras.models import load_model

def loadModel(env):
    state = env.reset()
    observation_space = len(state)
    state = np.reshape(state, [1, observation_space])
    pkg_share_directory = get_package_share_directory('nav2_turtlebot3_rl')
    path = os.path.join(pkg_share_directory,"saved_models/random_crawl_waffle.h5")
    model = load_model(path)
    while rclpy.ok():
        q_values = model.predict(state)
        action = np.argmax(q_values)
        next_state, reward, terminal = env.step(action)
        next_state = np.reshape(next_state, [1, observation_space])
        state = next_state
        sleep(parameters.LOOP_RATE)

def main(args=None):
    rclpy.init(args=args)
    env = TurtlebotEnv()
    action_size = env.action_space()
    
    # Ctrl-C doesn't make rclpy.ok() to return false. Thus, we catch the exception with
    # `finally` to shutdown ros and terminate the background thread cleanly.
    try:
        loadModel(env)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()
        env.cleanup()
    return

if __name__ == "__main__":
    main()
