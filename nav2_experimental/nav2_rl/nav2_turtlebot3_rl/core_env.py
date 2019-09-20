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

class Env(object):
    def __init__(self):
        pass

    def step(self, action):
        """Execute one time step of the environment.
        Argument:
            action (object): an action to take at current timestep
        Returns:
            observation (object): agent's observation of the environment at the current timestep
            reward (float) : reward the agent gets after performing the action
            done (bool): if episode is ended or not
            info (dict): diognastic information
        """
        raise NotImplementedError

    def reset(self):
        """Resets the environment and brings the agent (robot) to the initial state.
        Argument:
            None
        Returns:
            observation (object): the inital observation state
        """
        raise NotImplementedError

    def render(self, mode):
        """This method is not needed gazebo+robot environment.
        """
        pass

    def compute_reward(self):
        """Takes the observation from the environment and calculates the reward.

        Argument:
            None

        Returns:
            The calculated reward after taking action on the environment
        """
        raise NotImplementedError()

    def cleanup(self):
        """This method shutsdown the running node
        """
        pass

    def observation_space(self):
        """Provides the length of observation

        Argument:
            None

        Returns:
            The observation space length
        """
        raise NotImplementedError()

    def action_space(self):
        """Provides the length of actions

        Argument:
            None

        Returns:
            The action space length
        """
        raise NotImplementedError()
    
    def observation(self):
        """Provides environment states

        # Argument
            None

        # Returns
            All the states of the environment
        """
        raise NotImplementedError()
