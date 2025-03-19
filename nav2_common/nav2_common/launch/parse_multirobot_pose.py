# Copyright (c) 2023 LG Electronics.
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

from typing import Dict, Text

import launch
from launch.launch_context import LaunchContext
import yaml


class ParseMultiRobotPose(launch.Substitution):
    """
    A custom substitution to parse the robots argument for multi-robot poses.

    Expects input in the format:
    robots:="robot1={x: 1.0, y: 1.0, yaw: 0.0};
             robot2={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}"`

    The individual robots are separated by a `;` and each robot consists of a name and pose object.
    The name corresponds to the namespace of the robot and name of the Gazebo object.
    The pose consists of X, Y, Z, Roll, Pitch, Yaw each of which can be omitted in which case it is
    inferred as 0.
    """

    def __init__(self, robots_argument: launch.SomeSubstitutionsType) -> None:
        super().__init__()
        self.__robots_argument = robots_argument

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return ''

    def perform(self, context: LaunchContext) -> Dict:
        """Resolve and parse the robots argument string into a dictionary."""
        robots_str = self.__robots_argument.perform(context)
        if not robots_str:
            return {}

        multirobots = {}
        for robot_entry in robots_str.split(';'):
            key_val = robot_entry.strip().split('=')
            if len(key_val) != 2:
                continue

            robot_name, pose_str = key_val[0].strip(), key_val[1].strip()
            robot_pose = yaml.safe_load(pose_str)
            # Set default values if not provided
            robot_pose.setdefault('x', 0.0)
            robot_pose.setdefault('y', 0.0)
            robot_pose.setdefault('z', 0.0)
            robot_pose.setdefault('roll', 0.0)
            robot_pose.setdefault('pitch', 0.0)
            robot_pose.setdefault('yaw', 0.0)
            multirobots[robot_name] = robot_pose
        return multirobots
