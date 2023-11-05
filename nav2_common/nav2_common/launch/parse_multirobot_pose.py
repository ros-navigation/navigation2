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

import sys
from typing import Dict, Text

import yaml


class ParseMultiRobotPose:
    """Parsing argument using sys module."""

    def __init__(self, target_argument: Text):
        """
        Parse arguments for multi-robot's pose.

        for example,
        `ros2 launch nav2_bringup bringup_multirobot_launch.py
            robots:="robot1={x: 1.0, y: 1.0, yaw: 0.0};
                     robot2={x: 1.0, y: 1.0, z: 1.0, roll: 0.0, pitch: 1.5707, yaw: 1.5707}"`

        `target_argument` shall be 'robots'.
        Then, this will parse a string value for `robots` argument.

        Each robot name which is corresponding to namespace and pose of it will be separted by `;`.
        The pose consists of x, y and yaw with YAML format.

        :param: target argument name to parse
        """
        self.__args: Text = self.__parse_argument(target_argument)

    def __parse_argument(self, target_argument: Text) -> Text:
        """Get value of target argument."""
        if len(sys.argv) > 4:
            argv = sys.argv[4:]
            for arg in argv:
                if arg.startswith(target_argument + ':='):
                    return arg.replace(target_argument + ':=', '')
        return ''

    def value(self) -> Dict:
        """Get value of target argument."""
        args = self.__args
        parsed_args = [] if len(args) == 0 else args.split(';')
        multirobots = {}
        for arg in parsed_args:
            key_val = arg.strip().split('=')
            if len(key_val) != 2:
                continue
            key = key_val[0].strip()
            val = key_val[1].strip()
            robot_pose = yaml.safe_load(val)
            if 'x' not in robot_pose:
                robot_pose['x'] = 0.0
            if 'y' not in robot_pose:
                robot_pose['y'] = 0.0
            if 'z' not in robot_pose:
                robot_pose['z'] = 0.0
            if 'roll' not in robot_pose:
                robot_pose['roll'] = 0.0
            if 'pitch' not in robot_pose:
                robot_pose['pitch'] = 0.0
            if 'yaw' not in robot_pose:
                robot_pose['yaw'] = 0.0
            multirobots[key] = robot_pose
        return multirobots
