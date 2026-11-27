# Copyright (c) 2026 Nisarg Panchal
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

import os

from ament_index_python.packages import get_package_share_directory
from rosidl_adapter.parser import parse_action_string


def error_codes(action_name: str) -> dict[str, int]:
    path = os.path.join(
        get_package_share_directory('nav2_msgs'), 'action', action_name + '.action')
    with open(path) as action_file:
        spec = parse_action_string('nav2_msgs', action_name, action_file.read())
    codes = {constant.name: constant.value for constant in spec.result.constants}
    return {name: value for name, value in codes.items() if value >= codes['UNKNOWN']}


def test_to_pose_codes_mirror_through_poses_codes() -> None:
    to_pose = error_codes('ComputePathToPose')
    through_poses = error_codes('ComputePathThroughPoses')
    offset = through_poses['UNKNOWN'] - to_pose['UNKNOWN']

    for name, value in to_pose.items():
        assert through_poses.get(name) == value + offset, name

    for name, value in through_poses.items():
        if name not in to_pose:
            assert value - offset not in to_pose.values(), name
