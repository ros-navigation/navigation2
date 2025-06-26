# Copyright (c) 2025 Nishalan Govender
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

import unittest

from launch import LaunchContext
from nav2_common.launch import LaunchConfigAsBool


class TestLaunchConfigAsBool(unittest.TestCase):

    def setUp(self) -> None:
        self.context = LaunchContext()

    def evaluate(self, val: str) -> str:
        self.context.launch_configurations['test_key'] = val
        return LaunchConfigAsBool('test_key').perform(self.context)  # type: ignore[no-any-return]

    def test_truth_values(self) -> None:
        truth = ['true', 'True', 'TRUE', '1', ' yes ', 'YeS', 'ON', 'on']
        for val in truth:
            with self.subTest(val=val):
                self.assertEqual(self.evaluate(val), 'True')

    def test_false_values(self) -> None:
        false = ['false', 'False', '0', 'no', '', 'off', 'OFF', 'nonsense']
        for val in false:
            with self.subTest(val=val):
                self.assertEqual(self.evaluate(val), 'False')
