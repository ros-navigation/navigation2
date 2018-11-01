# Copyright 2018 Open Source Robotics Foundation, Inc.
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

"""Tests for the ExecuteProcess Action."""

import sys

from launch import LaunchDescription
from launch import LaunchService
from launch.actions.execute_process import ExecuteProcess


def test_execute_process_with_env():
    """Test launching a process with an environment variable."""
    ld = LaunchDescription([
        ExecuteProcess(
            cmd=[sys.executable, 'TEST_PROCESS_WITH_ENV'], output='screen',
            env={'TEST_PROCESS_WITH_ENV': 'Hello World'},
        )
    ])
    ls = LaunchService()
    ls.include_launch_description(ld)
    assert 0 == ls.run()
