#!/usr/bin/env python3

# Copyright (c) 2018 Intel Corporation
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
import sys

from launch import LaunchDescription, LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.legacy import LaunchTestService


def main(argv: list[str] = sys.argv[1:]):  # type: ignore[no-untyped-def]
    launchDir = os.getenv('TEST_LAUNCH_DIR', '')
    testExecutable = os.getenv('TEST_EXECUTABLE', '')

    launchFile = os.path.join(launchDir, 'map_saver_node.launch.py')
    ld = LaunchDescription(
        [
            IncludeLaunchDescription(PythonLaunchDescriptionSource([launchFile])),
        ]
    )
    test1_action = ExecuteProcess(
        cmd=[testExecutable],
        name='test_map_saver_node',
    )
    lts = LaunchTestService()  # type: ignore[no-untyped-call]
    lts.add_test_action(ld, test1_action)  # type: ignore[no-untyped-call]
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    os.chdir(launchDir)
    return lts.run(ls)  # type: ignore[no-untyped-call]


if __name__ == '__main__':
    sys.exit(main())
