#! /usr/bin/env python3
# Copyright (c) 2019 Samsung Research America
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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_testing.legacy import LaunchTestService
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')
    params_file = os.path.join(os.getenv('TEST_DIR'), 'error_code_param.yaml')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(bringup_dir, 'launch', 'tb3_simulation_launch.py')),
            launch_arguments={
                'use_rviz': 'False',
                'use_composition': 'False',
                'params_file': params_file}.items())
    ])


def main(argv=sys.argv[1:]):
    ld = generate_launch_description()

    test_error_codes_action = ExecuteProcess(
        cmd=[os.path.join(os.getenv('TEST_DIR'), 'test_error_codes.py')],
        name='test_error_codes',
        output='screen')

    lts = LaunchTestService()
    lts.add_test_action(ld, test_error_codes_action)
    ls = LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return lts.run(ls)


if __name__ == '__main__':
    sys.exit(main())
