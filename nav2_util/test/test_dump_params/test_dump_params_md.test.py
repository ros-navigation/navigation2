#! /usr/bin/env python3
# Copyright (c) 2020 Sarthak Mittal
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

import unittest

from launch import LaunchDescription
from launch.actions import ExecuteProcess

import launch_testing
import launch_testing.actions
import launch_testing.asserts
import launch_testing.util
import launch_testing_ros

import pytest


@pytest.mark.launch_test
def generate_test_description():
    launch_description = LaunchDescription()
    launch_description.add_action(
        ExecuteProcess(
            cmd=[os.path.join(os.path.dirname(__file__), 'test_dump_params_node.py')],
            name='test_dump_params')
    )
    processes_to_test = [
        ExecuteProcess(
            cmd=[os.getenv('TEST_EXECUTABLE'), '-f', 'md', '-n', 'test_dump_params'],
            name='test_dump_params_markdown',
            output='screen'),
        ExecuteProcess(
            cmd=[os.getenv('TEST_EXECUTABLE'), '-f', 'md', '-n', 'test_dump_params', '-v'],
            name='test_dump_params_markdown_verbose',
            output='screen')
    ]
    for process in processes_to_test:
        launch_description.add_action(process)
    launch_description.add_action(
        launch_testing.actions.ReadyToTest()
    )
    return launch_description, {'processes_to_test': processes_to_test}


# Tests without a unittest to run concurrently with
# the processes under test throw an exception
# The following is a dummy test to suppress the traceback
# https://github.com/ros2/launch/issues/380

class TestLoggingOutputFormat(unittest.TestCase):

    def test_logging_output(self, proc_info, processes_to_test):
        for process_name in processes_to_test:
            proc_info.assertWaitForShutdown(process=process_name, timeout=10)


@launch_testing.post_shutdown_test()
class TestDumpParams(unittest.TestCase):

    def test_processes_output(self, proc_output, processes_to_test):
        """Test all processes output against expectations."""
        from launch_testing.tools.output import get_default_filtered_prefixes
        output_filter = launch_testing_ros.tools.basic_output_filter(
            filtered_prefixes=get_default_filtered_prefixes()
        )
        output_files = [
            os.path.join(os.path.dirname(__file__), out)
            for out in ['dump_params_md',
                        'dump_params_md_verbose']
        ]
        for process, output_file in zip(processes_to_test, output_files):
            launch_testing.asserts.assertInStdout(
                proc_output,
                expected_output=launch_testing.tools.expected_output_from_file(
                    path=output_file),
                process=process, output_filter=output_filter
            )
