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

from collections import OrderedDict

from launch.actions import EmitEvent
from launch.actions import ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown

# for backward compatibility
from launch_testing.legacy import create_handler  # noqa: F401
from launch_testing.legacy import get_default_filtered_patterns  # noqa: F401
from launch_testing.legacy import get_default_filtered_prefixes  # noqa: F401
from launch_testing.legacy import get_rmw_output_filter  # noqa: F401
from launch_testing.legacy import InMemoryHandler  # noqa: F401
from launch_testing.legacy import UnmatchedOutputError  # noqa: F401


class LaunchTestService():

    def __init__(self):
        self.__test_processes = []
        self.__test_returncodes = OrderedDict()

    def add_test_action(self, launch_description, action):
        """
        Add action used for testing.

        If either all test actions exited with a return code of zero or any
        test action exited with a non-zero return code a shutdown event is
        emitted.
        """
        assert isinstance(action, ExecuteProcess), \
            'The passed action must be a ExecuteProcess action'

        self.__test_processes.append(action)

        def on_test_process_exit(event, context):
            nonlocal action
            nonlocal self
            self.__test_returncodes[action] = event.returncode

            if len(self.__test_returncodes) == len(self.__test_processes):
                shutdown_event = Shutdown(
                    reason='all test processes finished')
                return EmitEvent(event=shutdown_event)

        launch_description.add_action(
            RegisterEventHandler(OnProcessExit(
                target_action=action, on_exit=on_test_process_exit,
            ))
        )

    def run(self, launch_service, *args, **kwargs):
        """
        Invoke the `run` method of the launch service.

        :returns: If the return value of the parent method is zero but any of
          the test processes exited with a non-zero return code the return of
          the first failed test process is returned.
        """
        rc = launch_service.run(*args, **kwargs)
        if not rc:
            for test_process_rc in self.__test_returncodes.values():
                if test_process_rc:
                    rc = test_process_rc
                    break
        return rc
