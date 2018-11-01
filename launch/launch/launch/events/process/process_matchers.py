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

"""Module for standard "process_matchers", which are used with ProcessTargetedEvents."""

from typing import Callable
from typing import Text

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ...actions import ExecuteProcess  # noqa


def matches_action(execute_process_action: 'ExecuteProcess') -> Callable[['ExecuteProcess'], bool]:
    """Return a matcher which matches based on an exact given ExecuteProcess action."""
    return lambda action: action == execute_process_action


def matches_pid(pid: int) -> Callable[['ExecuteProcess'], bool]:
    """Return a matcher which matches based on the pid of the process."""
    def matcher(action: 'ExecuteProcess') -> bool:
        if action.process_details is None:
            # This can happen if the process in the action has not been started.
            return False
        return action.process_details['pid'] == pid

    return matcher


def matches_name(name: Text) -> Callable[['ExecuteProcess'], bool]:
    """Return a matcher which matches based on the name of the ExecuteProcess action."""
    def matcher(action: 'ExecuteProcess') -> bool:
        if action.process_details is None:
            # This can happen if the process in the action has not been started.
            return False
        return action.process_details['name'] == name

    return matcher


def matches_executable(executable: Text) -> Callable[['ExecuteProcess'], bool]:
    """
    Return a matcher which matches based on the name of the executable for the process.

    The given executable is compared with the first element of the 'cmd' using
    str.endswith().
    So, for example, 'ls' would match either 'ls .' or '/usr/bin/ls .'.
    """
    def matcher(action: 'ExecuteProcess') -> bool:
        if action.process_details is None:
            # This can happen if the process in the action has not been started.
            return False
        return action.process_details['cmd'][0].endswith(executable)

    return matcher
