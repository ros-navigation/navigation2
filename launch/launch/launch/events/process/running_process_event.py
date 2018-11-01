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

"""Module for RunningProcessEvent event."""

from typing import Dict
from typing import List
from typing import Optional
from typing import Text

from ...event import Event

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ...actions import ExecuteProcess  # noqa


class RunningProcessEvent(Event):
    """Event base class that is related to some running process."""

    name = 'launch.events.process.RunningProcessEvent'

    def __init__(
        self,
        *,
        action: 'ExecuteProcess',
        name: Text,
        cmd: List[Text],
        cwd: Optional[Text],
        env: Optional[Dict[Text, Text]],
        pid: int
    ) -> None:
        """
        Constructor.

        :param: action is the ExecuteProcess action associated with the event
        :param: name is the final name of the process instance, which is unique
        :param: cmd is the final command after substitution expansion
        :param: cwd is the final working directory after substitution expansion
        :param: env is the final environment variables after substitution expansion
        """
        super().__init__()
        self.__action = action
        self.__name = name
        self.__cmd = cmd
        self.__cwd = cwd
        self.__env = env
        self.__pid = pid

    @property
    def action(self) -> 'ExecuteProcess':
        """Getter for action."""
        return self.__action

    @property
    def execute_process_action(self) -> 'ExecuteProcess':
        """Getter for execute_process_action."""
        return self.__action

    @property
    def process_name(self) -> Text:
        """Getter for process_name."""
        return self.__name

    @property
    def cmd(self) -> List[Text]:
        """Getter for cmd."""
        return self.__cmd

    @property
    def cwd(self) -> Optional[Text]:
        """Getter for cwd."""
        return self.__cwd

    @property
    def env(self) -> Optional[Dict[Text, Text]]:
        """Getter for env."""
        return self.__env

    @property
    def pid(self) -> int:
        """Getter for pid."""
        return self.__pid
