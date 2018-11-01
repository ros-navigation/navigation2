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

"""Module for SignalProcess event."""

import signal as signal_module  # to avoid confusion with .signal property in type annotations
from typing import Callable
from typing import Text
from typing import Union

from .process_targeted_event import ProcessTargetedEvent
from ...utilities import ensure_argument_type

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from ...actions import ExecuteProcess  # noqa


class SignalProcess(ProcessTargetedEvent):
    """Event emitted when a signal should be sent to a process."""

    name = 'launch.events.process.SignalProcess'

    def __init__(
        self, *,
        signal_number: Union[Text, signal_module.Signals],
        process_matcher: Callable[['ExecuteProcess'], bool]
    ) -> None:
        """
        Constructor.

        Takes an optional process matcher, which can be used to match the
        signal to a process.

        Since Windows does not support SIGKILL, the string 'SIGKILL' can be
        given instead of `signal.SIGKILL`.
        Handlers of this event can then check for the 'SIGKILL' string and do
        the appropriate thing on Windows instead of sending a signal.

        :signal_number: either the string 'SIGKILL' or a signal.Signals
        """
        super().__init__(process_matcher=process_matcher)
        ensure_argument_type(
            signal_number, (str, signal_module.Signals), 'signal_number', 'SignalProcess')
        self.__signal = signal_number

    @property
    def signal(self) -> Union[Text, signal_module.Signals]:
        """Getter for signal, it will be 'SIGKILL' or match something from the signal module."""
        return self.__signal

    @property
    def signal_name(self) -> Text:
        """
        Getter for signal_name.

        It will be something like (e.g.) 'SIGINT'.
        """
        return self.__signal if isinstance(self.__signal, str) else self.__signal.name
