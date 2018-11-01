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

"""Module for ProcessStdin event."""

from .process_io import ProcessIO


class ProcessStdin(ProcessIO):
    """Event emitted when a process needs to handle stdin."""

    name = 'launch.events.process.ProcessStdin'

    def __init__(self, *, text: bytes, **kwargs) -> None:
        """
        Constructor.

        Unmatched keyword arguments are passed to ProcessEvent, see it for
        details on those arguments.

        :param: text is the unicode data associated with the event
        """
        super().__init__(text=text, fd=0, **kwargs)
