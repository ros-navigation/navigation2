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

"""Module for the LogInfo action."""

import logging
from typing import List
from typing import overload
from typing import Text
from typing import Union

from ..action import Action
from ..launch_context import LaunchContext
from ..substitution import Substitution
from ..utilities import normalize_to_list_of_substitutions

_logger = logging.getLogger('launch.user')


class LogInfo(Action):
    """Action that logs a message when executed."""

    @overload
    def __init__(self, *, msg: Text) -> None:
        """Construct with just Text (unicode string)."""
        ...

    @overload  # noqa: F811
    def __init__(self, *, msg: List[Union[Text, Substitution]]) -> None:
        """Construct with list of Text and Substitutions."""
        ...

    def __init__(self, *, msg, **kwargs):  # noqa: F811
        """Constructor."""
        super().__init__(**kwargs)

        self.__msg = normalize_to_list_of_substitutions(msg)

    @property
    def msg(self) -> List[Substitution]:
        """Getter for self.__msg."""
        return self.__msg

    def execute(self, context: LaunchContext) -> None:
        """Execute the action."""
        _logger.info(''.join([context.perform_substitution(sub) for sub in self.msg]))
        return None
