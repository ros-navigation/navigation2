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

"""Module for Condition class."""

from typing import Callable
from typing import Optional
from typing import Text

from .launch_context import LaunchContext


class Condition:
    """
    Encapsulates a condition to be evaluated when launching.

    The given predicate receives a launch context and is evaluated while
    launching, but must return True or False.

    If a predicate is not set when evaluated, False is returned.
    """

    def __init__(self, *, predicate: Optional[Callable[[LaunchContext], bool]] = None) -> None:
        self._predicate = predicate

    def describe(self) -> Text:
        """Return a description of this Condition."""
        return self.__repr__()

    def evaluate(self, context: LaunchContext) -> bool:
        """Evaluate the condition."""
        if self._predicate is not None:
            return self._predicate(context)
        return False
