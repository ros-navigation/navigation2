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

"""Module for the TextSubstitution substitution."""

from typing import Text

from ..launch_context import LaunchContext
from ..substitution import Substitution


class TextSubstitution(Substitution):
    """Substitution that wraps a single string text."""

    def __init__(self, *, text: Text) -> None:
        """Constructor."""
        super().__init__()

        if not isinstance(text, Text):
            raise TypeError(
                "TextSubstitution expected Text object got '{}' instead.".format(type(text))
            )

        self.__text = text

    @property
    def text(self) -> Text:
        """Getter for text."""
        return self.__text

    def describe(self) -> Text:
        """Return a description of this substitution as a string."""
        return "'{}'".format(self.text)

    def perform(self, context: LaunchContext) -> Text:
        """Perform the substitution by returning the string itself."""
        return self.text
