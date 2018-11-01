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

"""Module for the Substitution class."""

from typing import Text

if False:
    # imports here would cause loops, but are only used as forward-references for type-checking
    from .launch_context import LaunchContext  # noqa


class Substitution:
    """Encapsulates a substitution to be performed at runtime."""

    def describe(self) -> Text:
        """
        Return a description of this substitution as a string.

        When inherited from, calling this base class's default method is not
        required.
        """
        return repr(self)

    # Note: LaunchContext is in a string here to break a circular import.
    def perform(self, context: 'LaunchContext') -> Text:
        """
        Perform the substitution, given the launch context, and return it as a string.

        This should be overridden by the derived classes, and the default
        raises NotImplementedError.

        :raises: NotImplementedError
        """
        raise NotImplementedError('perform() not implemented for Substitution base class.')
