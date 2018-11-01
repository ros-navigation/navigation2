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

"""Module for the OpaqueFunction action."""

import collections
from typing import Any
from typing import Callable
from typing import Dict
from typing import Iterable
from typing import List
from typing import Optional
from typing import Text

from ..action import Action
from ..launch_context import LaunchContext
from ..utilities import ensure_argument_type


class OpaqueFunction(Action):
    """
    Action that executes a Python function.

    The signature of the function should be:

    .. code-block:: python

        def function(
            context: LaunchContext,
            *args,
            **kwargs
        ) -> Optional[List[LaunchDescriptionEntity]]:
            ...

    """

    def __init__(
        self, *,
        function: Callable,
        args: Optional[Iterable[Any]] = None,
        kwargs: Optional[Dict[Text, Any]] = None,
        **left_over_kwargs
    ) -> None:
        """Constructor."""
        super().__init__(**left_over_kwargs)
        if not callable(function):
            raise TypeError("OpaqueFunction expected a callable for 'function', got '{}'".format(
                type(function)
            ))
        ensure_argument_type(args, (collections.Iterable, type(None)), 'args', 'OpaqueFunction')
        ensure_argument_type(kwargs, (dict, type(None)), 'kwargs', 'OpaqueFunction')
        self.__function = function
        self.__args = []  # type: Iterable
        if args is not None:
            self.__args = args
        self.__kwargs = {}  # type: Dict[Text, Any]
        if kwargs is not None:
            self.__kwargs = kwargs

    def execute(self, context: LaunchContext) -> Optional[List[Action]]:
        """Execute the action."""
        return self.__function(context, *self.__args, **self.__kwargs)
