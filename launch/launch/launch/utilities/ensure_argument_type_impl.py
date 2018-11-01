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

"""Module for the ensure_argument_type() utility function."""

import collections
import inspect
from typing import Any
from typing import Iterable
from typing import Optional
from typing import Text
from typing import Union


def ensure_argument_type(
    argument: Any,
    types: Union[type, Iterable[type]],
    argument_name: Text,
    caller: Optional[Text] = None,
) -> None:
    """
    Ensure that the given argument is an instance of or subclass of one of the given types.

    If the argument does not match one of the types, a TypeError is raised.
    The caller is included in the error message if given and not None.
    """
    error_msg_template = "{}xpected '{}' to be one of [{}], but got '{}' of type '{}'"
    if not isinstance(types, collections.Iterable) and not isinstance(types, type):
        raise TypeError(error_msg_template.format(
            "'ensure_argument_type()' e",
            'types',
            'type, collections.Iterable of type',
            types,
            type(types),
        ))
    if not isinstance(argument_name, str):
        raise TypeError(error_msg_template.format(
            "'ensure_argument_type()' e",
            'argument_name',
            'str',
            argument_name,
            type(argument_name),
        ))
    if caller is not None and not isinstance(caller, str):
        raise TypeError(error_msg_template.format(
            "'ensure_argument_type()' e",
            'caller',
            'str, None',
            caller,
            type(caller),
        ))

    def check_argument(argument, type_var) -> bool:
        result = False
        result |= isinstance(argument, type_var)
        if hasattr(argument, '__class__') and inspect.isclass(type_var):
            result |= issubclass(argument.__class__, type_var)
        return result

    list_of_types = types if isinstance(types, collections.Iterable) else [types]
    if not any(check_argument(argument, type_var) for type_var in list_of_types):
        raise TypeError(error_msg_template.format(
            'E' if caller is None else "'{}' e".format(caller),
            argument_name,
            ', '.join([str(x) for x in list_of_types]),
            argument,
            type(argument),
        ))
