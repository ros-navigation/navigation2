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

"""Module for the LaunchIntrospector class."""

import logging
from typing import cast
from typing import List
from typing import Text

from .action import Action
from .actions import EmitEvent
from .actions import ExecuteProcess
from .actions import LogInfo
from .actions import RegisterEventHandler
from .event_handler import EventHandler
from .launch_description import LaunchDescription
from .launch_description_entity import LaunchDescriptionEntity
from .some_substitutions_type import SomeSubstitutionsType
from .utilities import is_a
from .utilities import normalize_to_list_of_substitutions

_logger = logging.getLogger(name='launch')


def indent(lines: List[Text], indention: Text = '    ') -> List[Text]:
    """Indent a list of strings and return them."""
    return ['{}{}'.format(indention, line) for line in lines]


def tree_like_indent(lines: List[Text]) -> List[Text]:
    """Replace whitespace with "tree"-like indentation symbols."""
    result = []
    previous_first_non_whitespace = None
    for old_line in lines:
        if not old_line.startswith('    '):
            continue
        line = str(old_line)
        line = '│' + line[1:]
        first_non_whitespace = len(old_line) - len(old_line.lstrip())
        if previous_first_non_whitespace is not None and '└' in result[-1]:
            if previous_first_non_whitespace <= first_non_whitespace:
                result[-1] = result[-1].replace('└', '├', 1)
        previous_first_non_whitespace = first_non_whitespace
        line = line[0:first_non_whitespace - 4] + '└── ' + line[first_non_whitespace:]
        result.append(line)
    if result[-1].startswith('│'):
        result[-1] = ' ' + result[-1][1:]
    # TODO(wjwwood): figure out how to handle remaining cases like how to fix this sample:
    # ├── OnProcessExit(...)
    # │   └── Action(...)
    # ├── OnProcessExit(...)
    #     └── Action(...)
    # ^ this dangling stub
    return result


def format_entities(entities: List[LaunchDescriptionEntity]) -> List[Text]:
    """Return a list of lines of text that represent of a list of LaunchDescriptionEntity's."""
    result = []
    for entity in entities:
        if is_a(entity, Action):
            result.extend(format_action(cast(Action, entity)))
        else:
            result.append("Unknown entity('{}')".format(entity))
    return result


def format_substitutions(substitutions: SomeSubstitutionsType) -> Text:
    """Return a text representation of some set of substitutions."""
    normalized_substitutions = normalize_to_list_of_substitutions(substitutions)
    return ' + '.join([sub.describe() for sub in normalized_substitutions])


def format_event_handler(event_handler: EventHandler) -> List[Text]:
    """Return a text representation of an event handler."""
    if hasattr(event_handler, 'describe'):
        # TODO(wjwwood): consider supporting mode complex descriptions of branching
        description, entities = event_handler.describe()  # type: ignore
        result = [description]
        result.extend(indent(format_entities(entities)))
        return result
    else:
        return ["EventHandler('{}')".format(hex(id(event_handler)))]


def format_action(action: Action) -> List[Text]:
    """Return a text representation of an action."""
    if is_a(action, LogInfo):
        return ['LogInfo({})'.format(format_substitutions(cast(LogInfo, action).msg))]
    elif is_a(action, EmitEvent):
        return ["EmitEvent(event='{}')".format(cast(EmitEvent, action).event.name)]
    elif is_a(action, ExecuteProcess):
        typed_action = cast(ExecuteProcess, action)
        msg = 'ExecuteProcess(cmd=[{}], cwd={}, env={}, shell={})'.format(
            ', '.join([format_substitutions(x) for x in typed_action.cmd]),
            typed_action.cwd if typed_action.cwd is None else "'{}'".format(
                format_substitutions(typed_action.cwd)
            ),
            typed_action.env if typed_action.env is None else '{' + ', '.join(
                ['{}: {}'.format(format_substitutions(k), format_substitutions(v))
                 for k, v in typed_action.env.items()]) + '}',
            typed_action.shell,
        )
        return [msg]
    elif is_a(action, RegisterEventHandler):
        # Different variable name used to assist with type checking.
        typed_action2 = cast(RegisterEventHandler, action)
        result = ["RegisterEventHandler('{}'):".format(typed_action2.event_handler)]
        result.extend(indent(format_event_handler(typed_action2.event_handler)))
        return result
    else:
        return ["Action('{}')".format(action)]


class LaunchIntrospector:
    """Provides an interface through which you can visit all entities of a LaunchDescription."""

    def format_launch_description(self, launch_description: LaunchDescription) -> Text:
        """Return a string representation of a LaunchDescription."""
        result = '{}\n'.format(launch_description)
        entity_descriptions = format_entities(launch_description.entities)
        result += '\n'.join(tree_like_indent(indent(entity_descriptions)))
        return result
