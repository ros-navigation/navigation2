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

"""Module for the visit_all_entities_and_collect_futures() utility function."""

import asyncio
from typing import List
from typing import Tuple

from ..launch_context import LaunchContext
from ..launch_description_entity import LaunchDescriptionEntity


def visit_all_entities_and_collect_futures(
    entity: LaunchDescriptionEntity,
    context: LaunchContext
) -> List[Tuple[LaunchDescriptionEntity, asyncio.Future]]:
    """
    Visit given entity, as well as all sub-entities, and collect any futures.

    Sub-entities are visited recursively and depth-first.
    The future is collected from each entity (unless it returns None) before
    continuing on to more sub-entities.

    This function may call itself to traverse the sub-entities recursively.
    """
    sub_entities = entity.visit(context)
    entity_future = entity.get_asyncio_future()
    futures_to_return = []
    if entity_future is not None:
        futures_to_return.append((entity, entity_future))
    if sub_entities is not None:
        for sub_entity in sub_entities:
            futures_to_return += visit_all_entities_and_collect_futures(sub_entity, context)
    return [future_pair for future_pair in futures_to_return if future_pair[1] is not None]
