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

"""Module for the perform_substitutions() utility function."""

from typing import List
from typing import Text

from ..launch_context import LaunchContext
from ..substitution import Substitution


def perform_substitutions(context: LaunchContext, subs: List[Substitution]) -> Text:
    """Resolve a list of Substitutions with a context into a single string."""
    return ''.join([context.perform_substitution(sub) for sub in subs])
