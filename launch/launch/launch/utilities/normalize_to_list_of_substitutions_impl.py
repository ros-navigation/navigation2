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

"""Module for the normalize_to_list_of_substitutions() utility function."""

from typing import cast
from typing import Iterable
from typing import List

from .class_tools_impl import is_a_subclass
from ..some_substitutions_type import SomeSubstitutionsType
from ..substitution import Substitution
from ..substitutions import TextSubstitution


def normalize_to_list_of_substitutions(subs: SomeSubstitutionsType) -> List[Substitution]:
    """Return a list of Substitutions given a variety of starting inputs."""
    def normalize(x):
        if isinstance(x, Substitution):
            return x
        if isinstance(x, str):
            return TextSubstitution(text=x)
        raise TypeError(
            "Failed to normalize given item of type '{}', when only "
            "'str' or 'launch.Substitution' were expected.".format(type(x)))

    if isinstance(subs, str):
        return [TextSubstitution(text=subs)]
    if is_a_subclass(subs, Substitution):
        return [cast(Substitution, subs)]
    return [normalize(y) for y in cast(Iterable, subs)]
