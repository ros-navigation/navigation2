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

"""Tests for the UnlessCondition condition class."""

from launch.conditions import InvalidConditionExpressionError
from launch.conditions import UnlessCondition
from launch.substitutions import TextSubstitution

import pytest


def test_if_condition():
    """Test UnlessCondition class."""
    class MockLaunchContext:

        def perform_substitution(self, substitution):
            return substitution.perform(self)

    lc = MockLaunchContext()
    test_cases = [
        ('true', True),
        ('True', True),
        ('TRUE', True),
        ('1', True),
        ('false', False),
        ('False', False),
        ('FALSE', False),
        ('0', False),
    ]

    for string, expected in test_cases:
        assert UnlessCondition([TextSubstitution(text=string)]).evaluate(lc) is not expected

    with pytest.raises(InvalidConditionExpressionError):
        UnlessCondition([TextSubstitution(text='')]).evaluate(lc)

    with pytest.raises(InvalidConditionExpressionError):
        UnlessCondition([TextSubstitution(text='typo')]).evaluate(lc)
