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

"""Tests for the Condition class."""

from launch import Condition


def test_condition_constructors():
    """Test the constructors for Condition class."""
    Condition()
    Condition(predicate=lambda context: True)


def test_condition_methods():
    """Test the methods of the Condition class."""
    class MockLaunchContext:
        ...

    condition = Condition()
    assert 'Condition' in condition.describe()
    assert condition.evaluate(MockLaunchContext()) is False

    condition = Condition(predicate=lambda context: True)
    assert condition.evaluate(MockLaunchContext()) is True

    condition = Condition(predicate=lambda context: False)
    assert condition.evaluate(MockLaunchContext()) is False
