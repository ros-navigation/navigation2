# Copyright 2022 Afif Swaidan
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

import pytest
from nav2_simple_commander.line_iterator import LineIterator


def test_type_error():
    # Test if a type error raised when passing invalid arguements types
    with pytest.raises(TypeError):
        LineIterator(0, 0, '10', 10, '1')


def test_value_error():
    # Test if a value error raised when passing negative or zero step_size
    with pytest.raises(ValueError):
        LineIterator(0, 0, 10, 10, -2)
    # Test if a value error raised when passing zero length line
    with pytest.raises(ValueError):
        LineIterator(2, 2, 2, 2, 1)


def test_straight_line():
    # Test if the calculations are correct for y = x
    lt = LineIterator(0, 0, 5, 5, 1)
    i = 0
    while lt.isValid():
        assert lt.getX() == lt.getX0() + i
        assert lt.getY() == lt.getY0() + i
        lt.advance()
        i += 1

    # Test if the calculations are correct for y = 2x (positive slope)
    lt = LineIterator(0, 0, 5, 10, 1)
    i = 0
    while lt.isValid():
        assert lt.getX() == lt.getX0() + i
        assert lt.getY() == lt.getY0() + (i*2)
        lt.advance()
        i += 1

    # Test if the calculations are correct for y = -2x (negative slope)
    lt = LineIterator(0, 0, 5, -10, 1)
    i = 0
    while lt.isValid():
        assert lt.getX() == lt.getX0() + i
        assert lt.getY() == lt.getY0() + (-i*2)
        lt.advance()
        i += 1


def test_hor_line():
    # Test if the calculations are correct for y = 0x+b (horizontal line)
    lt = LineIterator(0, 10, 5, 10, 1)
    i = 0
    while lt.isValid():
        assert lt.getX() == lt.getX0() + i
        assert lt.getY() == lt.getY0()
        lt.advance()
        i += 1


def test_ver_line():
    # Test if the calculations are correct for x = n (vertical line)
    lt = LineIterator(5, 0, 5, 10, 1)
    i = 0
    while lt.isValid():
        assert lt.getX() == lt.getX0()
        assert lt.getY() == lt.getY0() + i
        lt.advance()
        i += 1


def test_clamp():
    # Test if the increments are clamped to avoid crossing the final points
    # when step_size is large with respect to line length
    lt = LineIterator(0, 0, 5, 5, 10)
    assert lt.getX() == 0
    assert lt.getY() == 0
    lt.advance()
    while lt.isValid():
        assert lt.getX() == 5
        assert lt.getY() == 5
        lt.advance()
