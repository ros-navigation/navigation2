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

from cmath import sqrt
import unittest

from nav2_simple_commander.line_iterator import LineIterator


class TestLineIterator(unittest.TestCase):

    def test_type_error(self):
        # Test if a type error raised when passing invalid arguements types
        self.assertRaises(TypeError, LineIterator, 0, 0, '10', 10, '1')

    def test_value_error(self):
        # Test if a value error raised when passing negative or zero step_size
        self.assertRaises(ValueError, LineIterator, 0, 0, 10, 10, -2)
        # Test if a value error raised when passing zero length line
        self.assertRaises(ValueError, LineIterator, 2, 2, 2, 2, 1)

    def test_get_xy(self):
        # Test if the initial and final coordinates are returned correctly
        lt = LineIterator(0, 0, 5, 5, 1)
        self.assertEqual(lt.getX0(), 0)
        self.assertEqual(lt.getY0(), 0)
        self.assertEqual(lt.getX1(), 5)
        self.assertEqual(lt.getY1(), 5)

    def test_line_length(self):
        # Test if the line length is calculated correctly
        lt = LineIterator(0, 0, 5, 5, 1)
        self.assertEqual(lt.get_line_length(), sqrt(pow(5, 2) + pow(5, 2)))

    def test_straight_line(self):
        # Test if the calculations are correct for y = x
        lt = LineIterator(0, 0, 5, 5, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + i)
            self.assertEqual(lt.getY(), lt.getY0() + i)
            lt.advance()
            i += 1

        # Test if the calculations are correct for y = 2x (positive slope)
        lt = LineIterator(0, 0, 5, 10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + i)
            self.assertEqual(lt.getY(), lt.getY0() + (i * 2))
            lt.advance()
            i += 1

        # Test if the calculations are correct for y = -2x (negative slope)
        lt = LineIterator(0, 0, 5, -10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + i)
            self.assertEqual(lt.getY(), lt.getY0() + (-i * 2))
            lt.advance()
            i += 1

    def test_hor_line(self):
        # Test if the calculations are correct for y = 0x+b (horizontal line)
        lt = LineIterator(0, 10, 5, 10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + i)
            self.assertEqual(lt.getY(), lt.getY0())
            lt.advance()
            i += 1

    def test_ver_line(self):
        # Test if the calculations are correct for x = n (vertical line)
        lt = LineIterator(5, 0, 5, 10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0())
            self.assertEqual(lt.getY(), lt.getY0() + i)
            lt.advance()
            i += 1

    def test_clamp(self):
        # Test if the increments are clamped to avoid crossing the final points
        # when step_size is large with respect to line length
        lt = LineIterator(0, 0, 5, 5, 10)
        self.assertEqual(lt.getX(), 0)
        self.assertEqual(lt.getY(), 0)
        lt.advance()
        while lt.isValid():
            self.assertEqual(lt.getX(), 5)
            self.assertEqual(lt.getY(), 5)
            lt.advance()


if __name__ == '__main__':
    unittest.main()
