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
        # Test if a type error raised when passing invalid arguments types
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
            self.assertEqual(lt.getX(), lt.getX0() + (i / 2))
            self.assertEqual(lt.getY(), lt.getY0() + i)
            lt.advance()
            i += 1
        self.assertEqual(i, 11)

        # Test if the calculations are correct for y = -2x (negative slope)
        lt = LineIterator(0, 0, 5, -10, 1)
        i = 0
        while lt.isValid():
            self.assertEqual(lt.getX(), lt.getX0() + (i / 2))
            self.assertEqual(lt.getY(), lt.getY0() - i)
            lt.advance()
            i += 1
        self.assertEqual(i, 11)

    def test_steep_line_directions(self):
        # Visit each row of a steep line, including in reverse and after reflection.
        points = [(2 + i / 10, 2 + i) for i in range(11)]
        for reflect in (False, True):
            for reverse in (False, True):
                for transpose in (False, True):
                    with self.subTest(reflect=reflect, reverse=reverse, transpose=transpose):
                        expected = [(-x if reflect else x, y) for x, y in points]
                        if reverse:
                            expected.reverse()
                        if transpose:
                            expected = [(y, x) for x, y in expected]
                        start, end = expected[0], expected[-1]
                        lt = LineIterator(*start, *end)
                        self.assertEqual((lt.getX0(), lt.getY0()), start)
                        self.assertEqual((lt.getX1(), lt.getY1()), end)
                        self.assertEqual(lt.get_line_length(), sqrt(101))
                        for x, y in expected:
                            self.assertTrue(lt.isValid())
                            self.assertAlmostEqual(lt.getX(), x)
                            self.assertAlmostEqual(lt.getY(), y)
                            lt.advance()
                        self.assertFalse(lt.isValid())

    def test_steep_line_fractional_step(self):
        # A half-cell step must also apply along Y for a steep edge.
        expected = [(2, 2), (2.1, 2.5), (2.2, 3), (2.3, 3.5),
                    (2.4, 4), (2.5, 4.5), (2.6, 5), (2.7, 5.5),
                    (2.8, 6), (2.9, 6.5), (3, 7)]
        for reverse in (False, True):
            with self.subTest(reverse=reverse):
                points = expected[::-1] if reverse else expected
                lt = LineIterator(*points[0], *points[-1], 0.5)
                for point in points:
                    self.assertTrue(lt.isValid())
                    self.assertEqual((lt.getX(), lt.getY()), point)
                    lt.advance()
                self.assertFalse(lt.isValid())

    def test_steep_line_endpoint(self):
        # Include the endpoint when the step does not divide the span or exceeds it.
        cases = [
            (2, [(2, 2), (2.4, 4), (2.8, 6), (3, 7)]),
            (2, [(3, 7), (2.6, 5), (2.2, 3), (2, 2)]),
            (10, [(2, 2), (3, 7)]),
            (10, [(3, 7), (2, 2)]),
        ]
        for step, points in cases:
            with self.subTest(step=step, points=points):
                lt = LineIterator(*points[0], *points[-1], step)
                for point in points:
                    self.assertTrue(lt.isValid())
                    self.assertEqual((lt.getX(), lt.getY()), point)
                    lt.advance()
                self.assertFalse(lt.isValid())

    def test_vertical_line_constant_x(self):
        # Advancing a vertical line must leave its X coordinate unchanged.
        for ys in ([0, 1, 2], [2, 1, 0]):
            with self.subTest(ys=ys):
                lt = LineIterator(0.123456, ys[0], 0.123456, ys[-1])
                for y in ys:
                    self.assertTrue(lt.isValid())
                    self.assertEqual((lt.getX(), lt.getY()), (0.123456, y))
                    lt.advance()
                self.assertFalse(lt.isValid())

    def test_precise_endpoints(self):
        # Clamping must preserve endpoints that cannot be rounded to five decimals.
        for end in [(1, 2.000001), (1, 1.999999), (0.123456, 2.000001)]:
            for reverse in (False, True):
                for transpose in (False, True):
                    with self.subTest(end=end, reverse=reverse, transpose=transpose):
                        start, finish = (0, 0), end
                        if reverse:
                            start, finish = finish, start
                        if transpose:
                            start, finish = start[::-1], finish[::-1]
                        lt = LineIterator(*start, *finish, 1)
                        points = []
                        for _ in range(6):
                            if not lt.isValid():
                                break
                            points.append((lt.getX(), lt.getY()))
                            lt.advance()
                        self.assertFalse(lt.isValid())
                        self.assertEqual(points[0], start)
                        self.assertEqual(points[-1], finish)
                        self.assertEqual(points.count(finish), 1)
                        lt.advance()
                        self.assertFalse(lt.isValid())
                        self.assertEqual((lt.getX(), lt.getY()), finish)

    def test_small_steps_make_progress(self):
        # Rounding must not keep the iterator on the same dominant-axis coordinate.
        for reverse in (False, True):
            for transpose in (False, True):
                with self.subTest(reverse=reverse, transpose=transpose):
                    start, end = (0, 0), (0.000001, 0.000003)
                    if reverse:
                        start, end = end, start
                    if transpose:
                        start, end = start[::-1], end[::-1]
                    lt = LineIterator(*start, *end, 0.000001)
                    for _ in range(6):
                        if not lt.isValid():
                            break
                        previous = (lt.getX(), lt.getY())
                        lt.advance()
                        if previous != end:
                            self.assertNotEqual((lt.getX(), lt.getY()), previous)
                    self.assertFalse(lt.isValid())
                    self.assertEqual((lt.getX(), lt.getY()), end)

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
