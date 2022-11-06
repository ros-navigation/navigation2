#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
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

"""
This is a Python3 API for a line iterator.

It provides the ability to iterate
through the points of a line.
"""

from cmath import sqrt


class LineIterator():
    """
    LineIterator.

    LineIterator Python3 API for iterating along the points of a given line
    """

    def __init__(self, x0, y0, x1, y1, step_size=1.0):
        """
        Initialize the LineIterator.

        Args
        ----
            x0 (float): Abscissa of the initial point
            y0 (float): Ordinate of the initial point
            x1 (float): Abscissa of the final point
            y1 (float): Ordinate of the final point
            step_size (float): Optional, Increments' resolution, defaults to 1

        Raises
        ------
            TypeError: When one (or more) of the inputs is not a number
            ValueError: When step_size is not a positive number

        """
        if type(x0) not in [int, float]:
            raise TypeError("x0 must be a number (int or float)")

        if type(y0) not in [int, float]:
            raise TypeError("y0 must be a number (int or float)")

        if type(x1) not in [int, float]:
            raise TypeError("x1 must be a number (int or float)")

        if type(y1) not in [int, float]:
            raise TypeError("y1 must be a number (int or float)")

        if type(step_size) not in [int, float]:
            raise TypeError("step_size must be a number (int or float)")

        if step_size <= 0:
            raise ValueError("step_size must be a positive number")

        self.x0_ = x0
        self.y0_ = y0
        self.x1_ = x1
        self.y1_ = y1
        self.x_ = x0
        self.y_ = y0
        self.step_size_ = step_size

        if x1 != x0 and y1 != y0:
            self.valid_ = True
            self.m_ = (y1-y0)/(x1-x0)
            self.b_ = y1 - (self.m_*x1)
        elif x1 == x0 and y1 != y0:
            self.valid_ = True
        elif y1 == y1 and x1 != x0:
            self.valid_ = True
            self.m_ = (y1-y0)/(x1-x0)
            self.b_ = y1 - (self.m_*x1)
        else:
            self.valid_ = False
            raise ValueError(
                "Line has zero length (All 4 points have same coordinates)")

    def isValid(self):
        """Check if line is valid."""
        return self.valid_

    def advance(self):
        """Advance to the next point in the line."""
        if self.x1_ > self.x0_:
            if self.x_ < self.x1_:
                self.x_ = round(self.clamp(
                    self.x_ + self.step_size_, self.x0_, self.x1_), 5)
                self.y_ = round(self.m_ * self.x_ + self.b_, 5)
            else:
                self.valid_ = False
        elif self.x1_ < self.x0_:
            if self.x_ > self.x1_:
                self.x_ = round(self.clamp(
                    self.x_ - self.step_size_, self.x1_, self.x0_), 5)
                self.y_ = round(self.m_ * self.x_ + self.b_, 5)
            else:
                self.valid_ = False
        else:
            if self.y1_ > self.y0_:
                if self.y_ < self.y1_:
                    self.y_ = round(self.clamp(
                        self.y_ + self.step_size_, self.y0_, self.y1_), 5)
                else:
                    self.valid_ = False
            elif self.y1_ < self.y0_:
                if self.y_ > self.y1_:
                    self.y_ = round(self.clamp(
                        self.y_ - self.step_size_, self.y1_, self.y0_), 5)
                else:
                    self.valid_ = False
            else:
                self.valid_ = False

    def getX(self):
        """Get the abscissa of the current point."""
        return self.x_

    def getY(self):
        """Get the ordinate of the current point."""
        return self.y_

    def getX0(self):
        """Get the abscissa of the initial point."""
        return self.x0_

    def getY0(self):
        """Get the ordinate of the intial point."""
        return self.y0_

    def getX1(self):
        """Get the abscissa of the final point."""
        return self.x1_

    def getY1(self):
        """Get the ordinate of the final point."""
        return self.y1_

    def get_line_length(self):
        """Get the length of the line."""
        return sqrt(pow(self.x1_ - self.x0_, 2) + pow(self.y1_ - self.y0_, 2))

    def clamp(self, n, min_n, max_n):
        """
        Clamp n to be between min_n and max_n.

        Args
        ----
            n (float): input value
            min_n (float): minimum value
            max_n (float): maximum value

        Returns
        -------
            n (float): input value clamped between given min and max

        """
        if n < min_n:
            return min_n
        elif n > max_n:
            return max_n
        else:
            return n
