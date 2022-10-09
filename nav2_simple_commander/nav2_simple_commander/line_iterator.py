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

It provides the ability to iterate through the points of a line.
"""


from cmath import sqrt


class LineIterator():
    def __init__(self, x0, y0, x1, y1, step_size=1.0):
        """Initializer for LineIterator.

        Args:
            x0 (Float): Abscissa of the initial point
            y0 (Float): Ordinate of the initial point
            x1 (Float): Abscissa of the final point
            y1 (Float): Ordinate of the final point
            step_size (Float, optional): Increments Resolution. Defaults to 1.
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
            if (self.b_ < 0):
                self.equation_ = "y = " + str(self.m_) + "*x " + str(self.b_)
            else:
                self.equation_ = "y = " + str(self.m_) + "*x + " + str(self.b_)
        elif x1 == x0 and y1 != y0:
            self.valid_ = True
            self.equation_ = "x = " + str(x1)
        elif y1 == y1 and x1 != x0:
            self.valid_ = True
            self.m_ = (y1-y0)/(x1-x0)
            self.b_ = y1 - (self.m_*x1)
            self.equation_ = "y = " + str(y1)
        else:
            self.valid_ = False
            self.equation_ = "Invalid"
            raise ValueError(
                "Line has zero length (All 4 points have same coordinates)")

    def isValid(self):
        """Returns True if the line is valid
        Returns:
            Bool: Flag if live is valid (true) or not (false)
        """
        return self.valid_

    def advance(self):
        """Advances to the next point in the line."""
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
        """Returns the abscissa of the current point.

        Returns:
            Float: abscissa of current point
        """
        return self.x_

    def getY(self):
        """Returns the ordinate of the current point.

        Returns:
            Float: ordinate of current point
        """
        return self.y_

    def getX0(self):
        """Returns the abscissa of the initial point.

        Returns:
            Float: abscissa of initial point
        """
        return self.x0_

    def getY0(self):
        """Returns the ordinate of the intial point.

        Returns:
            Float: ordinate of intial point
        """
        return self.y0_

    def getX1(self):
        """Returns the abscissa of the final point.

        Returns:
            Float: abscissa of final point
        """
        return self.x1_

    def getY1(self):
        """Returns the ordinate of the final point.

        Returns:
            Float: ordinate of final point
        """
        return self.y1_

    def get_line_length(self):
        """Returns the length of the line.

        Returns:
            Float: Line Length
        """
        return sqrt(pow(self.x1_ - self.x0_, 2) + pow(self.y1_ - self.y0_, 2))

    def get_line_equation(self):
        """Returns the equation of the line as a string.

        Returns:
            String: Line's Equation
        """
        return self.equation_

    def get_curr_point_str(self):
        """Returns the coordinates of the current point as string.

        Returns:
            String: Current Coordinates
        """
        return "X: " + str(self.x_) + "   Y: " + str(self.y_)

    def get_curr_point(self):
        """Returns the coordinates of the current point as list [X,Y].

        Returns:
            List: Current Coordinates as float [X,Y]
        """
        return [self.x_, self.y_]

    def clamp(self, n, min_n, max_n):
        """Class Helper Function: Clamps n to be between min_n and max_n

        Args:
            n (Float): input value
            min_n (Float): minimum value
            max_n (Float): maximum value

        Returns:
            Float: input value clamped between given min and max
        """
        if n < min_n:
            return min_n
        elif n > max_n:
            return max_n
        else:
            return n
