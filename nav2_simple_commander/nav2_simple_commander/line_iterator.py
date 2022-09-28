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

    def __init__(self, x0, y0, x1, y1):
        """
        Initializer for LineIterator.

        Initialize instance variables with parameter occupancy_map.
            Args:
                x0: Abscissa of the initial point
                y0: Ordinate of the initial point
                x1: Abscissa of the final point
                y1: Ordinate of the final point
            Returns:
                None
        """
        self.x0_ = x0
        self.y0_ = x0
        self.x1_ = x1
        self.y1_ = y1
        self.x_ = x0
        self.y_ = y0

        if x1 != x0 and y1 != y0:
            self.valid_ = True
            self.distance_ = abs(x1-x0)
            self.m_ = (y1-y0)/(x1-x0)
            self.b_ = y1 - (self.m_*x1)
            if (self.b_ < 0):
                self.equation_ = "y = " + str(self.m_) + "*x " + str(self.b_)
            else:
                self.equation_ = "y = " + str(self.m_) + "*x + " + str(self.b_)
        elif x1 == x0 and y1 != y0:
            self.valid_ = True
            self.distance_ = abs(y1-y0)
            self.equation_ = "x = " + str(x1)
        elif y1 == y1 and x1 != x0:
            self.valid_ = True
            self.distance_ = abs(x1-x0)
            self.equation_ = "y = " + str(y1)
        else:
            self.valid_ = False
            self.equation_ = "Invalid"
        self.step_ = self.distance_/1000

    def isValid(self):
        """Returns True if the line is valid"""
        return self.valid_

    def advance(self):
        """Advances to the next point in the line."""
        if self.x1_ > self.x0_:
            if self.x_ < self.x1_:
                self.x_ = round(self.x_ + self.step_, 5)
                self.y_ = round(self.m_ * self.x_ + self.b_, 5)
            else:
                self.valid_ = False
        elif self.x1_ < self.x0_:
            if self.x_ > self.x1_:
                self.x_ = round(self.x_ - self.step_, 5)
                self.y_ = round(self.m_ * self.x_ + self.b_, 5)
            else:
                self.valid_ = False
        else:
            if self.y1_ > self.y0_:
                if self.y_ < self.y1_:
                    self.y_ = round(self.y_ + self.step_, 5)
                else:
                    self.valid_ = False
            elif self.y1_ < self.y0_:
                if self.y_ > self.y1_:
                    self.y_ = round(self.y_ - self.step_, 5)
                else:
                    self.valid_ = False
            else:
                self.valid_ = False

    def getX(self):
        """Returns the abscissa of the current point."""
        return self.x_

    def getY(self):
        """Returns the ordinate of the current point."""
        return self.y_

    def getX0(self):
        """Returns the abscissa of the initial point."""
        return self.x0_

    def getY0(self):
        """Returns the ordinate of the initial point."""
        return self.y0_

    def getX1(self):
        """Returns the abscissa of the final point."""
        return self.x1_

    def getY1(self):
        """Returns the ordinate of the final point."""
        return self.y1_

    def get_line_length(self):
        """Returns the length of the line."""
        return sqrt(pow(self.x1_ - self.x0_, 2) + pow(self.y1_ - self.y0_, 2))

    def get_line_equation(self):
        """Returns the equation of the line as a string."""
        return self.equation_

    def get_curr_point_str(self):
        """Returns the coordinates of the current point as string."""
        return "X: " + str(self.x_) + "   Y: " + str(self.y_)

    def get_curr_point(self):
        """Returns the coordinates of the current point as list [X,Y]."""
        return [self.x_, self.y_]