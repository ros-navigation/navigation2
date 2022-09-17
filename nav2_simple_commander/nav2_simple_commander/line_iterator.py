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

        self.delta_x_ = abs(self.x1_ - self.x0_)
        self.delta_y_ = abs(self.y1_ - self.y0_)
        self.curpixel_ = 0
        self.xinc1_, self.xinc2_, self.yinc1_, self.yinc2_ = 0, 0, 0, 0
        self.num_, self.den_ = 0, 0
        self.numadd_ = 0

        if (self.x1_ >= x0):
            self.xinc1_ = 1
            self.xinc2_ = 1
        else:
            self.xinc1_ = -1
            self.xinc2_ = -1

        if (y1 >= y0):
            self.yinc1_ = 1
            self.yinc2_ = 1
        else:
            self.yinc1_ = -1
            self.yinc2_ = -1

        if (self.delta_x_ >= self.delta_y_):
            self.xinc1_ = 0
            self.yinc2_ = 0
            self.den_ = self.delta_x_
            self.num_ = self.delta_x_ / 2
            self.numadd_ = self.delta_y_
            self.numpixels_ = self.delta_x_
        else:
            self.xinc2_ = 0
            self.yinc1_ = 0
            self.den_ = self.delta_y_
            self.num_ = self.delta_y_ / 2
            self.numadd_ = self.delta_x_
            self.numpixels_ = self.delta_y_

    def isValid(self):
        """Checks if the given line is valid"""
        return self.curpixel_ <= self.numpixels_

    def advance(self):
        """Advances to the next point in the line."""
        self.num_ = self.numadd_
        if (self.num_ >= self.den_):
            self.num_ -= self.den_
            self.x_ += self.xinc1_
            self.y_ += self.yinc1_
        self.x_ += self.xinc2_
        self.y_ += self.yinc2_
        self.curpixel_ += 1

    def getX(self):
        """Gets abscissa of the current point."""
        return self.x_

    def getY(self):
        """Gets ordinate of the current point."""
        return self.y_

    def getX0(self):
        """Gets abscissa of the initial point."""
        return self.x0_

    def getY0(self):
        """Gets ordinate of the initial point."""
        return self.y0_

    def getX1(self):
        """Gets abscissa of the final point."""
        return self.x1_

    def getY1(self):
        """Gets ordinate of the final point."""
        return self.y1_
