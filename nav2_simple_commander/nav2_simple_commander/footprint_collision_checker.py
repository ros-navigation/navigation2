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

from math import cos, sin
from line_iterator import LineIterator
from costmap_2d import PyCostmap2D

NO_INFORMATION = 255
LETHAL_OBSTACLE = 254
INSCRIBED_INFLATED_OBSTACLE = 253
MAX_NON_OBSTACLE = 252
FREE_SPACE = 0


class FootprintCollisionChecker():
    def __init__(self, costmap: PyCostmap2D):
        self.costmap_ = costmap

    def footprintCost(self, footprint):
        if not self.costmap_.worldToMapCheck(footprint[0].x, footprint[0].y):
            return LETHAL_OBSTACLE

        x0, y0 = self.costmap_.worldToMap(footprint[0].x, footprint[0].y)
        xstart = x0
        ystart = y0

        for i in range(footprint.size() - 1):
            if not self.costmap_.worldToMapCheck(
                    footprint[i + 1].x, footprint[i + 1].y):
                return LETHAL_OBSTACLE

            x1, y1 = self.costmap_.worldToMap(
                footprint[i + 1].x, footprint[i + 1].y)

            footprint_cost = self.lineCost(x0, x1, y0, y1)
            x0 = x1
            y0 = y1

            if footprint_cost == LETHAL_OBSTACLE:
                return footprint_cost

        return max(self.lineCost(xstart, x1, ystart, y1), footprint_cost)

    def lineCost(self, x0, x1, y0, y1):
        line_cost = 0.0
        point_cost = -1.0
        line_iterator = LineIterator(x0, y0, x1, y1, 0.1)

        while line_iterator.isValid():
            point_cost = self.costmap_.getCostXY(
                line_iterator.getX(), line_iterator.getY())

            if point_cost == LETHAL_OBSTACLE:
                return point_cost

            if line_cost < point_cost:
                line_cost = point_cost

        return line_cost

    def setCostmap(self, costmap: PyCostmap2D):
        self.costmap_ = costmap

