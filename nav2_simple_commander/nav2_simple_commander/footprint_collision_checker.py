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
This is a Python3 API for a Footprint Collision Checker.

It provides the needed methods to manipulate the coordinates
and calculate the cost of a Footprint
"""

from math import cos, sin

from geometry_msgs.msg import Point32, Polygon
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_simple_commander.line_iterator import LineIterator

NO_INFORMATION = 255
LETHAL_OBSTACLE = 254
INSCRIBED_INFLATED_OBSTACLE = 253
MAX_NON_OBSTACLE = 252
FREE_SPACE = 0


class FootprintCollisionChecker:
    """
    FootprintCollisionChecker.

    FootprintCollisionChecker Class for getting the cost
    and checking the collisions of a Footprint
    """

    def __init__(self):
        """Initialize the FootprintCollisionChecker Object."""
        self.costmap_ = None
        pass

    def footprintCost(self, footprint: Polygon):
        """
        Iterate over all the points in a footprint and check for collision.

        Args
        ----
            footprint (Polygon): The footprint to calculate the collision cost for

        Returns
        -------
            LETHAL_OBSTACLE (int): If collision was found, 254 will be returned
            footprint_cost (float): The maximum cost found in the footprint points

        """
        footprint_cost = 0.0
        x1 = 0.0
        y1 = 0.0

        x0, y0 = self.worldToMapValidated(footprint.points[0].x, footprint.points[0].y)

        if x0 is None or y0 is None:
            return LETHAL_OBSTACLE

        xstart = x0
        ystart = y0

        for i in range(len(footprint.points) - 1):
            x1, y1 = self.worldToMapValidated(
                footprint.points[i + 1].x, footprint.points[i + 1].y
            )

            if x1 is None or y1 is None:
                return LETHAL_OBSTACLE

            footprint_cost = max(float(self.lineCost(x0, x1, y0, y1)), footprint_cost)
            x0 = x1
            y0 = y1

            if footprint_cost == LETHAL_OBSTACLE:
                return footprint_cost

        return max(float(self.lineCost(xstart, x1, ystart, y1)), footprint_cost)

    def lineCost(self, x0, x1, y0, y1, step_size=0.5):
        """
        Iterate over all the points along a line and check for collision.

        Args
        ----
            x0 (float): Abscissa of the initial point in map coordinates
            y0 (float): Ordinate of the initial point in map coordinates
            x1 (float): Abscissa of the final point in map coordinates
            y1 (float): Ordinate of the final point in map coordinates
            step_size (float): Optional, Increments' resolution, defaults to 0.5

        Returns
        -------
            LETHAL_OBSTACLE (int): If collision was found, 254 will be returned
            line_cost (float): The maximum cost found in the line points

        """
        line_cost = 0.0
        point_cost = -1.0
        line_iterator = LineIterator(x0, y0, x1, y1, step_size)

        while line_iterator.isValid():
            point_cost = self.pointCost(
                int(line_iterator.getX()), int(line_iterator.getY())
            )

            if point_cost == LETHAL_OBSTACLE:
                return point_cost

            if line_cost < point_cost:
                line_cost = point_cost

            line_iterator.advance()

        return line_cost

    def worldToMapValidated(self, wx: float, wy: float):
        """
        Get the map coordinate XY using world coordinate XY.

        Args
        ----
            wx (float): world coordinate X
            wy (float): world coordinate Y

        Returns
        -------
            None: if coordinates are invalid
            tuple of int: mx, my (if coordinates are valid)
            mx (int): map coordinate X
            my (int): map coordinate Y

        """
        if self.costmap_ is None:
            raise ValueError(
                'Costmap not specified, use setCostmap to specify the costmap first'
            )
        return self.costmap_.worldToMapValidated(wx, wy)

    def pointCost(self, x: int, y: int):
        """
        Get the cost of a point in the costmap using map coordinates XY.

        Args
        ----
            mx (int): map coordinate X
            my (int): map coordinate Y

        Returns
        -------
            np.uint8: cost of a point

        """
        if self.costmap_ is None:
            raise ValueError(
                'Costmap not specified, use setCostmap to specify the costmap first'
            )
        return self.costmap_.getCostXY(x, y)

    def setCostmap(self, costmap: PyCostmap2D):
        """
        Specify which costmap to use.

        Args
        ----
            costmap (PyCostmap2D): costmap to use in the object's methods

        Returns
        -------
            None

        """
        self.costmap_ = costmap
        return None

    def footprintCostAtPose(self, x: float, y: float, theta: float, footprint: Polygon):
        """
        Get the cost of a footprint at a specific Pose in map coordinates.

        Args
        ----
            x (float): map coordinate X
            y (float): map coordinate Y
            theta (float): absolute rotation angle of the footprint
            footprint (Polygon): the footprint to calculate its cost at the given Pose

        Returns
        -------
            LETHAL_OBSTACLE (int): If collision was found, 254 will be returned
            footprint_cost (float): The maximum cost found in the footprint points

        """
        cos_th = cos(theta)
        sin_th = sin(theta)
        oriented_footprint = Polygon()

        for i in range(len(footprint.points)):
            new_pt = Point32()
            new_pt.x = x + (
                footprint.points[i].x * cos_th - footprint.points[i].y * sin_th
            )
            new_pt.y = y + (
                footprint.points[i].x * sin_th + footprint.points[i].y * cos_th
            )
            oriented_footprint.points.append(new_pt)

        return self.footprintCost(oriented_footprint)
