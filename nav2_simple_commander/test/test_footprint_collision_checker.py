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

import unittest

from geometry_msgs.msg import Point32, Polygon
from nav2_msgs.msg import Costmap
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_simple_commander.footprint_collision_checker import FootprintCollisionChecker

LETHAL_OBSTACLE = 254


class TestFootprintCollisionChecker(unittest.TestCase):

    def test_no_costmap(self):
        # Test if a type error raised when costmap is not specified yet
        fcc_ = FootprintCollisionChecker()
        self.assertRaises(ValueError, fcc_.worldToMapValidated, 0.0, 0.0)
        self.assertRaises(ValueError, fcc_.pointCost, 0.0, 0.0)

    def test_pointCost(self):
        # Test if point cost is calculated correctly
        # Create test grid 10 pixels wide by 10 pixels long, at 1 meters per pixel
        # AKA 10 meters x 10 meters
        occupancyGrid_ = Costmap()
        occupancyGrid_.metadata.resolution = 1.0
        occupancyGrid_.metadata.size_x = 10
        occupancyGrid_.metadata.size_y = 10
        occupancyGrid_.metadata.origin.position.x = 0.0
        occupancyGrid_.metadata.origin.position.y = 0.0
        map_data = [0] * 10 * 10
        occupancyGrid_.data = map_data
        costmap_ = PyCostmap2D(occupancyGrid_)
        fcc_ = FootprintCollisionChecker()
        fcc_.setCostmap(costmap_)
        self.assertEqual(fcc_.pointCost(1, 1), 0)
        self.assertRaises(IndexError, fcc_.pointCost, 11, 11)

    def test_worldToMapValidated(self):
        # Test if worldToMap conversion is calculated correctly
        # Create test grid 10 pixels wide by 10 pixels long, at 1 meters per pixel
        # AKA 10 meters x 10 meters
        # Map origin is at (5,5) of world coordinates
        occupancyGrid_ = Costmap()
        occupancyGrid_.metadata.resolution = 1.0
        occupancyGrid_.metadata.size_x = 10
        occupancyGrid_.metadata.size_y = 10
        occupancyGrid_.metadata.origin.position.x = 5.0
        occupancyGrid_.metadata.origin.position.y = 5.0
        map_data = [0] * 10 * 10
        occupancyGrid_.data = map_data
        costmap_ = PyCostmap2D(occupancyGrid_)
        fcc_ = FootprintCollisionChecker()
        fcc_.setCostmap(costmap_)
        self.assertEqual(fcc_.worldToMapValidated(0, 5), (None, None))
        self.assertEqual(fcc_.worldToMapValidated(5, 0), (None, None))
        self.assertEqual(fcc_.worldToMapValidated(5, 5), (0, 0))
        self.assertEqual(fcc_.worldToMapValidated(14, 14), (9, 9))
        self.assertEqual(fcc_.worldToMapValidated(15, 14), (None, None))

    def test_lineCost(self):
        # Test if line cost is calculated correctly
        # Create test grid 10 pixels wide by 10 pixels long, at 1 meters per pixel
        # AKA 10 meters x 10 meters
        occupancyGrid_ = Costmap()
        occupancyGrid_.metadata.resolution = 1.0
        occupancyGrid_.metadata.size_x = 10
        occupancyGrid_.metadata.size_y = 10
        occupancyGrid_.metadata.origin.position.x = 0.0
        occupancyGrid_.metadata.origin.position.y = 0.0
        map_data = [0] * 10 * 10
        occupancyGrid_.data = map_data
        costmap_ = PyCostmap2D(occupancyGrid_)
        fcc_ = FootprintCollisionChecker()
        fcc_.setCostmap(costmap_)
        self.assertRaises(IndexError, fcc_.lineCost, 0, 15, 0, 9, 1)
        self.assertEqual(fcc_.lineCost(0, 9, 0, 9, 1), 0.0)

    def test_lineCost_steep_edges(self):
        # A steep edge must not jump over a lethal cell between its endpoints.
        cases = [
            ((2, 2), (3, 12), (2, 4)),
            ((3, 12), (2, 2), (2, 4)),
            ((3, 2), (2, 12), (2, 8)),
            ((2, 12), (3, 2), (2, 8)),
        ]
        for start, end, obstacle in cases:
            for transpose in (False, True):
                with self.subTest(start=start, end=end, transpose=transpose):
                    x0, y0 = start[::-1] if transpose else start
                    x1, y1 = end[::-1] if transpose else end
                    ox, oy = obstacle[::-1] if transpose else obstacle
                    message = Costmap()
                    message.metadata.resolution = 0.05
                    message.metadata.size_x = 20
                    message.metadata.size_y = 20
                    data = [0] * 400
                    data[oy * 20 + ox] = LETHAL_OBSTACLE
                    message.data = data
                    checker = FootprintCollisionChecker()
                    checker.setCostmap(PyCostmap2D(message))
                    self.assertEqual(checker.lineCost(x0, x1, y0, y1), LETHAL_OBSTACLE)

    def test_footprintCost_steep_edge(self):
        # A valid rectangle with a steep side crosses lethal cell (2, 4).
        message = Costmap()
        message.metadata.resolution = 0.05
        message.metadata.size_x = 20
        message.metadata.size_y = 20
        data = [0] * 400
        data[4 * 20 + 2] = LETHAL_OBSTACLE
        message.data = data
        checker = FootprintCollisionChecker()
        checker.setCostmap(PyCostmap2D(message))
        points = [
            Point32(x=x * 0.05 + 0.025, y=y * 0.05 + 0.025)
            for x, y in [(2, 2), (3, 12), (13, 11), (12, 1)]
        ]
        for vertices in (points, points[::-1]):
            with self.subTest(vertices=vertices):
                self.assertEqual(
                    checker.footprintCost(Polygon(points=vertices)), LETHAL_OBSTACLE)

    def test_footprintCost(self):
        # Test if footprint cost is calculated correctly
        # Create test grid 10 pixels wide by 10 pixels long, at 1 meters per pixel
        # AKA 10 meters x 10 meters
        occupancyGrid_ = Costmap()
        occupancyGrid_.metadata.resolution = 1.0
        occupancyGrid_.metadata.size_x = 10
        occupancyGrid_.metadata.size_y = 10
        occupancyGrid_.metadata.origin.position.x = 0.0
        occupancyGrid_.metadata.origin.position.y = 0.0
        map_data = [0] * 10 * 10
        occupancyGrid_.data = map_data
        costmap_ = PyCostmap2D(occupancyGrid_)
        fcc_ = FootprintCollisionChecker()
        fcc_.setCostmap(costmap_)
        # Create square footprint 1m x 1m
        footprint = Polygon()
        points = []
        point = Point32()
        point.x = 0.0
        point.y = 0.0
        point.z = 0.0
        points.append(point)
        point = Point32()
        point.x = 1.0
        point.y = 1.0
        point.z = 0.0
        points.append(point)
        point = Point32()
        point.x = 1.0
        point.y = 0.0
        point.z = 0.0
        points.append(point)
        point = Point32()
        point.x = 0.0
        point.y = 1.0
        point.z = 0.0
        points.append(point)
        footprint.points = points
        self.assertEqual(fcc_.footprintCost(footprint), 0.0)
        # Test none-zero cost
        # Create in the map center a full box of cost value 100
        for i in range(24, 28):
            map_data[i] = 100
            map_data[i + 10] = 100
            map_data[i + 20] = 100
            map_data[i + 30] = 100
            map_data[i + 40] = 100
        occupancyGrid_.data = map_data
        costmap_ = PyCostmap2D(occupancyGrid_)
        fcc_ = FootprintCollisionChecker()
        fcc_.setCostmap(costmap_)
        self.assertEqual(fcc_.footprintCostAtPose(4.0, 4.0, 0.0, footprint), 100)
        # Append a point that is outside the map
        point = Point32()
        point.x = 30.0
        point.y = 5.0
        point.z = 3.0
        points.append(point)
        footprint.points = points
        self.assertEqual(fcc_.footprintCost(footprint), LETHAL_OBSTACLE)


if __name__ == '__main__':
    unittest.main()
