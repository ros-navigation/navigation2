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
from nav2_simple_commander.costmap_2d import PyCostmap2D
from nav2_simple_commander.footprint_collision_checker import FootprintCollisionChecker
from nav_msgs.msg import OccupancyGrid

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
        occupancyGrid_ = OccupancyGrid()
        occupancyGrid_.info.resolution = 1.0
        occupancyGrid_.info.width = 10
        occupancyGrid_.info.height = 10
        occupancyGrid_.info.origin.position.x = 0.0
        occupancyGrid_.info.origin.position.y = 0.0
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
        occupancyGrid_ = OccupancyGrid()
        occupancyGrid_.info.resolution = 1.0
        occupancyGrid_.info.width = 10
        occupancyGrid_.info.height = 10
        occupancyGrid_.info.origin.position.x = 5.0
        occupancyGrid_.info.origin.position.y = 5.0
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
        occupancyGrid_ = OccupancyGrid()
        occupancyGrid_.info.resolution = 1.0
        occupancyGrid_.info.width = 10
        occupancyGrid_.info.height = 10
        occupancyGrid_.info.origin.position.x = 0.0
        occupancyGrid_.info.origin.position.y = 0.0
        map_data = [0] * 10 * 10
        occupancyGrid_.data = map_data
        costmap_ = PyCostmap2D(occupancyGrid_)
        fcc_ = FootprintCollisionChecker()
        fcc_.setCostmap(costmap_)
        self.assertRaises(IndexError, fcc_.lineCost, 0, 15, 0, 9, 1)
        self.assertEqual(fcc_.lineCost(0, 9, 0, 9, 1), 0.0)

    def test_footprintCost(self):
        # Test if footprint cost is calculated correctly
        # Create test grid 10 pixels wide by 10 pixels long, at 1 meters per pixel
        # AKA 10 meters x 10 meters
        occupancyGrid_ = OccupancyGrid()
        occupancyGrid_.info.resolution = 1.0
        occupancyGrid_.info.width = 10
        occupancyGrid_.info.height = 10
        occupancyGrid_.info.origin.position.x = 0.0
        occupancyGrid_.info.origin.position.y = 0.0
        map_data = [0] * 10 * 10
        occupancyGrid_.data = map_data
        costmap_ = PyCostmap2D(occupancyGrid_)
        fcc_ = FootprintCollisionChecker()
        fcc_.setCostmap(costmap_)
        # Create square footprint 1m x 1m
        footprint = Polygon()
        point = Point32()
        point.x = 0.0
        point.y = 0.0
        point.z = 0.0
        footprint.points.append(point)
        point = Point32()
        point.x = 1.0
        point.y = 1.0
        point.z = 0.0
        footprint.points.append(point)
        point = Point32()
        point.x = 1.0
        point.y = 0.0
        point.z = 0.0
        footprint.points.append(point)
        point = Point32()
        point.x = 0.0
        point.y = 1.0
        point.z = 0.0
        footprint.points.append(point)
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
        footprint.points.append(point)
        self.assertEqual(fcc_.footprintCost(footprint), LETHAL_OBSTACLE)


if __name__ == '__main__':
    unittest.main()
