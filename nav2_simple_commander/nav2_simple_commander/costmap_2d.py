#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
# Copyright 2022 Stevedan Ogochukwu Omodolor
# Copyright 2022 Jaehun Jackson Kim
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

import numpy as np


class PyCostmap2D:
    """
    PyCostmap2D.

    Costmap Python3 API for OccupancyGrids to populate from published messages
    """

    def __init__(self, occupancy_map):
        self.size_x = occupancy_map.info.width
        self.size_y = occupancy_map.info.height
        self.resolution = occupancy_map.info.resolution
        self.origin_x = occupancy_map.info.origin.position.x
        self.origin_y = occupancy_map.info.origin.position.y
        self.global_frame_id = occupancy_map.header.frame_id
        self.costmap_timestamp = occupancy_map.header.stamp
        # Extract costmap
        self.costmap = np.array(occupancy_map.data, dtype=np.int8)

    def getSizeInCellsX(self):
        """Get map width in cells."""
        return self.size_x

    def getSizeInCellsY(self):
        """Get map height in cells."""
        return self.size_y

    def getSizeInMetersX(self):
        """Get x axis map size in meters."""
        return (self.size_x - 1 + 0.5) * self.resolution_

    def getSizeInMetersY(self):
        """Get y axis map size in meters."""
        return (self.size_y - 1 + 0.5) * self.resolution_

    def getOriginX(self):
        """Get the origin x axis of the map [m]."""
        return self.origin_x

    def getOriginY(self):
        """Get the origin y axis of the map [m]."""
        return self.origin_y

    def getResolution(self):
        """Get map resolution [m/cell]."""
        return self.resolution

    def getGlobalFrameID(self):
        """Get global frame_id."""
        return self.global_frame_id

    def getCostmapTimestamp(self):
        """Get costmap timestamp."""
        return self.costmap_timestamp

    def getCost(self, _index: int = None, _mx: int = None, _my: int = None) -> np.int8:
        """
        Get the cost of a cell in the costmap
            Args:
                _index (int): index of cell to get cost
                _mx (int): x position of cell to get cost
                _my (int): y position of cell to get cost
            Returns:
                np.int8: cost of a cell
        """
        if _index != None:
            return self.costmap[_index]
        else:
            return self.costmap[self.getIndex(_mx, _my)]

    def setCost(self, _mx: int, _my: int, _cost: np.int8) -> None:
        """
        Set the cost of a cell in the costmap
            Args:
                _cost (int): The cost to set the cell to
                _mx (int): The x coordinate of the cell
                _my (int): The y coordinate of the cell
            Returns:
                None
        """
        self.costmap[self.getIndex(_mx, _my)] = _cost

    def mapToWorld(self, _mx: int, _my: int) -> tuple[float, float]:
        """
        Set the cost of a cell in the costmap
            Args:
                _mx (int): The x coordinate of the cell
                _my (int): The y coordinate of the cell
            Returns:
                tuple[float, float]: _wx, _wy
                _wx (float): The x world coordinate
                _wy (float): The y world coordinate
        """
        _wx = self.origin_x + (_mx + 0.5) * self.resolution
        _wy = self.origin_y + (_my + 0.5) * self.resolution
        return (_wx, _wy)

    def worldToMap(self, _wx: float, _wy: float) -> tuple[int, int]:
        """
        Set the cost of a cell in the costmap
            Args:
                _wx (int): The x world coordinate
                _wy (int): The y world coordinate
            Returns:
                tuple[int, int]: _mx, _my
                _mx (int): The x coordinate of the cell
                _my (int): The y coordinate of the cell
        """
        _mx = int((_wx - self.origin_x) // self.resolution)
        _my = int((_wy - self.origin_y) // self.resolution)
        return (_mx, _my)

    def getIndex(self, _mx: int, _my: int) -> int:
        """
        Set the cost of a cell in the costmap
            Args:
                _mx (int): The x coordinate of the cell
                _my (int): The y coordinate of the cell
            Returns:
                int: The index of the cell
        """
        return _my * self.size_x + _mx
    
