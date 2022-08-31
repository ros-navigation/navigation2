#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
# Copyright 2022 Stevedan Ogochukwu Omodolor
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
        self.costmap = np.array(occupancy_map.data, dtype=np.int8).reshape(
            self.size_y, self.size_x)

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
