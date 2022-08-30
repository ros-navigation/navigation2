#! /usr/bin/env python3
from nav_msgs.msg import OccupancyGrid
import numpy as np

class PyCostmap2D:
    def __init__(self, map):
        self.size_x_ = map.info.width;
        self.size_y_ = map.info.height;
        self.resolution_ = map.info.resolution;
        self.origin_x_ = map.info.origin.position.x
        self.origin_y_ = map.info.origin.position.y
        self.global_frame_id_ = map.header.frame_id
        self.costmap_timestamp_= map.header.stamp
        # Extract costmap
        self.costmap_ = np.array(map.data, dtype=np.int8).reshape(self.size_y_, self.size_x_)
    def getSizeInCellsX(self):
        return self.size_x_
    def getSizeInCellsY(self):
        return self.size_y_
    def getSizeInMetersX(self):
        return (self.size_x_ - 1 + 0.5) * self.resolution_
    def getSizeInMetersY(self):
        return (self.size_y_ - 1 + 0.5) * self.resolution_
    def getOriginX(self):
        return self.origin_x_
    def getOriginY(self):
        return self.origin_y_
    def getResolution(self):
        return self.resolution_
    def getGlobalFrameID(self):
        return self.global_frame_id_
    def getCostmapTimestamp(self):
        return self.costmap_timestamp_
