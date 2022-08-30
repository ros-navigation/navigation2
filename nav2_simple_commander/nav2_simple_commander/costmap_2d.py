#! /usr/bin/env python3
from nav_msgs.msg import OccupancyGrid
import numpy as np

class PyCostmap2D:
    def __init__(self, map):
        self.size_x_ = map.info.width;
        self.size_y_ = map.info.height;
        self.resolution_ = map.info.resolution;
        self.origin_x_ = map.info.origin.position.x;
        self.origin_y_ = map.info.origin.position.y;
        self.costmap_ = np.array(msg.data, dtype=np.int8).reshape(self.size_y_, self.size_x_)
    def getSizeinMetersX(self):
        ...
    def getSizeInCellsY(self):
        ...
    def getSizeInMetersX(self):
        ...
    def getOriginX(self):
        ...
    def getOriginY(self):
        ...
    def getResolution(self):
        ...
    def getGlobalFrameID(self):
        ...
    def getCostmapTimestamp(self):
        ...
