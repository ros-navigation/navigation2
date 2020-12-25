import math
import numpy as np

class State:
    
    def __init__(self, x, y, yaw, velocity = 0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.velocity = velocity

    def to_numpy(self):
        return np.array([self.x, self.y, self.yaw], dtype=float)
