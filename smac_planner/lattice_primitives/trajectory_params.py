from dataclasses import dataclass
import numpy as np

@dataclass
class TrajectoryParameters:
    radius: float
    x_offset: float
    y_offset: float
    end_point: np.array
    start_angle: float
    end_angle: float

    start_to_arc_distance: float
    arc_to_end_distance: float

