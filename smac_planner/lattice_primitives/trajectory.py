from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class TrajectoryParameters:
    radius: float
    x_offset: float
    y_offset: float
    end_point: np.array
    start_angle: float
    end_angle: float
    left_turn: bool

    start_to_arc_distance: float
    arc_to_end_distance: float

@dataclass(frozen=True)
class TrajectoryPath:
    xs: np.array
    ys: np.array
    yaws: np.array
    
    def __add__(self, rhs):
        xs = np.concatenate((self.xs, rhs.xs))
        ys = np.concatenate((self.ys, rhs.ys))
        yaws = np.concatenate((self.yaws, rhs.yaws))

        return TrajectoryPath(xs, ys, yaws)

@dataclass(frozen=True)
class Trajectory:
    path: TrajectoryPath
    parameters: TrajectoryParameters