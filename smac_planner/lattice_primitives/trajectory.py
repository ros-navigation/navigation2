from dataclasses import dataclass

import numpy as np

from helper import angle_difference


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

    @property
    def arc_length(self):
        return 2 * np.pi * self.radius * np.rad2deg(angle_difference(self.start_angle, self.end_angle)) / 360

    @property
    def total_length(self):
        return self.arc_length + self.start_to_arc_distance + self.arc_to_end_distance

    @staticmethod
    def no_arc(end_point, start_angle, end_angle, left_turn):
        line_distance = np.linalg.norm(end_point)

        return TrajectoryParameters(radius=0, x_offset=0, y_offset=0, end_point=end_point,
                                    start_angle=start_angle, end_angle=end_angle, left_turn=left_turn,
                                    start_to_arc_distance=line_distance, arc_to_end_distance=0)


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
