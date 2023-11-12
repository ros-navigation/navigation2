# Copyright (c) 2021, Matthew Booker
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
# limitations under the License. Reserved.

from dataclasses import dataclass

from helper import angle_difference, normalize_angle

import numpy as np


@dataclass(frozen=True)
class TrajectoryParameters:
    """
    A dataclass that holds the data needed to create the path for a trajectory.

    turning_radius: The radius of the circle used to generate
        the arc of the path
    x_offset: The x coordinate of the circle used to generate
        the arc of the path
    y_offset: They y coordinate of the circle used to generate
        the arc of the path
    end_point: The end coordinate of the path
    start_angle: The starting angle of the path
        - given in radians from -pi to pi where 0 radians is along
            the positive x axis
    end_angle: The end angle of the path
        - given in radians from -pi to pi where 0 radians is along
            the positive x axis
    left_turn: Whether the arc in the path turns to the left
    arc_start_point: Coordinates of the starting position of the arc
    arc_end_point: Coordinates of the ending position of the arc
    """

    turning_radius: float
    x_offset: float
    y_offset: float
    end_point: np.array
    start_angle: float
    end_angle: float
    left_turn: bool

    arc_start_point: float
    arc_end_point: float

    @property
    def arc_length(self):
        """Arc length of the trajectory."""
        return self.turning_radius * angle_difference(
            self.start_angle, self.end_angle, self.left_turn
        )

    @property
    def start_straight_length(self):
        """Length of the straight line from start to arc."""
        return np.linalg.norm(self.arc_start_point)

    @property
    def end_straight_length(self):
        """Length of the straight line from arc to end."""
        return np.linalg.norm(self.end_point - self.arc_end_point)

    @property
    def total_length(self):
        """Total length of trajectory."""
        return self.arc_length + self.start_straight_length + self.end_straight_length

    @staticmethod
    def no_arc(end_point, start_angle, end_angle):
        """Create the parameters for a trajectory with no arc."""
        return TrajectoryParameters(
            turning_radius=0.0,
            x_offset=0.0,
            y_offset=0.0,
            end_point=end_point,
            start_angle=start_angle,
            end_angle=end_angle,
            left_turn=True,
            arc_start_point=end_point,
            arc_end_point=end_point,
        )


@dataclass(frozen=True)
class Path:
    """
    A dataclass that holds the generated poses for a given trajectory.

    xs: X coordinates of poses along trajectory
    ys: Y coordinates of poses along trajectory
    yaws: Yaws of poses along trajectory
    """

    xs: np.array
    ys: np.array
    yaws: np.array

    def __add__(self, rhs):
        """Add two paths together by concatenating them."""
        if self.xs is None:
            return rhs

        xs = np.concatenate((self.xs, rhs.xs))
        ys = np.concatenate((self.ys, rhs.ys))
        yaws = np.concatenate((self.yaws, rhs.yaws))

        return Path(xs, ys, yaws)

    def to_output_format(self):
        """Return the path data in a format suitable for outputting."""
        output_xs = self.xs.round(5)
        output_ys = self.ys.round(5)

        # A bit of a hack but it removes any -0.0
        output_xs = output_xs + 0.0
        output_ys = output_ys + 0.0
        output_yaws = self.yaws + 0.0

        vectorized_normalize_angle = np.vectorize(normalize_angle)
        output_yaws = vectorized_normalize_angle(output_yaws)

        stacked = np.vstack([output_xs, output_ys, output_yaws]).transpose()

        return stacked.tolist()


@dataclass(frozen=True)
class Trajectory:
    """
    A dataclass that holds the path and parameters for a trajectory.

    path: The Path that represents the trajectory
    parameters: The TrajectoryParameters that represent the trajectory
    """

    path: Path
    parameters: TrajectoryParameters
