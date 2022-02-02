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

import logging
from typing import Tuple, Union

import numpy as np

from trajectory import Path, Trajectory, TrajectoryParameters

logger = logging.getLogger(__name__)


class TrajectoryGenerator:
    """Handles all the logic for generating trajectories."""

    def __init__(self, config: dict):
        """Init TrajectoryGenerator using the user supplied config."""
        self.turning_radius = config['turning_radius']

    def _get_arc_point(
        self, trajectory_params: TrajectoryParameters, t: float
    ) -> Tuple[float, float, float]:
        """
        Get point on the arc trajectory using the following parameterization.

            r(t) = <R * cos(t - pi/2) + a, R * sin(t - pi/2) + b>

            R = radius
            a = x offset
            b = y offset

        Args
        ----
        trajectory_params: TrajectoryParameters
            The parameters that describe the arc to create
        t: float
            A value between 0 - 1 that denotes where along the arc
            to calculate the point

        Returns
        -------
        x: float
            x coordinate of generated point
        y: float
            y coordinate of generated point
        yaw: float
            angle of tangent line to arc at point (x,y)

        """
        start_angle = trajectory_params.start_angle

        arc_dist = t * trajectory_params.arc_length
        angle_step = arc_dist / trajectory_params.turning_radius

        if trajectory_params.left_turn:
            # Calculate points using
            # r(t) = <R * cos(t - pi/2) + a, R * sin(t - pi/2) + b>
            t = start_angle + angle_step
            x = (
                trajectory_params.turning_radius * np.cos(t - np.pi / 2)
                + trajectory_params.x_offset
            )
            y = (
                trajectory_params.turning_radius * np.sin(t - np.pi / 2)
                + trajectory_params.y_offset
            )

            yaw = t

        else:
            # Right turns go the opposite way across the arc, so we
            # need to invert the angles and adjust the parametrization
            start_angle = -start_angle

            # Calculate points using
            # r(t) = <R * -cos(t + pi/2) + a, R * sin(t + pi/2) + b>
            t = start_angle + angle_step
            x = (
                trajectory_params.turning_radius * -np.cos(t + np.pi / 2)
                + trajectory_params.x_offset
            )
            y = (
                trajectory_params.turning_radius * np.sin(t + np.pi / 2)
                + trajectory_params.y_offset
            )

            yaw = -t

        return x, y, yaw

    def _get_line_point(
        self, start_point: np.array, end_point: np.array, t: float
    ) -> Tuple[float, float]:
        """
        Get point on a line segment using the following parameterization.

            r(t) = p + t * (q - p)

            p = start point
            q = end point

        Args
        ----
        start_point: np.array(2,)
            Starting point of the line segment
        end_point: np.array(2,)
            End point of the line segment
        t: float
            A value between 0 - 1 that denotes where along the segment
            to calculate the point

        Returns
        -------
        x: float
            x coordinate of generated point
        y: float
            y coordinate of generated point

        """
        return start_point + t * (end_point - start_point)

    def _create_path(
        self, trajectory_params: TrajectoryParameters, primitive_resolution: float
    ) -> Path:
        """
        Create the full trajectory path from the given trajectory parameters.

        Args
        ----
        trajectory_params: TrajectoryParameters
            The parameters that describe the trajectory to create
        primitive_resolution: float
            The desired distance between sampled points along the line.
            This value is not strictly adhered to as the path may not
            be neatly divisible, however the spacing will be as close
            as possible.

        Returns
        -------
        TrajectoryPath
            The trajectory path described by the trajectory parameters

        """
        number_of_steps = np.round(
            trajectory_params.total_length / primitive_resolution
        ).astype(int)
        t_step = 1 / number_of_steps

        start_to_arc_dist = np.linalg.norm(trajectory_params.arc_start_point)

        transition_points = [
            start_to_arc_dist / trajectory_params.total_length,
            (start_to_arc_dist + trajectory_params.arc_length)
            / trajectory_params.total_length,
        ]

        cur_t = t_step

        xs = []
        ys = []
        yaws = []

        for i in range(1, number_of_steps + 1):

            # This prevents cur_t going over 1 due to rounding issues in t_step
            cur_t = min(cur_t, 1)

            # Handle the initial straight line segment
            if cur_t <= transition_points[0]:
                line_t = cur_t / transition_points[0]
                x, y = self._get_line_point(
                    np.array([0, 0]), trajectory_params.arc_start_point, line_t
                )
                yaw = trajectory_params.start_angle

            # Handle the arc
            elif cur_t <= transition_points[1]:
                arc_t = (cur_t - transition_points[0]) / (
                    transition_points[1] - transition_points[0]
                )
                x, y, yaw = self._get_arc_point(trajectory_params, arc_t)

            # Handle the end straight line segment
            else:
                line_t = (cur_t - transition_points[1]) / (1 - transition_points[1])
                x, y = self._get_line_point(
                    trajectory_params.arc_end_point, trajectory_params.end_point, line_t
                )
                yaw = trajectory_params.end_angle

            xs.append(x)
            ys.append(y)
            yaws.append(yaw)

            cur_t += t_step

        # Convert to numpy arrays
        xs = np.array(xs)
        ys = np.array(ys)
        yaws = np.array(yaws)

        # The last point may be slightly off due to rounding issues
        # so we correct the last point to be exactly the end point
        xs[-1], ys[-1] = trajectory_params.end_point
        yaws[-1] = trajectory_params.end_angle

        return Path(xs, ys, yaws)

    def _get_intersection_point(
        self, m1: float, c1: float, m2: float, c2: float
    ) -> np.array:
        """
        Get the intersection point of two lines.

        The two lines are described by m1 * x + c1 and m2 * x + c2.

        Args
        ----
        m1: float
            Gradient of line 1
        c1: float
            y-intercept of line 1
        m2: float
            Gradient of line 2
        c2: float
            y-intercept of line2

        Returns
        -------
        np.array (2,)
            The intersection point of line 1 and 2

        """
        def line1(x):
            return m1 * x + c1

        x_point = (c2 - c1) / (m1 - m2)

        return np.array([x_point, line1(x_point)])

    def _is_left_turn(self, intersection_point: np.array, end_point: np.array) -> bool:
        """
        Determine if a trajectory will be a left turn.

        Uses the determinant to determine whether the arc formed by the
        intersection and end point turns left or right.

        Args
        ----
        intersection_point: np.array(2,)
            The intersection point of the lines formed from the start
            and end angles
        end_point: np.array(2,)
            The chosen end point of the trajectory

        Returns
        -------
        bool
            True if curve turns left, false otherwise

        """
        matrix = np.vstack([intersection_point, end_point])
        det = np.linalg.det(matrix)

        return det >= 0

    def _is_dir_vec_correct(
        self, point1: np.array, point2: np.array, line_angle: float
    ) -> bool:
        """
        Check that the direction vector agrees with the line angle.

        The direction vector is defined as the vector from point 1 to
        point 2.

        Args
        ----
        point1: np.array(2,)
            The start point of the vector
        point2: np.array(2,)
            The end point of the vector
        line_angle: float
            The angle of a line to compare against the vector

        Returns
        -------
        bool
            True if both line and vector point in same direction

        """
        # Need to round to prevent very small values for 0
        m = abs(np.tan(line_angle).round(5))

        if line_angle < 0:
            m *= -1

        direction_vec_from_points = point2 - point1

        direction_vec_from_gradient = np.array([1, m])

        # Handle when line angle is in quadrant 2 or 3 and when angle is 90
        if abs(line_angle) > np.pi / 2:
            direction_vec_from_gradient = np.array([-1, m])
        elif abs(line_angle) == np.pi / 2:
            direction_vec_from_gradient = np.array([0, m])

        direction_vec_from_gradient = direction_vec_from_gradient.round(5)
        direction_vec_from_points = direction_vec_from_points.round(5)

        if np.all(
            np.sign(direction_vec_from_points) == np.sign(direction_vec_from_gradient)
        ):
            return True
        else:
            return False

    def _calculate_trajectory_params(
        self, end_point: np.array, start_angle: float, end_angle: float
    ) -> Union[TrajectoryParameters, None]:
        """
        Calculate the parameters for a trajectory with the desired constraints.

        The trajectory may consist of an arc and at most two line segments.
        A straight trajectory will consist of a single line segment. Similarly,
        a purely curving trajectory will only consist of an arc.

        Idea:
            1. Extend a line from (0,0) with angle of start_angle
            2. Extend a line from end_point with angle of end_angle
            3. Compute their intersection point, I
            4. Check that the intersection point leads to a
            valid trajectory
                - If I is too close to (0,0) or the end point then
                no arc greater than the turning radius will reach
                from (0,0) to end point

        If two segments from the same exterior point are tangent to
        a circle then they are congruent

        Args
        ----
        end_point: np.array(2,)
            The desired end point of the trajectory
        start_angle: float
            The start angle of the trajectory in radians
        end_angle: float
            The end angle of the trajectory in radians

        Returns
        -------
        TrajectoryParameters or None
            If a valid trajectory exists then the Trajectory parameters
            are returned, otherwise None

        """
        x2, y2 = end_point
        arc_start_point = np.array([0, 0])
        arc_end_point = end_point

        # Find gradient of line 1 passing through (0,0) that makes an angle
        # of start_angle with x_axis
        m1 = np.tan(start_angle).round(5)

        # Find gradient of line 2 passing through end point that makes an angle
        # of end_angle with x-axis
        m2 = np.tan(end_angle).round(5)

        # Deal with lines that are parallel
        if m1 == m2:
            # If they are coincident (i.e. y-intercept is same) then simply
            # return a circle with infinite radius
            if round(-m2 * x2 + y2, 5) == 0:
                return TrajectoryParameters.no_arc(
                    end_point=end_point, start_angle=start_angle, end_angle=end_angle
                )

            # Deal with edge case of 90
            elif (
                abs(start_angle) == np.pi / 2 and arc_end_point[0] == arc_start_point[0]
            ):
                return TrajectoryParameters.no_arc(
                    end_point=end_point,
                    start_angle=start_angle,
                    end_angle=end_angle,
                )

            else:
                logger.debug(
                    'No trajectory possible for equivalent start and '
                    + f'end angles that also passes through p = {x2, y2}'
                )
                return None

        # Find intersection point of lines 1 and 2
        intersection_point = self._get_intersection_point(m1, 0, m2, -m2 * x2 + y2)

        # Check that the vector from (0,0) to intersection point agrees
        # with the angle of line 1
        if not self._is_dir_vec_correct(
            arc_start_point, intersection_point, start_angle
        ):
            logger.debug(
                'No trajectory possible since intersection point occurs '
                + 'before start point on line 1'
            )
            return None

        # Check that the vector from intersection point to arc start point agrees with
        # the angle of line 2
        if not self._is_dir_vec_correct(intersection_point, arc_end_point, end_angle):
            logger.debug(
                'No trajectory possible since intersection point occurs '
                + 'after end point on line 2'
            )
            return None

        # Calculate distance between arc start point and intersection point
        dist_a = round(np.linalg.norm(arc_start_point - intersection_point), 5)

        # Calculate distance between arc end point and intersection point
        dist_b = round(np.linalg.norm(arc_end_point - intersection_point), 5)

        # Calculate the angle between start angle and end angle lines
        angle_between_lines = np.pi - abs(end_angle - start_angle)

        # The closer the arc start and end points are to the intersection point the
        # smaller the turning radius will be. However, we have a constraint on how
        # small this turning radius can get. To calculate the minimum allowed
        # distance we draw a right triangle with height equal to the constrained
        # radius and angle opposite the height leg as half the angle between the
        # start and end angle lines. The minimum valid distance is then the
        # base of this triangle.
        min_valid_distance = round(
            self.turning_radius / np.tan(angle_between_lines / 2), 5
        )

        # Both the distance of p along line 2 and intersection point along
        # line 1 must be greater than the minimum valid distance
        if dist_a < min_valid_distance or dist_b < min_valid_distance:
            logger.debug(
                'No trajectory possible where radius is larger than '
                + 'minimum turning radius'
            )
            return None

        if dist_a < dist_b:
            # Find new point on line 2 that is equidistant away from
            # intersection point as arc start point is on line 1
            vec_line2 = arc_end_point - intersection_point
            vec_line2 /= np.linalg.norm(vec_line2)
            arc_end_point = intersection_point + dist_a * vec_line2

        elif dist_a > dist_b:
            # Find new point on line 1 that is equidistant away from
            # intersection point as arc end point is on line 2
            vec_line1 = arc_start_point - intersection_point
            vec_line1 /= np.linalg.norm(vec_line1)

            arc_start_point = intersection_point + dist_b * vec_line1

        x1, y1 = arc_start_point
        x2, y2 = arc_end_point

        # Find intersection point of the perpindicular lines of line 1 and 2
        # that pass through arc start and arc end point respectively
        if m1 == 0:
            # If line 1 has gradient 0 then it is the x-axis.

            def perp_line2(x):
                return -1 / m2 * (x - x2) + y2

            circle_center = np.array([x1, perp_line2(x1)])
        elif m2 == 0:

            def perp_line1(x):
                return -1 / m1 * (x - x1) + y1

            circle_center = np.array([x2, perp_line1(x2)])
        else:
            perp_m1 = -1 / m1 if m1 != 0 else 0
            perp_m2 = -1 / m2 if m2 != 0 else 0

            circle_center = self._get_intersection_point(
                perp_m1, -perp_m1 * x1 + y1, perp_m2, -perp_m2 * x2 + y2
            )

        # The circles radius is the length from the center to arc start/end point
        # (both distances are the same)
        radius = np.linalg.norm(circle_center - arc_end_point).round(5)
        x_offset = circle_center[0].round(5)
        y_offset = circle_center[1].round(5)

        if radius < self.turning_radius:
            logger.debug(
                'Calculated circle radius is smaller than allowed turning '
                + f'radius: r = {radius}, min_radius = {self.turning_radius}'
            )
            return None

        left_turn = self._is_left_turn(intersection_point, end_point)

        return TrajectoryParameters(
            radius,
            x_offset,
            y_offset,
            end_point,
            start_angle,
            end_angle,
            left_turn,
            arc_start_point,
            arc_end_point,
        )

    def generate_trajectory(
        self,
        end_point: np.array,
        start_angle: float,
        end_angle: float,
        primitive_resolution: float,
    ) -> Union[Trajectory, None]:
        """
        Create a trajectory from (0,0, start_angle) to (end_point, end_angle).

        The trajectory will consist of a path that contains discrete points
        that are spaced primitive_resolution apart.

        Args
        ----
        end_point: np.array(2,)
            The desired end point of the trajectory
        start_angle: float
            The start angle of the trajectory in radians
        end_angle: float
            The end angle of the trajectory in radians
        primitive_resolution: float
            The spacing between points along the trajectory

        Returns
        -------
        Trajectory or None
            If a valid trajectory exists then the Trajectory is returned,
            otherwise None

        """
        trajectory_params = self._calculate_trajectory_params(
            end_point, start_angle, end_angle
        )

        if trajectory_params is None:
            return None

        logger.debug('Trajectory found')

        trajectory_path = self._create_path(trajectory_params, primitive_resolution)

        return Trajectory(trajectory_path, trajectory_params)
