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

from helper import get_rotation_matrix, interpolate_yaws
from trajectory import Trajectory, TrajectoryParameters, TrajectoryPath

logger = logging.getLogger(__name__)


import matplotlib.pyplot as plt

class TrajectoryGenerator:
    def __init__(self, config: dict):
        self.turning_radius = config["turning_radius"]


    def _create_arc_path(
        self,
        trajectory_params: TrajectoryParameters,
        primitive_resolution: float,
        distance_offset: float
    ) -> Tuple[TrajectoryPath, float]:
        """
        Create arc trajectory using the following parameterization:
            r(t) = <R * cos(t - pi/2) + a, R * sin(t - pi/2) + b>

            R = radius
            a = x offset
            b = y offset

        This parameterization has the property that start_t represents
        the slope angle of the start point of the curve
        (represented in radians) and end_t represents the slope angle
        of the end point of the curve.

        Parameters
        ----------
        trajectory_params: TrajectoryParameters
            The parameters that describe the arc to create
        primitive_resolution: float
            The distance between sampled points along the arc
        distance_offset: float
            The distance to adjust the starting point of the arc by.
            This value is used to ensure that the spacing between the 
            previous trajectory point and the start of the arc is still 
            roughly equal to the primitive resolution.

        Returns
        -------
        TrajectoryPath
            A trajectory path built only from the arc portion of the
            trajectory parameters
        float
            The distance that remains between the actual generated end point
            of the arc and the desired end point. This value arises from the
            fact that we are attempting to represent the arc as a set of
            discrete points each spaced out by a distance equal to the primitive
            resolution.
        """

        # Calculate number of steps needed and the distance that remains 
        # between the last generated point and the end point
        steps = int((trajectory_params.arc_length - distance_offset) // primitive_resolution)
        remaining_distance = (trajectory_params.arc_length - distance_offset) - steps * primitive_resolution

        # Need to convert distance offset to an angle offset due to how we parametrize the arc
        angle_offset = distance_offset / trajectory_params.turning_radius

        start_angle = trajectory_params.start_angle
        end_angle = trajectory_params.end_angle

        angle_spacing = primitive_resolution / trajectory_params.turning_radius

        # The -pi/pi boundary presents a problem so ensure that
        # both angles have the same sign if we are on the boundary
        if abs(start_angle) == np.pi:
            start_angle = np.copysign(start_angle, end_angle)
        if abs(end_angle) == np.pi:
            end_angle = np.copysign(end_angle, start_angle)

        # Need to add angle offset if its a left turn and subtract if its a right turn
        if trajectory_params.left_turn:
            start_angle += angle_offset
        else:
            start_angle -= angle_offset

        yaws = interpolate_yaws(
            start_angle, end_angle, trajectory_params.left_turn, steps + 1
        )

        if trajectory_params.left_turn:
            if start_angle > end_angle:
                end_angle += 2 * np.pi

            # Calculate points using
            # r(t) = <R * cos(t - pi/2) + a, R * sin(t - pi/2) + b>
            ts = angle_spacing * np.arange(0, steps + 1) + start_angle
            xs = (
                trajectory_params.turning_radius * np.cos(ts - np.pi / 2)
                + trajectory_params.x_offset
            )
            ys = (
                trajectory_params.turning_radius * np.sin(ts - np.pi / 2)
                + trajectory_params.y_offset
            )
        else:
            # Right turns go the opposite way across the arc, so we 
            # need to invert the angles and adjust the parametrization
            start_angle = -start_angle
            end_angle = -end_angle

            if start_angle > end_angle:
                end_angle += 2 * np.pi

            # Calculate points using
            # r(t) = <R * -cos(t + pi/2) + a, R * sin(t + pi/2) + b>
            ts = angle_spacing * np.arange(0, steps + 1) + start_angle
            xs = (
                trajectory_params.turning_radius * -np.cos(ts + np.pi / 2)
                + trajectory_params.x_offset
            )
            ys = (
                trajectory_params.turning_radius * np.sin(ts + np.pi / 2)
                + trajectory_params.y_offset
            )

        return TrajectoryPath(xs, ys, yaws), remaining_distance


    def _create_line_path(
        self,
        distance,
        line_angle,
        start_point: np.array,
        end_point: np.array,
        primitive_resolution: float,
        distance_offset: float
    ) -> Tuple[TrajectoryPath, float]:
        """
        Create straight line trajectory from (x1, y1) to (x2, y2)

        Parameters
        ----------
        trajectory_params: TrajectoryParameters
            The parameters that describe the trajectory to create
        start_p: np.array(2,)
            [x, y] coordinate of start point
        end_p: np.array(2,)
            [x, y] coordinate of end point
        primitive_resolution: float
            The distance between sampled points along the line
        distance_offset: float
            The distance to adjust the starting point of the line by.
            This value is used to ensure that the spacing between the 
            previous trajectory point and the start of the line is still 
            roughly equal to the primitive resolution.

        Returns
        -------
        TrajectoryPath
            A straight line path from (x1, y1) to (x2, y2)
        float
            The distance that remains between the actual generated end point
            of the line and the desired end point. This value arises from the
            fact that we are attempting to represent the line as a set of
            discrete points each spaced out by a distance equal to the primitive
            resolution.
        """

        # Calculate number of steps needed and the distance that remains 
        # between the last generated point and the end point
        steps = int((distance - distance_offset) // primitive_resolution)
        remaining_distance = (distance - distance_offset) - (primitive_resolution * steps)

        # Compute the direction vector that points from start point to end point
        # and has a length of primitve_resolution
        dir_vec = (end_point - start_point).astype("float").reshape(2, 1)
        dir_vec /= np.linalg.norm(dir_vec)
        dir_vec *= primitive_resolution

        # Create the points along the path by multiplying the step number 
        # by the direction vector and adding the start point
        if steps == 0:
            step_numbers = np.array([1])
        else:
            step_numbers = np.arange(1, steps + 1)

        points = dir_vec * step_numbers + start_point.reshape(2,1)
        xs = points[0, :]
        ys = points[1, :]

        # yaw is always the same angle since its a straight line
        yaws = np.full(xs.shape, line_angle, dtype=np.float64)

        return TrajectoryPath(xs, ys, yaws), remaining_distance


    def _create_path(
        self,
        trajectory_params: TrajectoryParameters,
        primitive_resolution: float
    ) -> TrajectoryPath:
        """
        Create the full trajectory path that is represented by the
        given trajectory parameters

        Parameters
        ----------
        trajectory_params: TrajectoryParameters
            The parameters that describe the trajectory to create
        primitive_resolution: float
            The distance between sampled points along the line

        Returns
        -------
        TrajectoryPath
            The trajectory path described by the trajectory parameters
        """

        if trajectory_params.turning_radius > 0:
            final_trajectory = TrajectoryPath.empty()
            remaining_distance = 0

            # Create straight line path from start to arc if needed
            if trajectory_params.start_to_arc_distance > primitive_resolution:
                start_point = np.array([0, 0])

                # Calculate the end point of the path
                rot_matrix = get_rotation_matrix(trajectory_params.start_angle)
                end_point = rot_matrix @ np.array([[trajectory_params.start_to_arc_distance], [0]])
                end_point = end_point.flatten()          

                start_to_arc_trajectory, remaining_distance = self._create_line_path(
                    trajectory_params.start_to_arc_distance,
                    trajectory_params.start_angle,
                    start_point,
                    end_point,
                    primitive_resolution,
                    0
                )
                
                final_trajectory += start_to_arc_trajectory

            # The distance between two points should always be primitive_resolution
            # so we calculate the offset needed based on the previous generated point
            # (or leave it as 0 if the last point was at the correct end point)
            distance_offset = primitive_resolution - remaining_distance if remaining_distance > 0 else 0

            # Calculate the arc portion of the path
            arc_path, remaining_distance = self._create_arc_path(trajectory_params, primitive_resolution, distance_offset)
            final_trajectory += arc_path

            # Create straight line path from arc to end if needed
            if trajectory_params.arc_to_end_distance > primitive_resolution:
                start_point = np.array(
                    [final_trajectory.xs[-1], final_trajectory.ys[-1]]
                )

                distance_offset = primitive_resolution - remaining_distance if remaining_distance > 0 else 0
                arc_to_end_trajectory, remaining_distance = self._create_line_path(
                    trajectory_params.arc_to_end_distance,
                    trajectory_params.end_angle,
                    start_point,
                    trajectory_params.end_point,
                    primitive_resolution,
                    distance_offset
                )

                final_trajectory += arc_to_end_trajectory

        else:
            # No arc in path therefore its only a straight line
            start_point = np.array([0, 0])
            line_path, remaining_distance = self._create_line_path(
                trajectory_params.start_to_arc_distance,
                trajectory_params.start_angle,
                start_point,
                trajectory_params.end_point,
                primitive_resolution,
                0
            )

            final_trajectory = line_path

        # Due to discretization issues we need to force the last point to be the end point
        traj_end = np.array([final_trajectory.xs[-1], final_trajectory.ys[-1]])
        dist_to_end = np.linalg.norm(traj_end - trajectory_params.end_point)

        # If the last point in the generated path is closer than half a primtiive_resolution 
        # to the end we just change it to lie on the end. Otherwise we add in a new point to 
        # the path that lies at the end.
        if dist_to_end < 0.5 * primitive_resolution:
            final_trajectory.xs[-1] = trajectory_params.end_point[0]
            final_trajectory.ys[-1] = trajectory_params.end_point[1]
            final_trajectory.yaws[-1] = trajectory_params.end_angle
        else:
            end = TrajectoryPath(np.array([trajectory_params.end_point[0]]), 
                                 np.array([trajectory_params.end_point[1]]), 
                                 np.array([trajectory_params.end_angle]))
            final_trajectory += end

        # Remove the start point (i.e. 0,0)
        final_trajectory = final_trajectory.remove_start()

        return final_trajectory


    def _get_intersection_point(
        self, m1: float, c1: float, m2: float, c2: float
    ) -> np.array:
        """
        Gets the intersection point of two lines described by m1 * x + c1
        and m2 * x + c2

        Parameters
        ----------
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


    def _is_left_turn(
        self,
        intersection_point: np.array,
        end_point: np.array
    ) -> bool:
        """
        Uses the determinant to determine whether the arc formed by
        intersection and end point turns left or right

        Parameters
        ----------
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
        Checks whether the vector from point 1 -> point 2 shares the
        same gradient as line_angle

        Parameters
        ----------
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
            np.sign(direction_vec_from_points) ==
            np.sign(direction_vec_from_gradient)
        ):
            return True
        else:
            return False


    def _calculate_trajectory_params(
        self, end_point: np.array, start_angle: float, end_angle: float
    ) -> Union[TrajectoryParameters, None]:
        """
        Calculates the trajectory parameters for a circle passing through (0,0)
        with a heading of start_angle and subsequently passing through point p
        with heading of end_angle.

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

        Parameters
        ----------
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
        start_to_arc_distance = 0
        arc_to_end_distance = 0

        x2, y2 = end_point
        p = end_point
        q = np.array([0, 0])

        # Find gradient of line 1 passing through (0,0) that makes an angle
        # of start_angle with x_axis
        m1 = np.tan(start_angle).round(5)

        # Find gradient of line 2 passing through (x2,y2) that makes an angle
        # of end_angle with x-axis
        m2 = np.tan(end_angle).round(5)

        # Deal with lines that are parallel
        if m1 == m2:
            # If they are coincident (i.e. y-intercept is same) then simply
            # return a circle with infinite radius
            if round(-m2 * x2 + y2, 5) == 0:
                return TrajectoryParameters.no_arc(
                    end_point=end_point,
                    start_angle=start_angle,
                    end_angle=end_angle,
                    left_turn=True,
                )

            # Deal with edge case of 90
            elif abs(start_angle) == np.pi / 2 and p[0] == q[0]:
                return TrajectoryParameters.no_arc(
                    end_point=end_point,
                    start_angle=start_angle,
                    end_angle=end_angle,
                    left_turn=True,
                )

            else:
                logger.debug(
                    'No trajectory possible for equivalent start and ' +
                    f'end angles that also passes through p = {x2, y2}'
                )
                return None

        # Find intersection point of the lines 1 and 2
        intersection_point = \
            self._get_intersection_point(m1, 0, m2, -m2 * x2 + y2)

        # Check that the vector from (0,0) to intersection point agrees
        # with the angle of line 1
        if not self._is_dir_vec_correct(q, intersection_point, start_angle):
            logger.debug(
                "No trajectory possible since intersection point occurs " +
                "before start point on line 1"
            )
            return None

        # Check that the vector from intersection point to p agrees with
        # the angle of line 2
        if not self._is_dir_vec_correct(intersection_point, p, end_angle):
            logger.debug(
                "No trajectory possible since intersection point occurs " +
                "after end point on line 2"
            )
            return None

        # Calculate distance between point p = (x2,y2) and intersection point
        dist_p = round(np.linalg.norm(p - intersection_point, axis=0), 5)

        # Calculate distance between point q = (0,0) and intersection point
        dist_q = round(np.linalg.norm(q - intersection_point), 5)

        # Calculate the angle between start angle and end angle lines
        angle_between_lines = np.pi - abs(end_angle - start_angle)

        # The closer p and q are to the intersection point the smaller
        # the turning radius will be. However, we have a constraint on how
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
        if dist_p < min_valid_distance or dist_q < min_valid_distance:
            logger.debug(
                "No trajectory possible where radius is larger than " +
                "minimum turning radius"
            )
            return None

        if dist_q < dist_p:
            # Find new point p on line 2 that is equidistant away from
            # intersection point as point q is on line 1
            vec_line2 = p - intersection_point
            vec_line2 /= np.linalg.norm(vec_line2)
            p = intersection_point + dist_q * vec_line2

            arc_to_end_distance = (dist_p - dist_q).round(5)

        elif dist_q > dist_p:
            # Find new point q on line 1 that is equidistant away from
            # intersection point as point p is on line 2
            vec_line1 = q - intersection_point
            vec_line1 /= np.linalg.norm(vec_line1)

            q = intersection_point + dist_p * vec_line1

            start_to_arc_distance = (dist_q - dist_p).round(5)

        x1, y1 = q
        x2, y2 = p

        # Find intersection point of the perpindicular lines of line 1 and 2
        # that pass through q and p respectively
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

        # The circles radius is the length from the center to point p (or q)
        radius = np.linalg.norm(circle_center - p).round(5)
        x_offset = circle_center[0].round(5)
        y_offset = circle_center[1].round(5)

        if radius < self.turning_radius:
            logger.debug(
                f"Calculated circle radius is smaller than allowed turning " +
                "radius: r = {radius}, min_radius = {self.turning_radius}"
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
            start_to_arc_distance,
            arc_to_end_distance,
        )


    def generate_trajectory(
        self,
        end_point: np.array,
        start_angle: float,
        end_angle: float,
        primitive_resolution: float,
    ) -> Union[Trajectory, None]:
        """
        Creates a trajectory from (0,0, start_angle) to (end_point, end_angle)
        with points spaced primitive_resolution apart

        Parameters
        ----------
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

        logger.debug("Trajectory found")

        trajectory_path = \
            self._create_path(trajectory_params, primitive_resolution)

        return Trajectory(trajectory_path, trajectory_params)

if __name__ == "__main__":
    test = TrajectoryGenerator({"turning_radius": 0.5})

    end_point = np.array([0.15, 0.2])
    start_angle = 0.7853981633974483
    end_angle = 1.1071487177940904

    traj = test.generate_trajectory(end_point, start_angle, end_angle, 0.05)

    plt.plot(traj.path.xs, traj.path.ys, 'x')
    plt.axis("equal")
    plt.show()