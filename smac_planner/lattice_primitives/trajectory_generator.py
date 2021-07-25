from helper import normalize_angle
import logging
from typing import Union

import numpy as np

from trajectory import Trajectory, TrajectoryParameters, TrajectoryPath

logger = logging.getLogger(__name__)


class TrajectoryGenerator:

    def __init__(self, config: dict):
        self.turning_radius = config["turningRadius"]

    def _create_arc_path(self, trajectory_params: TrajectoryParameters, step_distance: float) -> TrajectoryPath:
        '''
        Create arc trajectory using the following parameterization:
                r(t) = <R cos(t - pi/2) + a, R sin(t - pi/2) + b>

                R = radius
                a = x offset
                b = y offset

        This parameterization has the property that start_t represents the slope angle of the start point of the curve (represented in radians) and
        end_t represents the slope angle of the end point of the curve.

        Parameters
        ----------
        trajectory_params: TrajectoryParameters
                The parameters that describe the arc to create
        step_distance: float
                The distance between sampled points along the arc

        Returns
        -------
        TrajectoryPath
                A trajectory path built only from the arc portion of the trajectory parameters
        '''

        steps = int(round(trajectory_params.arc_length / step_distance))

        start_angle = trajectory_params.start_angle
        end_angle = trajectory_params.end_angle

        # The -pi/pi boundary presents a problem so ensure that both angles have the same sign if we are on the boundary
        if abs(start_angle) == np.pi:
            start_angle = np.copysign(start_angle, end_angle)

        if abs(end_angle) == np.pi:
            end_angle = np.copysign(end_angle, start_angle)

        if trajectory_params.left_turn:
            if start_angle > end_angle:
                end_angle += 2*np.pi

            yaws = np.linspace(start_angle, end_angle, steps)
            yaws = normalize_angle(yaws)

            ts = np.linspace(start_angle, end_angle, steps)
            xs = trajectory_params.radius * \
                np.cos(ts - np.pi/2) + trajectory_params.x_offset
            ys = trajectory_params.radius * \
                np.sin(ts - np.pi/2) + trajectory_params.y_offset
        else:
            start_angle = -start_angle
            end_angle = -end_angle

            if start_angle > end_angle:
                end_angle += 2*np.pi

            yaws = np.linspace(-start_angle, -end_angle, steps)
            yaws = normalize_angle(yaws)

            ts = np.linspace(start_angle, end_angle, steps)
            xs = trajectory_params.radius * - \
                np.cos(ts + np.pi/2) + trajectory_params.x_offset
            ys = trajectory_params.radius * \
                np.sin(ts + np.pi/2) + trajectory_params.y_offset

        return TrajectoryPath(xs, ys, yaws)

    def _create_line_path(self, trajectory_params: TrajectoryParameters, start_point: np.array, end_point: np.array, step_distance: float) -> TrajectoryPath:
        '''
        Create straight line trajectory from (x1, y1) to (x2, y2)

        Parameters
        ----------
        trajectory_params: TrajectoryParameters
                The parameters that describe the trajectory to create
        start_p: np.array(2,)
                [x, y] coordinate of start point
        end_p: np.array(2,)
                [x, y] coordinate of end point
        step_distance: float
                The distance between sampled points along the line

        Returns
        -------
        TrajectoryPath
                A straight line path from (x1, y1) to (x2, y2)
        '''
        if start_point[0] == 0 and start_point[1] == 0:
            distance = trajectory_params.start_to_arc_distance
            line_angle = trajectory_params.start_angle
        else:
            distance = trajectory_params.arc_to_end_distance
            line_angle = trajectory_params.end_angle

        steps = int(round(distance / step_distance))

        # If steps is 0 or 1 then simply return the end point
        if steps <= 1:
            xs = np.array([end_point[0]])
            ys = np.array([end_point[1]])
            yaws = np.array([line_angle])
            return TrajectoryPath(xs, ys, yaws)

        ts = np.linspace(0, 1, steps)

        xs = (1-ts) * start_point[0] + ts * end_point[0]
        ys = (1-ts) * start_point[1] + ts * end_point[1]

        yaws = np.full(xs.shape, line_angle, dtype=np.float64)

        return TrajectoryPath(xs, ys, yaws)

    def _create_path(self, trajectory_params: TrajectoryParameters, step_distance: float) -> TrajectoryPath:
        '''
        Create the full trajectory path that is represented by the given trajectory parameters

        Parameters
        ----------
        trajectory_params: TrajectoryParameters
                The parameters that describe the trajectory to create
        step_distance: float
                The distance between sampled points along the line

        Returns
        -------
        TrajectoryPath
                The trajectory path described by the trajectory parameters
        '''

        if trajectory_params.radius > 0:
            final_trajectory = self._create_arc_path(
                trajectory_params, step_distance)

            if trajectory_params.start_to_arc_distance > 0:
                start_point = np.array([0, 0])
                end_point = np.array(
                    [final_trajectory.xs[0], final_trajectory.ys[0]])
                start_to_arc_trajectory = self._create_line_path(
                    trajectory_params, start_point, end_point, step_distance)

                final_trajectory = start_to_arc_trajectory + final_trajectory

            if trajectory_params.arc_to_end_distance > 0:
                start_point = np.array(
                    [final_trajectory.xs[-1], final_trajectory.ys[-1]])
                arc_to_end_trajectory = self._create_line_path(
                    trajectory_params, start_point, trajectory_params.end_point, step_distance)

                final_trajectory = final_trajectory + arc_to_end_trajectory

            return final_trajectory
        else:
            # No arc in path therefore its only a straight line3
            start_point = np.array([0, 0])
            return self._create_line_path(trajectory_params, start_point, trajectory_params.end_point, step_distance)

    def _get_intersection_point(self, m1: float, c1: float, m2: float, c2: float) -> np.array:
        '''
        Gets the intersection point of two lines described by m1 * x + c1 and m2 * x + c2

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
        '''

        def line1(x): return m1 * x + c1

        x_point = (c2 - c1) / (m1 - m2)

        return np.array([x_point, line1(x_point)])

    def _is_left_turn(self, intersection_point: np.array, end_point: np.array) -> bool:
        '''
        Uses the determinant to determine whether the arc formed by intersection and end point turns left or right

        Parameters
        ----------
        intersection_point: np.array(2,)
                The intersection point of the lines formed from the start and end angles
        end_point: np.array(2,)
                The chosen end point of the trajectory

        Returns
        -------
        bool
                True if curve turns left, false otherwise
        '''
        matrix = np.vstack([intersection_point, end_point])
        det = np.linalg.det(matrix)

        return det >= 0

    def _is_direction_vector_correct(self, point1: np.array, point2: np.array, line_angle: float) -> bool:
        '''
        Checks whether the vector from point 1 -> point 2 shares the same direction as line angle

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
        '''

        # Need to round to prevent very small values for 0
        m = abs(np.tan(line_angle).round(5))

        if line_angle < 0:
            m *= -1

        direction_vec_from_points = point2 - point1

        direction_vec_from_gradient = np.array([1, m])

        # Handle when line angle is in quadrant 2/3 and when angle is 90
        if abs(line_angle) > np.pi/2:
            direction_vec_from_gradient = np.array([-1, m])
        elif abs(line_angle) == np.pi/2:
            direction_vec_from_gradient = np.array([0, m])

        direction_vec_from_gradient = direction_vec_from_gradient.round(5)
        direction_vec_from_points = direction_vec_from_points.round(5)

        if np.all(np.sign(direction_vec_from_points) == np.sign(direction_vec_from_gradient)):
            return True
        else:
            return False

    def _calculate_trajectory_params(self, end_point: np.array, start_angle: float, end_angle: float) -> Union[TrajectoryParameters, None]:
        '''
        Calculates the trajectory parameters for a circle passing through (0,0) with a heading of start_angle
        and subsequently passing through point p with heading of end_angle.

        Idea:
                1. Extend a line from (0,0) with angle of start_angle
                2. Extend a line from end_point with angle of end_angle
                3. Compute their intersection point, I
                4. Check that the intersection point leads to a valid trajectory
                        - If I is too close to (0,0) or the end point then no arc greater than the turning radius will reach from (0,0) to end point



                If two segments from the same exterior point are tangent to a circle then they are congruent

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
                If a valid trajectory exists then the Trajectory parameters are returned, otherwise None
        '''
        start_to_arc_distance = 0
        arc_to_end_distance = 0

        x2, y2 = end_point
        p = end_point
        q = np.array([0, 0])

        # Find gradient of line 1 passing through (0,0) that makes an angle of start_angle with x_axis
        m1 = np.tan(start_angle).round(5)

        # Find gradient of line 2 passing through (x2,y2) that makes an angle of end_angle with x-axis
        m2 = np.tan(end_angle).round(5)

        # Deal with lines that are parallel
        if m1 == m2:
            # If they are coincident (i.e. y-intercept is same) then simply return a circle with infinite radius
            if round(-m2 * x2 + y2, 5) == 0:
                return TrajectoryParameters.no_arc(end_point=end_point, start_angle=start_angle, end_angle=end_angle, left_turn=True)

            # Deal with edge case of 90
            elif abs(start_angle) == np.pi/2 and p[0] == q[0]:
                return TrajectoryParameters.no_arc(end_point=end_point, start_angle=start_angle, end_angle=end_angle, left_turn=True)

            else:
                logger.info(
                    f'No trajectory possible for equivalent start and end angles that also passes through p = {x2, y2}')
                return None

        angle_between_lines = np.pi - abs(end_angle - start_angle)

        # Find intersection point of the lines 1 and 2
        intersection_point = self._get_intersection_point(
            m1, 0, m2, -m2 * x2 + y2)

        # Check that the vector from (0,0) to intersection point agrees with the angle of line 1
        if not self._is_direction_vector_correct(q, intersection_point, start_angle):
            logger.info(
                "No trajectory possible since intersection point occurs before start point on line 1")
            return None

        # Check that the vector from intersection point to p agrees with the angle of line 2
        if not self._is_direction_vector_correct(intersection_point, p, end_angle):
            logger.info(
                "No trajectory possible since intersection point occurs after end point on line 2")
            return None

        # Calculate distance between point p = (x2,y2) and intersection point
        dist_p = round(np.linalg.norm(p-intersection_point, axis=0), 5)

        # Calculate distance between point q = (0,0) and intersection point
        dist_q = round(np.linalg.norm(q-intersection_point), 5)

        # Turn the turning radius into a minimum valid distance
        min_valid_distance = round(
            self.turning_radius / np.tan(angle_between_lines / 2), 5)

        # Both the distance of p along line 2 and intersection point along line 1 must be greater than the minimum valid distance
        if dist_p < min_valid_distance or dist_q < min_valid_distance:
            logger.info(
                "No trajectory possible where radius is larger than minimum turning radius")
            return None

        if dist_q < dist_p:
            # Find new point p on line 2 that is equidistant away from intersection point as point q is on line 1
            vec_line2 = p - intersection_point
            vec_line2 /= np.linalg.norm(vec_line2)
            p = intersection_point + dist_q * vec_line2

            arc_to_end_distance = dist_p - dist_q

        elif dist_q > dist_p:
            # Find new point q on line 1 that is equidistant away from intersection point as point p is on line 2
            vec_line1 = q - intersection_point
            vec_line1 /= np.linalg.norm(vec_line1)

            q = intersection_point + dist_p * vec_line1

            start_to_arc_distance = dist_q - dist_p

        x1, y1 = q
        x2, y2 = p

        # Find intersection point of the perpindicular lines of line 1 and 2 that pass through q and p respectively
        if m1 == 0:
            # If line 1 has gradient 0 then it is the x-axis.

            def perp_line2(x): return -1/m2 * (x - x2) + y2
            circle_center = np.array([x1, perp_line2(x1)])
        elif m2 == 0:
            def perp_line1(x): return -1/m1 * (x - x1) + y1
            circle_center = np.array([x2, perp_line1(x2)])
        else:
            perp_m1 = -1/m1 if m1 != 0 else 0
            perp_m2 = -1/m2 if m2 != 0 else 0

            circle_center = self._get_intersection_point(
                perp_m1, -perp_m1 * x1 + y1, perp_m2, -perp_m2 * x2 + y2)

        # The circles radius is the length from the center to point p (or q)
        radius = round(np.linalg.norm(circle_center - p), 4)
        x_offset = round(circle_center[0], 4)
        y_offset = round(circle_center[1], 4)

        if radius < self.turning_radius:
            logger.info(
                f'Calculated circle radius is smaller than allowed turning radius: r = {radius}, min_radius = {self.turning_radius}')
            return None

        left_turn = self._is_left_turn(intersection_point, end_point)

        return TrajectoryParameters(radius, x_offset, y_offset, end_point, start_angle, end_angle, left_turn, start_to_arc_distance, arc_to_end_distance)

    def generate_trajectory(self, end_point: float, start_angle: float, end_angle: float, step_distance: float) -> Union[Trajectory, None]:
        '''
        Creates a trajectory from (0,0, start_angle) to (end_point, end_angle) with points spaced step_distance apart

        Parameters
        ----------
        end_point: np.array(2,)
                The desired end point of the trajectory
        start_angle: float
                The start angle of the trajectory in radians
        end_angle: float
                The end angle of the trajectory in radians
        step_distance: float
                The spacing between points along the trajectory

        Returns
        -------
        Trajectory or None
                If a valid trajectory exists then the Trajectory is returned, otherwise None
        '''
        trajectory_params = self._calculate_trajectory_params(
            end_point, start_angle, end_angle)

        if trajectory_params is None:
            return None

        logger.debug("Trajectory found")

        trajectory_path = self._create_path(trajectory_params, step_distance)

        return Trajectory(trajectory_path, trajectory_params)
