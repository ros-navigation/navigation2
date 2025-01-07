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

from collections import defaultdict
from enum import Enum

from helper import angle_difference, interpolate_yaws

import numpy as np

from rtree import index

from trajectory import Path, Trajectory, TrajectoryParameters

from trajectory_generator import TrajectoryGenerator


class LatticeGenerator:
    """
    Handles all the logic for computing the minimal control set.

    Computes the minimal control set for a vehicle given its parameters.
    Includes handling the propagating and searching along wavefronts as
    well as determining if a trajectory is part of the minimal set based
    on previously added trajectories.
    """

    class MotionModel(Enum):
        """An Enum used for determining the motion model to use."""

        ACKERMANN = 1
        DIFF = 2
        OMNI = 3

    class Flip(Enum):
        """An Enum used for determining how a trajectory should be flipped."""

        X = 1
        Y = 2
        BOTH = 3

    def __init__(self, config: dict):
        """Init the lattice generator from the user supplied config."""
        self.trajectory_generator = TrajectoryGenerator(config)
        self.grid_resolution = config['grid_resolution']
        self.turning_radius = config['turning_radius']
        self.stopping_threshold = config['stopping_threshold']
        self.num_of_headings = config['num_of_headings']
        self.headings = self._get_heading_discretization(config['num_of_headings'])

        self.motion_model = self.MotionModel[config['motion_model'].upper()]

        self.DISTANCE_THRESHOLD = 0.5 * self.grid_resolution
        self.ROTATION_THRESHOLD = 0.5 * (2 * np.pi / self.num_of_headings)

    def _get_wave_front_points(self, pos: int) -> np.array:
        """
        Calculate the end points that lie on the wave front.

        Uses the user supplied grid resolution to calculate the
        valid end points that lie on a wave front at a discrete
        interval away from the origin.

        Args:
        pos: int
            The number of discrete intervals of grid resolution
            away from the origin to generate the wave points at

        Returns
        -------
        np.array
            An array of coordinates

        """
        positions = []

        max_point_coord = self.grid_resolution * pos

        for i in range(pos):
            varying_point_coord = self.grid_resolution * i

            # Change the y and keep x at max
            positions.append((max_point_coord, varying_point_coord))

            # Change the x and keep y at max
            positions.append((varying_point_coord, max_point_coord))

        # Append the corner
        positions.append((max_point_coord, max_point_coord))

        return np.array(positions)

    def _get_heading_discretization(self, number_of_headings: int) -> list:
        """
        Calculate the heading discretization based on the number of headings.

        Does not uniformly generate headings but instead generates a set of
        discrete headings that is better suited for straight line trajectories.

        Args:
        number_of_headings: int
            The number of headings to discretize a 360 degree turn into

        Returns
        -------
        list
            A list of headings in radians

        """
        max_val = int(number_of_headings / 8)

        outer_edge_x = []
        outer_edge_y = []

        # Generate points that lie on the perimeter of the surface
        # of a square with sides of length max_val
        for i in range(-max_val, max_val + 1):
            outer_edge_x.extend([i, i])
            outer_edge_y.extend([-max_val, max_val])

            if i != max_val and i != -max_val:
                outer_edge_x.extend([-max_val, max_val])
                outer_edge_y.extend([i, i])

        return sorted([np.arctan2(j, i) for i, j in zip(outer_edge_x, outer_edge_y)])

    def _point_to_line_distance(self, p1: np.array, p2: np.array, q: np.array) -> float:
        """
        Return the shortest distance from a point to a line segment.

        Args:
        p1: np.array(2,)
            Start point of line segment
        p2: np.array(2,)
            End point of line segment
        q: np.array(2,)
            Point to get distance away from line of

        Returns
        -------
        float
            The shortest distance between q and line segment p1p2

        """
        # Get back the l2-norm without the square root
        l2 = np.inner(p1 - p2, p1 - p2)

        if l2 == 0:
            return np.linalg.norm(p1 - q)

        # Ensure t lies in [0, 1]
        t = max(0, min(1, np.dot(q - p1, p2 - p1) / l2))
        projected_point = p1 + t * (p2 - p1)

        return np.linalg.norm(q - projected_point)

    def _is_minimal_trajectory(
        self, trajectory: Trajectory, prior_end_poses: index.Rtree
    ) -> bool:
        """
        Determine whether a trajectory is a minimal trajectory.

        Uses an RTree for speedup.

        Args:
        trajectory: Trajectory
            The trajectory to check
        prior_end_poses: RTree
            An RTree holding the current minimal set of trajectories

        Returns
        -------
        bool
            True if the trajectory is a minimal trajectory otherwise false

        """
        # Iterate over line segments in the trajectory
        for x1, y1, x2, y2, yaw in zip(
            trajectory.path.xs[:-1],
            trajectory.path.ys[:-1],
            trajectory.path.xs[1:],
            trajectory.path.ys[1:],
            trajectory.path.yaws[:-1],
        ):

            p1 = np.array([x1, y1])
            p2 = np.array([x2, y2])

            # Create a bounding box search region
            # around the line segment
            left_bb = min(x1, x2) - self.DISTANCE_THRESHOLD
            right_bb = max(x1, x2) + self.DISTANCE_THRESHOLD
            top_bb = max(y1, y2) + self.DISTANCE_THRESHOLD
            bottom_bb = min(y1, y2) - self.DISTANCE_THRESHOLD

            # For any previous end points in the search region we
            # check the distance to that point and the angle
            # difference. If they are within threshold then this
            # trajectory can be composed from a previous trajectory
            for prior_end_pose in prior_end_poses.intersection(
                (left_bb, bottom_bb, right_bb, top_bb), objects='raw'
            ):
                if (
                    self._point_to_line_distance(p1, p2, prior_end_pose[:-1])
                    < self.DISTANCE_THRESHOLD
                    and angle_difference(yaw, prior_end_pose[-1])
                    < self.ROTATION_THRESHOLD
                ):
                    return False

        return True

    def _compute_min_trajectory_length(self) -> float:
        """
        Compute the minimum trajectory length for the given parameters.

        The minimum trajectory length is defined as the length needed
        for the sharpest possible turn to move from 0 degrees to the next
        discrete heading. Since the distance between headings is not uniform
        we take the smallest possible difference.

        Returns
        -------
        float
            The minimal length of a trajectory

        """
        # Compute arc length for a turn that moves from 0 degrees to
        # the minimum heading difference
        heading_diff = [
            abs(self.headings[i + 1] - self.headings[i])
            for i in range(len(self.headings) - 1)
        ]

        return self.turning_radius * min(heading_diff)

    def _generate_minimal_spanning_set(self) -> dict:
        """
        Generate the minimal spanning set.

        Iteratves over all possible trajectories and keeps only those that
        are part of the minimal set.

        Returns
        -------
        dict
            A dictionary where the key is the start_angle and the value is
            a list of trajectories that begin at that angle

        """
        quadrant1_end_poses = defaultdict(list)

        # Since we only compute for quadrant 1 we only need headings between
        # 0 and 90 degrees
        initial_headings = sorted(
            filter(lambda x: 0 <= x and x <= np.pi / 2, self.headings)
        )

        # Use the minimum trajectory length to find the starting wave front
        min_trajectory_length = self._compute_min_trajectory_length()
        wave_front_start_pos = int(
            np.round(min_trajectory_length / self.grid_resolution)
        )

        for start_heading in initial_headings:
            iterations_without_trajectory = 0

            prior_end_poses = index.Index()

            wave_front_cur_pos = wave_front_start_pos

            # To get target headings: sort headings radially and remove those
            # that are more than 90 degrees away
            target_headings = sorted(
                self.headings, key=lambda x: (abs(x - start_heading), -x)
            )
            target_headings = list(
                filter(lambda x: abs(start_heading - x) <= np.pi / 2, target_headings)
            )

            while iterations_without_trajectory < self.stopping_threshold:
                iterations_without_trajectory += 1

                # Generate x,y coordinates for current wave front
                positions = self._get_wave_front_points(wave_front_cur_pos)

                for target_point in positions:
                    for target_heading in target_headings:
                        # Use 10% of grid separation for finer granularity
                        # when checking if trajectory overlaps another already
                        # seen trajectory
                        trajectory = self.trajectory_generator.generate_trajectory(
                            target_point,
                            start_heading,
                            target_heading,
                            0.1 * self.grid_resolution,
                        )

                        if trajectory is not None:
                            # Check if path overlaps something in minimal
                            # spanning set
                            if self._is_minimal_trajectory(trajectory, prior_end_poses):

                                # Add end pose to minimal set
                                new_end_pose = np.array(
                                    [target_point[0], target_point[1], target_heading]
                                )

                                quadrant1_end_poses[start_heading].append(
                                    (target_point, target_heading)
                                )

                                # Create a new bounding box in the RTree
                                # for this trajectory
                                left_bb = target_point[0] - self.DISTANCE_THRESHOLD
                                right_bb = target_point[0] + self.DISTANCE_THRESHOLD
                                bottom_bb = target_point[1] - self.DISTANCE_THRESHOLD
                                top_bb = target_point[1] + self.DISTANCE_THRESHOLD

                                prior_end_poses.insert(
                                    0,
                                    (left_bb, bottom_bb, right_bb, top_bb),
                                    new_end_pose,
                                )

                                iterations_without_trajectory = 0

                wave_front_cur_pos += 1

        # Once we have found the minimal trajectory set for quadrant 1
        # we can leverage symmetry to create the complete minimal set
        return self._create_complete_minimal_spanning_set(quadrant1_end_poses)

    def _flip_angle(self, angle: float, flip_type: Flip) -> float:
        """
        Return the the appropriate flip of the angle in self.headings.

        Args:
        angle: float
            The angle to flip
        flip_type: Flip
            Whether to flip acrpss X axis, Y axis, or both

        Returns
        -------
        float
            The angle in self.heading that is the appropriate flip

        """
        angle_idx = self.headings.index(angle)

        if flip_type == self.Flip.X:
            heading_idx = (self.num_of_headings / 2 - 1) - angle_idx - 1
        elif flip_type == self.Flip.Y:
            heading_idx = self.num_of_headings - angle_idx - 2
        elif flip_type == self.Flip.BOTH:
            heading_idx = (
                angle_idx - (self.num_of_headings / 2)
            ) % self.num_of_headings
        else:
            raise Exception(f'Unsupported flip type: {flip_type}')

        return self.headings[int(heading_idx)]

    def _create_complete_minimal_spanning_set(
        self, single_quadrant_minimal_set: dict
    ) -> dict:
        """
        Create the full minimal spanning set from a single quadrant set.

        Exploits the symmetry between the quadrants to create the full set.
        This is done by flipping every trajectory in the first quadrant across
        either the X-axis, Y-axis, or both axes.

        Args:
        single_quadrant_minimal_set: dict
            The minimal set for quadrant 1 (positive x and positive y)

        Returns
        -------
        dict
            The complete minimal spanning set containing the trajectories
            in all quadrants

        """
        all_trajectories = defaultdict(list)

        for start_angle in single_quadrant_minimal_set.keys():

            for end_point, end_angle in single_quadrant_minimal_set[start_angle]:

                x, y = end_point

                # Prevent double adding trajectories that lie on axes
                # (i.e. start and end angle are either both 0 or both pi/2)
                if start_angle == 0 and end_angle == 0:
                    unflipped_start_angle = 0.0
                    flipped_x_start_angle = np.pi

                    unflipped_end_angle = 0.0
                    flipped_x_end_angle = np.pi

                    # Generate trajectories from calculated parameters
                    unflipped_trajectory = (
                        self.trajectory_generator.generate_trajectory(
                            np.array([x, y]),
                            unflipped_start_angle,
                            unflipped_end_angle,
                            self.grid_resolution,
                        )
                    )
                    flipped_x_trajectory = (
                        self.trajectory_generator.generate_trajectory(
                            np.array([-x, -y]),
                            flipped_x_start_angle,
                            flipped_x_end_angle,
                            self.grid_resolution,
                        )
                    )

                    all_trajectories[
                        unflipped_trajectory.parameters.start_angle
                    ].append(unflipped_trajectory)

                    all_trajectories[
                        flipped_x_trajectory.parameters.start_angle
                    ].append(flipped_x_trajectory)

                elif abs(start_angle) == np.pi / 2 and abs(end_angle) == np.pi / 2:
                    unflipped_start_angle = np.pi / 2
                    flipped_y_start_angle = -np.pi / 2

                    unflipped_end_angle = np.pi / 2
                    flipped_y_end_angle = -np.pi / 2

                    # Generate trajectories from calculated parameters
                    unflipped_trajectory = (
                        self.trajectory_generator.generate_trajectory(
                            np.array([-x, y]),
                            unflipped_start_angle,
                            unflipped_end_angle,
                            self.grid_resolution,
                        )
                    )

                    flipped_y_trajectory = (
                        self.trajectory_generator.generate_trajectory(
                            np.array([x, -y]),
                            flipped_y_start_angle,
                            flipped_y_end_angle,
                            self.grid_resolution,
                        )
                    )

                    all_trajectories[
                        unflipped_trajectory.parameters.start_angle
                    ].append(unflipped_trajectory)
                    all_trajectories[
                        flipped_y_trajectory.parameters.start_angle
                    ].append(flipped_y_trajectory)

                else:
                    unflipped_start_angle = start_angle
                    flipped_x_start_angle = self._flip_angle(start_angle, self.Flip.X)
                    flipped_y_start_angle = self._flip_angle(start_angle, self.Flip.Y)
                    flipped_xy_start_angle = self._flip_angle(
                        start_angle, self.Flip.BOTH
                    )

                    unflipped_end_angle = end_angle
                    flipped_x_end_angle = self._flip_angle(end_angle, self.Flip.X)
                    flipped_y_end_angle = self._flip_angle(end_angle, self.Flip.Y)
                    flipped_xy_end_angle = self._flip_angle(end_angle, self.Flip.BOTH)

                    # Generate trajectories from calculated parameters
                    unflipped_trajectory = (
                        self.trajectory_generator.generate_trajectory(
                            np.array([x, y]),
                            unflipped_start_angle,
                            unflipped_end_angle,
                            self.grid_resolution,
                        )
                    )
                    flipped_x_trajectory = (
                        self.trajectory_generator.generate_trajectory(
                            np.array([-x, y]),
                            flipped_x_start_angle,
                            flipped_x_end_angle,
                            self.grid_resolution,
                        )
                    )
                    flipped_y_trajectory = (
                        self.trajectory_generator.generate_trajectory(
                            np.array([x, -y]),
                            flipped_y_start_angle,
                            flipped_y_end_angle,
                            self.grid_resolution,
                        )
                    )
                    flipped_xy_trajectory = (
                        self.trajectory_generator.generate_trajectory(
                            np.array([-x, -y]),
                            flipped_xy_start_angle,
                            flipped_xy_end_angle,
                            self.grid_resolution,
                        )
                    )

                    all_trajectories[
                        unflipped_trajectory.parameters.start_angle
                    ].append(unflipped_trajectory)
                    all_trajectories[
                        flipped_x_trajectory.parameters.start_angle
                    ].append(flipped_x_trajectory)
                    all_trajectories[
                        flipped_y_trajectory.parameters.start_angle
                    ].append(flipped_y_trajectory)
                    all_trajectories[
                        flipped_xy_trajectory.parameters.start_angle
                    ].append(flipped_xy_trajectory)

        return all_trajectories

    def _handle_motion_model(self, spanning_set: dict) -> dict:
        """
        Add the appropriate motions for the user supplied motion model.

        Ackerman: No additional trajectories

        Diff: In place turns to the right and left

        Omni: Diff + Sliding movements to right and left

        Args:
        spanning set: dict
            The minimal spanning set

        Returns
        -------
        dict
            The minimal spanning set with additional trajectories based
            on the motion model

        """
        if self.motion_model == self.MotionModel.ACKERMANN:
            return spanning_set

        elif self.motion_model == self.MotionModel.DIFF:
            diff_spanning_set = self._add_in_place_turns(spanning_set)
            return diff_spanning_set

        elif self.motion_model == self.MotionModel.OMNI:
            omni_spanning_set = self._add_in_place_turns(spanning_set)
            omni_spanning_set = self._add_horizontal_motions(omni_spanning_set)
            return omni_spanning_set

        else:
            print('No handling implemented for Motion Model: ' + f'{self.motion_model}')
            raise NotImplementedError

    def _add_in_place_turns(self, spanning_set: dict) -> dict:
        """
        Add in place turns to the spanning set.

        In place turns are trajectories with only a rotational component and
        only shift a single angular heading step

        Args:
        spanning_set: dict
            The minimal spanning set

        Returns
        -------
        dict
            The minimal spanning set containing additional in place turns
            for each start angle

        """
        all_angles = sorted(spanning_set.keys())

        for idx, start_angle in enumerate(all_angles):
            prev_angle_idx = idx - 1 if idx - 1 >= 0 else len(all_angles) - 1
            next_angle_idx = idx + 1 if idx + 1 < len(all_angles) else 0

            prev_angle = all_angles[prev_angle_idx]
            next_angle = all_angles[next_angle_idx]

            left_turn_params = TrajectoryParameters.no_arc(
                end_point=np.array([0, 0]),
                start_angle=start_angle,
                end_angle=next_angle,
            )
            right_turn_params = TrajectoryParameters.no_arc(
                end_point=np.array([0, 0]),
                start_angle=start_angle,
                end_angle=prev_angle,
            )

            # Calculate number of steps needed to rotate by roughly 10 degrees
            # for each pose
            angle_dif = angle_difference(start_angle, next_angle)
            steps = int(round(angle_dif / np.deg2rad(10))) + 1

            position = np.full(steps, 0)
            left_yaws = interpolate_yaws(start_angle, next_angle, True, steps)
            right_yaws = interpolate_yaws(start_angle, prev_angle, False, steps)

            left_turn_path = Path(xs=position, ys=position, yaws=left_yaws)
            right_turn_path = Path(xs=position, ys=position, yaws=right_yaws)

            left_turn = Trajectory(parameters=left_turn_params, path=left_turn_path)
            right_turn = Trajectory(parameters=right_turn_params, path=right_turn_path)

            spanning_set[start_angle].append(left_turn)
            spanning_set[start_angle].append(right_turn)

        return spanning_set

    def _add_horizontal_motions(self, spanning_set: dict) -> dict:
        """
        Add horizontal sliding motions to the spanning set.

        The horizontal sliding motions are simply straight line trajectories
        at 90 degrees to every start angle in the spanning set. The yaw of these
        trajectories is the same as the start angle for which it is generated.

        Args:
        spanning_set: dict
            The minimal spanning set

        Returns
        -------
        dict
            The minimal spanning set containing additional sliding motions
            for each start angle

        """
        # Calculate the offset in the headings list that represents an
        # angle change of 90 degrees
        idx_offset = int(self.num_of_headings / 4)

        for idx, angle in enumerate(self.headings):

            # Copy the straight line trajectory for the start angle that
            # is 90 degrees to the left
            left_angle_idx = int((idx + idx_offset) % self.num_of_headings)
            left_angle = self.headings[left_angle_idx]
            left_trajectories = spanning_set[left_angle]
            left_straight_trajectory = next(
                t for t in left_trajectories if t.parameters.end_angle == left_angle
            )

            # Copy the straight line trajectory for the start angle that
            # is 90 degrees to the right
            right_angle_idx = int((idx - idx_offset) % self.num_of_headings)
            right_angle = self.headings[right_angle_idx]
            right_trajectories = spanning_set[right_angle]
            right_straight_trajectory = next(
                t for t in right_trajectories if t.parameters.end_angle == right_angle
            )

            yaws = np.full(
                len(left_straight_trajectory.path.xs), angle, dtype=np.float64
            )

            # Create a new set of parameters that represents
            # the left sliding motion
            parmas_l = left_straight_trajectory.parameters
            left_motion_parameters = TrajectoryParameters(
                parmas_l.turning_radius,
                parmas_l.x_offset,
                parmas_l.y_offset,
                parmas_l.end_point,
                angle,
                angle,
                parmas_l.left_turn,
                parmas_l.arc_start_point,
                parmas_l.arc_end_point,
            )

            # Create a new set of parameters that represents
            # the right sliding motion
            params_r = right_straight_trajectory.parameters
            right_motion_parameters = TrajectoryParameters(
                params_r.turning_radius,
                params_r.x_offset,
                params_r.y_offset,
                params_r.end_point,
                angle,
                angle,
                params_r.left_turn,
                parmas_l.arc_start_point,
                parmas_l.arc_end_point,
            )

            left_motion = Trajectory(
                parameters=left_motion_parameters,
                path=Path(
                    xs=left_straight_trajectory.path.xs,
                    ys=left_straight_trajectory.path.ys,
                    yaws=yaws,
                ),
            )

            right_motion = Trajectory(
                parameters=right_motion_parameters,
                path=Path(
                    xs=right_straight_trajectory.path.xs,
                    ys=right_straight_trajectory.path.ys,
                    yaws=yaws,
                ),
            )

            spanning_set[angle].append(left_motion)
            spanning_set[angle].append(right_motion)

        return spanning_set

    def run(self):
        """
        Run the lattice generator.

        Returns
        -------
        dict
            The minimal spanning set including additional motions for the
            specified motion model

        """
        complete_spanning_set = self._generate_minimal_spanning_set()

        return self._handle_motion_model(complete_spanning_set)
