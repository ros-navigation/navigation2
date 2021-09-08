from collections import defaultdict
from enum import Enum

import numpy as np
from rtree import index

from helper import angle_difference, interpolate_yaws
from trajectory import Trajectory, TrajectoryParameters, TrajectoryPath
from trajectory_generator import TrajectoryGenerator


class LatticeGenerator:
    class MotionModel(Enum):
        ACKERMANN = 1
        DIFF = 2
        OMNI = 3

    class Flip(Enum):
        X = 1
        Y = 2
        BOTH = 3
    
    def __init__(self, config: dict):
        self.trajectory_generator = TrajectoryGenerator(config)
        self.grid_resolution = config["grid_resolution"]
        self.turning_radius = config["turning_radius"]
        self.stopping_threshold = config["stopping_threshold"]
        self.num_of_headings = config["num_of_headings"]
        self.headings = self.get_heading_discretization(config["num_of_headings"])

        self.motion_model = self.MotionModel[config["motion_model"].upper()]

        self.DISTANCE_THRESHOLD = 0.5 * self.grid_resolution
        self.ROTATION_THRESHOLD = 0.5 * (2 * np.pi / self.num_of_headings)

    def get_coords_at_level(self, level: int) -> np.array:
        positions = []

        max_point_coord = self.grid_resolution * level

        for i in range(level):
            varying_point_coord = self.grid_resolution * i

            # Varying y-coord
            positions.append((max_point_coord, varying_point_coord))

            # Varying x-coord
            positions.append((varying_point_coord, max_point_coord))

        # Append the corner
        positions.append((max_point_coord, max_point_coord))

        return np.array(positions)

    def get_heading_discretization(self, number_of_headings) -> list:
        max_val = int((((number_of_headings + 4) / 4) - 1) / 2)

        outer_edge_x = []
        outer_edge_y = []

        for i in range(-max_val, max_val + 1):
            outer_edge_x += [i, i]
            outer_edge_y += [-max_val, max_val]

            if i != max_val and i != -max_val:
                outer_edge_y += [i, i]
                outer_edge_x += [-max_val, max_val]

        return sorted([np.arctan2(j, i) for i, j in zip(outer_edge_x, outer_edge_y)])

    def point_to_line_distance(self, p1, p2, q):
        """
        Return minimum distance from q to line segment defined by p1, p2.
            Projects q onto line segment p1, p2 and returns the distance
        """

        # Get back the l2-norm without the square root
        l2 = np.inner(p1 - p2, p1 - p2)

        if l2 == 0:
            return np.linalg.norm(p1 - q)

        # Ensure t lies in [0, 1]
        t = max(0, min(1, np.dot(q - p1, p2 - p1) / l2))
        projected_point = p1 + t * (p2 - p1)

        return np.linalg.norm(q - projected_point)

    def is_minimal_trajectory(self, trajectory: Trajectory, prior_end_poses):

        for x1, y1, x2, y2, yaw in zip(
            trajectory.path.xs[:-1],
            trajectory.path.ys[:-1],
            trajectory.path.xs[1:],
            trajectory.path.ys[1:],
            trajectory.path.yaws[:-1],
        ):

            p1 = np.array([x1, y1])
            p2 = np.array([x2, y2])

            left_bb = min(x1, x2) - self.DISTANCE_THRESHOLD
            right_bb = max(x1, x2) + self.DISTANCE_THRESHOLD
            top_bb = max(y1, y2) + self.DISTANCE_THRESHOLD
            bottom_bb = min(y1, y2) - self.DISTANCE_THRESHOLD

            for prior_end_pose in prior_end_poses.intersection(
                (left_bb, bottom_bb, right_bb, top_bb), objects="raw"
            ):
                if (
                    self.point_to_line_distance(p1, p2, prior_end_pose[:-1])
                    < self.DISTANCE_THRESHOLD
                    and angle_difference(yaw, prior_end_pose[-1])
                    < self.ROTATION_THRESHOLD
                ):
                    return False

        return True

    def compute_min_trajectory_length(self):

        # Compute arc length for a turn that moves from 0 degrees to the minimum heading difference
        heading_diff = [
            abs(self.headings[i + 1] - self.headings[i]) for i in range(len(self.headings) - 1)
        ]

        return self.turning_radius * min(heading_diff)

    def generate_minimal_spanning_set(self):
        quadrant1_end_poses = defaultdict(list)

        initial_headings = sorted(
            list(filter(lambda x: 0 <= x and x <= np.pi / 2, self.headings))
        )

        min_trajectory_length = self.compute_min_trajectory_length()
        start_level = int(np.round(min_trajectory_length / self.grid_resolution))

        for start_heading in initial_headings:
            iterations_without_trajectory = 0

            prior_end_poses = index.Index()

            current_level = start_level

            # To get target headings: sort headings radially and remove those that are more than 90 degrees away
            target_headings = sorted(
                self.headings, key=lambda x: (abs(x - start_heading), -x)
            )
            target_headings = list(
                filter(lambda x: abs(start_heading - x) <= np.pi / 2, target_headings)
            )

            while iterations_without_trajectory < self.stopping_threshold:
                iterations_without_trajectory += 1

                # Generate x,y coordinates for current level
                positions = self.get_coords_at_level(current_level)

                for target_point in positions:
                    for target_heading in target_headings:
                        # Use 10% of grid separation for finer granularity when checking if trajectory overlaps another already seen trajectory
                        trajectory = self.trajectory_generator.generate_trajectory(
                            target_point,
                            start_heading,
                            target_heading,
                            0.1 * self.grid_resolution,
                        )

                        if trajectory is not None:
                            # Check if path overlaps something in minimal spanning set
                            if self.is_minimal_trajectory(trajectory, prior_end_poses):
                                new_end_pose = np.array(
                                    [target_point[0], target_point[1], target_heading]
                                )

                                left_bb = target_point[0] - self.DISTANCE_THRESHOLD
                                right_bb = target_point[0] + self.DISTANCE_THRESHOLD
                                bottom_bb = target_point[1] - self.DISTANCE_THRESHOLD
                                top_bb = target_point[1] + self.DISTANCE_THRESHOLD

                                prior_end_poses.insert(
                                    0,
                                    (left_bb, bottom_bb, right_bb, top_bb),
                                    new_end_pose,
                                )

                                quadrant1_end_poses[start_heading].append(
                                    (target_point, target_heading)
                                )

                                iterations_without_trajectory = 0

                current_level += 1

        return self.create_complete_minimal_spanning_set(quadrant1_end_poses)

    def flip_angle(self, angle, flip_type):
        angle_idx = self.headings.index(angle)

        # TODO: Neesd to convert from any quadrant to any other quadrant

        if flip_type == self.Flip.X:
            heading_idx = (self.num_of_headings / 2 - 1) - angle_idx - 1
        elif flip_type == self.Flip.Y:
            heading_idx = self.num_of_headings - angle_idx - 2
        elif flip_type == self.Flip.BOTH:
            heading_idx = (angle_idx - (self.num_of_headings / 2)) % self.num_of_headings
        else:
            raise Exception(f'Unsupported flip type: {flip_type}')

        return self.headings[int(heading_idx)]

    def create_complete_minimal_spanning_set(self, single_quadrant_minimal_set):
        # Generate the paths for trajectories in all quadrants
        all_trajectories = defaultdict(list)

        for start_angle in single_quadrant_minimal_set.keys():

            for end_point, end_angle in single_quadrant_minimal_set[start_angle]:
                x, y = end_point

                # Prevent double adding trajectories that lie on axes (i.e. start and end angle are either both 0 or both pi/2)
                if start_angle == 0 and end_angle == 0:
                    unflipped_start_angle = 0.0
                    flipped_x_start_angle = np.pi

                    unflipped_end_angle = 0.0
                    flipped_x_end_angle = np.pi

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
                    flipped_x_start_angle = self.flip_angle(start_angle, self.Flip.X)
                    flipped_y_start_angle = self.flip_angle(start_angle, self.Flip.Y)
                    flipped_xy_start_angle = self.flip_angle(start_angle, self.Flip.BOTH)

                    unflipped_end_angle = end_angle
                    flipped_x_end_angle = self.flip_angle(end_angle, self.Flip.X)
                    flipped_y_end_angle = self.flip_angle(end_angle, self.Flip.Y)
                    flipped_xy_end_angle = self.flip_angle(end_angle, self.Flip.BOTH)

                    # Generate trajectories for all quadrants and use the grid separation as step distance
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

    def handle_motion_model(self, spanning_set):

        if self.motion_model == self.MotionModel.ACKERMANN:
            return spanning_set

        elif self.motion_model == self.MotionModel.DIFF:
            diff_spanning_set = self.add_in_place_turns(spanning_set)
            return diff_spanning_set

        elif self.motion_model == self.MotionModel.OMNI:
            omni_spanning_set = self.add_in_place_turns(spanning_set)
            omni_spanning_set = self.add_horizontal_motions(omni_spanning_set)
            return omni_spanning_set

        else:
            print(f"No handling implemented for Motion Model: {self.motion_model}")
            raise NotImplementedError

    def add_in_place_turns(self, spanning_set):
        all_angles = sorted(list(spanning_set.keys()))

        for idx, start_angle in enumerate(all_angles):
            prev_angle_idx = idx - 1 if idx - 1 >= 0 else len(all_angles) - 1
            next_angle_idx = idx + 1 if idx + 1 < len(all_angles) else 0

            prev_angle = all_angles[prev_angle_idx]
            next_angle = all_angles[next_angle_idx]

            left_turn_params = TrajectoryParameters.no_arc(
                end_point=np.array([0, 0]),
                start_angle=start_angle,
                end_angle=next_angle,
                left_turn=True,
            )
            right_turn_params = TrajectoryParameters.no_arc(
                end_point=np.array([0, 0]),
                start_angle=start_angle,
                end_angle=prev_angle,
                left_turn=False,
            )

            # Calculate number of steps needed to rotate by roughly 10 degrees for each pose
            angle_dif = angle_difference(start_angle, next_angle)
            steps = int(round(angle_dif / np.deg2rad(10))) + 1

            position = np.full(steps, 0)
            left_yaws = interpolate_yaws(start_angle, next_angle, True, steps)
            right_yaws = interpolate_yaws(start_angle, prev_angle, False, steps)

            left_turn_path = TrajectoryPath(xs=position, ys=position, yaws=left_yaws)
            right_turn_path = TrajectoryPath(xs=position, ys=position, yaws=right_yaws)

            left_turn = Trajectory(parameters=left_turn_params, path=left_turn_path)
            right_turn = Trajectory(parameters=right_turn_params, path=right_turn_path)

            spanning_set[start_angle].append(left_turn)
            spanning_set[start_angle].append(right_turn)

        return spanning_set

    def add_horizontal_motions(self, spanning_set):

        idx_offset = int(self.num_of_headings / 4)

        for idx, angle in enumerate(self.headings):
            left_angle_idx = int((idx + idx_offset) % self.num_of_headings)
            left_angle = self.headings[left_angle_idx]
            left_trajectories = spanning_set[left_angle]
            left_straight_trajectory = next(t for t in left_trajectories if t.parameters.end_angle == left_angle)

            right_angle_idx = int((idx - idx_offset) % self.num_of_headings)
            right_angle = self.headings[right_angle_idx]
            right_trajectories = spanning_set[right_angle]
            right_straight_trajectory = next(t for t in right_trajectories if t.parameters.end_angle == right_angle)

            yaws = np.full(len(left_straight_trajectory.path.xs), angle, dtype=np.float64)

            parmas_l = left_straight_trajectory.parameters
            left_motion_parameters = TrajectoryParameters(parmas_l.turning_radius, parmas_l.x_offset, parmas_l.y_offset, parmas_l.end_point, angle, angle, parmas_l.left_turn, parmas_l.start_to_arc_distance, parmas_l.arc_to_end_distance)

            params_r = right_straight_trajectory.parameters
            right_motion_parameters = TrajectoryParameters(params_r.turning_radius, params_r.x_offset, params_r.y_offset, params_r.end_point, angle, angle, params_r.left_turn, params_r.start_to_arc_distance, params_r.arc_to_end_distance)

            left_motion = Trajectory(
                parameters=left_motion_parameters,
                path=TrajectoryPath(xs=left_straight_trajectory.path.xs, ys=left_straight_trajectory.path.ys, yaws=yaws),
            )

            right_motion = Trajectory(
                parameters=right_motion_parameters,
                path=TrajectoryPath(xs=right_straight_trajectory.path.xs, ys=right_straight_trajectory.path.ys, yaws=yaws),
            )

            spanning_set[angle].append(left_motion)
            spanning_set[angle].append(right_motion)

        return spanning_set

    def run(self):
        complete_spanning_set = self.generate_minimal_spanning_set()

        return self.handle_motion_model(complete_spanning_set)
