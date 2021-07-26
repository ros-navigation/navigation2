from collections import defaultdict

import numpy as np

from helper import angle_difference, normalize_angle
from motion_model import MotionModel
from trajectory import Trajectory, TrajectoryParameters, TrajectoryPath
from trajectory_generator import TrajectoryGenerator


class LatticeGenerator:

    def __init__(self, config):
        self.trajectory_generator = TrajectoryGenerator(config)
        self.grid_resolution = config["grid_resolution"]
        self.turning_radius = config["turning_radius"]
        self.max_level = round(config["max_length"] / self.grid_resolution)
        self.num_of_headings = config["num_of_headings"]

        self.motion_model = MotionModel[config["motion_model"].upper()]

    def get_coords_at_level(self, level):
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

    def get_heading_discretization(self):
        max_val = int((((self.num_of_headings + 4)/4) - 1) / 2)

        outer_edge_x = []
        outer_edge_y = []

        for i in range(-max_val, max_val+1):
            outer_edge_x += [i, i]
            outer_edge_y += [-max_val, max_val]

            if i != max_val and i != -max_val:
                outer_edge_y += [i, i]
                outer_edge_x += [-max_val, max_val]

        return [np.arctan2(j, i) for i, j in zip(outer_edge_x, outer_edge_y)]

    def point_to_line_distance(self, p1, p2, q):
        '''
        Return minimum distance from q to line segment defined by p1, p2.
            Projects q onto line segment p1, p2 and returns the distance
        '''

        # Get back the l2-norm without the square root
        l2 = np.inner(p1-p2, p1-p2)

        if l2 == 0:
            return np.linalg.norm(p1 - q)

        # Ensure t lies in [0, 1]
        t = max(0, min(1, np.dot(q - p1, p2 - p1) / l2))
        projected_point = p1 + t * (p2 - p1)

        return np.linalg.norm(q - projected_point)

    def is_minimal_trajectory(self, trajectory: Trajectory, minimal_spanning_trajectories):

        distance_threshold = 0.5 * self.grid_resolution
        rotation_threshold = 0.5 * (2 * np.pi / self.num_of_headings)

        for x1, y1, x2, y2, yaw in zip(trajectory.path.xs[:-1], trajectory.path.ys[:-1], trajectory.path.xs[1:], trajectory.path.ys[1:], trajectory.path.yaws[:-1]):

            p1 = np.array([x1, y1])
            p2 = np.array([x2, y2])

            for prior_end_point in minimal_spanning_trajectories:
                # TODO: point_to_line_distance gives direct distance which means the distance_threshold represents a circle
                # around each point. Change so that we calculate manhattan distance? <- d_t will represent a box instead
                if self.point_to_line_distance(p1, p2, prior_end_point[:-1]) < distance_threshold \
                        and angle_difference(yaw, prior_end_point[-1]) < rotation_threshold:
                    return False

        return True

    def compute_min_trajectory_length(self):
        # Compute arc length of circle that moves through an angle of 360/number of headings
        return 2 * np.pi * self.turning_radius * (360/self.num_of_headings) / 360

    def generate_minimal_spanning_set(self):
        quadrant1_end_poses = defaultdict(list)

        heading_discretization = self.get_heading_discretization()

        initial_headings = sorted(
            list(filter(lambda x: 0 <= x and x <= np.pi/2, heading_discretization)))

        min_trajectory_length = self.compute_min_trajectory_length()
        start_level = int(
            np.floor(min_trajectory_length / self.grid_resolution))

        for start_heading in initial_headings:
            minimal_trajectory_end_poses = []

            current_level = start_level

            # To get target headings: sort headings radially and remove those that are more than 90 degrees away
            target_headings = sorted(
                heading_discretization, key=lambda x: (abs(x - start_heading), -x))
            target_headings = list(filter(lambda x: abs(
                start_heading - x) <= np.pi/2, target_headings))

            while current_level <= self.max_level:

                # Generate x,y coordinates for current level
                positions = self.get_coords_at_level(current_level)

                for target_point in positions:
                    for target_heading in target_headings:
                        # Use 10% of grid separation for finer granularity when checking if trajectory overlaps another already seen trajectory
                        trajectory = self.trajectory_generator.generate_trajectory(
                            target_point, start_heading, target_heading, 0.1 * self.grid_resolution)

                        if trajectory is not None:
                            # Check if path overlaps something in minimal spanning set
                            if(self.is_minimal_trajectory(trajectory, minimal_trajectory_end_poses)):
                                new_end_pose = np.array(
                                    [target_point[0], target_point[1], target_heading])
                                minimal_trajectory_end_poses.append(
                                    new_end_pose)

                                quadrant1_end_poses[start_heading].append(
                                    (target_point, target_heading))

                current_level += 1

        return self.create_complete_minimal_spanning_set(quadrant1_end_poses)

    def create_complete_minimal_spanning_set(self, single_quadrant_minimal_set):
        # Generate the paths for all trajectories
        all_trajectories = defaultdict(list)

        for start_angle in single_quadrant_minimal_set.keys():

            for end_point, end_angle in single_quadrant_minimal_set[start_angle]:
                x, y = end_point

                # Prevent double adding trajectories that lie on axes (i.e. start and end angle are either both 0 or both pi/2)
                if start_angle == 0 and end_angle == 0:
                    quadrant1_start_angle = 0.0
                    quadrant3_start_angle = -np.pi

                    quadrant1_end_angle = 0.0
                    quadrant3_end_angle = -np.pi

                    quadrant1_trajectory = self.trajectory_generator.generate_trajectory(np.array(
                        [x, y]), quadrant1_start_angle, quadrant1_end_angle, self.grid_resolution)
                    quadrant3_trajectory = self.trajectory_generator.generate_trajectory(np.array(
                        [-x, -y]), quadrant3_start_angle, quadrant3_end_angle, self.grid_resolution)

                    all_trajectories[quadrant1_trajectory.parameters.start_angle].append(
                        quadrant1_trajectory)

                    all_trajectories[quadrant3_trajectory.parameters.start_angle].append(
                        quadrant3_trajectory)
                elif abs(start_angle) == np.pi/2 and abs(end_angle) == np.pi/2:
                    quadrant2_start_angle = np.pi/2
                    quadrant4_start_angle = -np.pi/2

                    quadrant2_end_angle = np.pi/2
                    quadrant4_end_angle = -np.pi/2

                    quadrant2_trajectory = self.trajectory_generator.generate_trajectory(np.array(
                        [-x, y]), quadrant2_start_angle, quadrant2_end_angle, self.grid_resolution)

                    quadrant4_trajectory = self.trajectory_generator.generate_trajectory(np.array(
                        [x, -y]), quadrant4_start_angle, quadrant4_end_angle, self.grid_resolution)

                    all_trajectories[quadrant2_trajectory.parameters.start_angle].append(
                        quadrant2_trajectory)
                    all_trajectories[quadrant4_trajectory.parameters.start_angle].append(
                        quadrant4_trajectory)
                else:
                    quadrant1_start_angle = start_angle
                    quadrant2_start_angle = normalize_angle(
                        np.pi - start_angle) if start_angle != 0 else -np.pi
                    quadrant3_start_angle = normalize_angle(
                        start_angle - np.pi)
                    quadrant4_start_angle = normalize_angle(
                        -start_angle) if start_angle != 0 else 0.0

                    quadrant1_end_angle = end_angle
                    quadrant2_end_angle = normalize_angle(
                        np.pi - end_angle) if end_angle != 0 else -np.pi
                    quadrant3_end_angle = normalize_angle(end_angle - np.pi)
                    quadrant4_end_angle = normalize_angle(
                        -end_angle) if end_angle != 0 else 0.0

                    # Generate trajectories for all quadrants and use the grid separation as step distance
                    quadrant1_trajectory = self.trajectory_generator.generate_trajectory(np.array(
                        [x, y]), quadrant1_start_angle, quadrant1_end_angle, self.grid_resolution)
                    quadrant2_trajectory = self.trajectory_generator.generate_trajectory(np.array(
                        [-x, y]), quadrant2_start_angle, quadrant2_end_angle, self.grid_resolution)
                    quadrant3_trajectory = self.trajectory_generator.generate_trajectory(np.array(
                        [-x, -y]), quadrant3_start_angle, quadrant3_end_angle, self.grid_resolution)
                    quadrant4_trajectory = self.trajectory_generator.generate_trajectory(np.array(
                        [x, -y]), quadrant4_start_angle, quadrant4_end_angle, self.grid_resolution)

                    all_trajectories[quadrant1_trajectory.parameters.start_angle].append(
                        quadrant1_trajectory)
                    all_trajectories[quadrant2_trajectory.parameters.start_angle].append(
                        quadrant2_trajectory)
                    all_trajectories[quadrant3_trajectory.parameters.start_angle].append(
                        quadrant3_trajectory)
                    all_trajectories[quadrant4_trajectory.parameters.start_angle].append(
                        quadrant4_trajectory)

        return all_trajectories

    def handle_motion_model(self, spanning_set):

        if self.motion_model == MotionModel.ACKERMANN:
            return spanning_set

        elif self.motion_model == MotionModel.DIFF:
            diff_spanning_set = self.add_in_place_turns(spanning_set)
            return diff_spanning_set

        elif self.motion_model == MotionModel.OMNI:
            omni_spanning_set = self.add_in_place_turns(spanning_set)
            omni_spanning_set = self.add_horizontal_motions(omni_spanning_set)
            return omni_spanning_set

        else:
            print(
                f"No handling implemented for Motion Model: {self.motion_model}")
            raise NotImplementedError

    def add_in_place_turns(self, spanning_set):
        all_angles = sorted(spanning_set.keys(), key=spanning_set.get)

        for idx, start_angle in enumerate(all_angles):
            prev_angle_idx = idx - 1 if idx - 1 >= 0 else len(all_angles) - 1
            next_angle_idx = idx + 1 if idx + 1 < len(all_angles) else 0

            prev_angle = all_angles[prev_angle_idx]
            next_angle = all_angles[next_angle_idx]

            left_turn_params = TrajectoryParameters.no_arc(end_point=np.array(
                [0, 0]), start_angle=start_angle, end_angle=prev_angle, left_turn=True)
            right_turn_params = TrajectoryParameters.no_arc(end_point=np.array(
                [0, 0]), start_angle=start_angle, end_angle=next_angle, left_turn=False)

            left_turn_path = TrajectoryPath(xs=np.array([0.0, 0.0]), ys=np.array(
                [0, 0]), yaws=np.array([start_angle, prev_angle]))
            right_turn_path = TrajectoryPath(xs=np.array([0.0, 0.0]), ys=np.array(
                [0, 0]), yaws=np.array([start_angle, next_angle]))

            left_turn = Trajectory(
                parameters=left_turn_params, path=left_turn_path)
            right_turn = Trajectory(
                parameters=right_turn_params, path=right_turn_path)

            spanning_set[start_angle].append(left_turn)
            spanning_set[start_angle].append(right_turn)

        return spanning_set

    def add_horizontal_motions(self, spanning_set):
        min_trajectory_length = self.compute_min_trajectory_length()
        steps = int(np.round(min_trajectory_length/self.grid_resolution))

        for start_angle in spanning_set.keys():
            left_shift_xs = np.linspace(0, min_trajectory_length *
                                        np.cos(start_angle + np.pi/2), steps)
            left_shift_ys = np.linspace(0, min_trajectory_length *
                                        np.sin(start_angle + np.pi/2), steps)

            right_shift_xs = np.linspace(0, min_trajectory_length *
                                         np.cos(start_angle - np.pi/2), steps)
            right_shift_ys = np.linspace(0, min_trajectory_length *
                                         np.sin(start_angle - np.pi/2), steps)

            yaws = np.full(steps, start_angle, dtype=np.float64)

            left_end_point = np.array([left_shift_xs[-1], left_shift_ys[-1]])
            left_shift_params = TrajectoryParameters.no_arc(
                end_point=left_end_point, start_angle=start_angle, end_angle=start_angle, left_turn=True)

            right_end_point = np.array(
                [right_shift_xs[-1], right_shift_ys[-1]])
            right_shift_params = TrajectoryParameters.no_arc(
                end_point=right_end_point, start_angle=start_angle, end_angle=start_angle, left_turn=False)

            left_motion = Trajectory(parameters=left_shift_params, path=TrajectoryPath(
                xs=left_shift_xs, ys=left_shift_ys, yaws=yaws))
            right_motion = Trajectory(parameters=right_shift_params, path=TrajectoryPath(
                xs=right_shift_xs, ys=right_shift_ys, yaws=yaws))

            spanning_set[start_angle].append(left_motion)
            spanning_set[start_angle].append(right_motion)

        return spanning_set

    def run(self):
        complete_spanning_set = self.generate_minimal_spanning_set()

        return self.handle_motion_model(complete_spanning_set)
