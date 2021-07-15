from trajectory_generator import TrajectoryGenerator
from trajectory import Trajectory, TrajectoryPath, TrajectoryParameters
import numpy as np
from collections import defaultdict

import math

class LatticeGenerator:

    def __init__(self, config):
        self.grid_separation = config["gridSeparation"]
        self.trajectory_generator = TrajectoryGenerator(config)
        self.turning_radius = config["turningRadius"]
        self.max_level = round(config["maxLength"] / self.grid_separation)
        self.number_of_headings = config["numberOfHeadings"]
        
    def angle_difference(self, angle_1, angle_2):

        difference = abs(angle_1 - angle_2)
        
        if difference > math.pi:
            # If difference > 180 return the shorter distance between the angles
            difference = 2*math.pi - difference
        
        return difference

    def get_coords_at_level(self, level):
        positions = []

        max_point_coord = self.grid_separation * level

        for i in range(level):
            varying_point_coord = self.grid_separation * i

            # Varying y-coord
            positions.append((max_point_coord, varying_point_coord))
            
            # Varying x-coord
            positions.append((varying_point_coord, max_point_coord))


        # Append the corner
        positions.append((max_point_coord, max_point_coord))

        return np.array(positions)

    def get_heading_discretization(self):
        max_val = int((((self.number_of_headings + 4)/4) -1) / 2)

        outer_edge_x = []
        outer_edge_y = []

        for i in range(-max_val, max_val+1):
            outer_edge_x += [i, i]
            outer_edge_y += [-max_val, max_val]

            if i != max_val and i != -max_val:
                outer_edge_y += [i, i]
                outer_edge_x += [-max_val, max_val]

        return [np.rad2deg(np.arctan2(j, i)) for i, j in zip(outer_edge_x, outer_edge_y)]

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

    def is_minimal_path(self, trajectory_path: TrajectoryPath, minimal_spanning_trajectories):
    
        distance_threshold = 0.5 * self.grid_separation
        rotation_threshold = 0.5 * np.deg2rad(360 / self.number_of_headings)

        for x1, y1, x2, y2, yaw in zip(trajectory_path.xs[:-1], trajectory_path.ys[:-1], trajectory_path.xs[1:], trajectory_path.ys[1:], trajectory_path.yaws[:-1]):

            p1 = np.array([x1, y1])
            p2 = np.array([x2, y2])

            for prior_end_point in minimal_spanning_trajectories:

                # TODO: point_to_line_distance gives direct distance which means the distance_threshold represents a circle 
                # around each point. Change so that we calculate manhattan distance? <- d_t will represent a box instead
                if self.point_to_line_distance(p1, p2, prior_end_point[:-1]) < distance_threshold \
                    and self.angle_difference(yaw, prior_end_point[-1]) < rotation_threshold:
                    return False
        
        return True

    def compute_min_trajectory_length(self):
        # Compute arc length of circle that moves through an angle of 360/number of headings 
        return 2 * math.pi * self.turning_radius * (1/self.number_of_headings)

    def generate_minimal_spanning_set(self):
        single_quadrant_spanning_set = defaultdict(list)

        # Firstly generate the minimal set for a single quadrant
        single_quadrant_headings = self.get_heading_discretization()

        initial_headings = sorted(list(filter(lambda x: 0 <= x and x < 90, single_quadrant_headings)))

        min_trajectory_length = self.compute_min_trajectory_length()
        start_level = int(np.floor(min_trajectory_length / self.grid_separation))

        for start_heading in initial_headings:
            
            minimal_trajectory_set = []
            current_level = start_level

            # To get target headings: sort headings radially and remove those that are more than 90 degrees away
            target_headings = sorted(single_quadrant_headings, key=lambda x: (abs(x - start_heading),-x))
            target_headings = list(filter(lambda x : abs(start_heading - x) <= 90, target_headings))
                
            while current_level <= self.max_level:

                # Generate x,y coordinates for current level
                positions = self.get_coords_at_level(current_level)

                for target_point in positions:
                    for target_heading in target_headings:
                        trajectory = self.trajectory_generator.generate_trajectory(target_point, start_heading, target_heading)

                        if trajectory:

                            # Check if path overlaps something in minimal spanning set
                            if(self.is_minimal_path(trajectory.path, minimal_trajectory_set)):
                                new_end_pose = np.array([target_point[0], target_point[1], np.deg2rad(target_heading)])
                                minimal_trajectory_set.append(new_end_pose)

                                single_quadrant_spanning_set[start_heading].append((target_point, target_heading))

                current_level += 1

        return self.create_complete_minimal_spanning_set(single_quadrant_spanning_set)

    def create_complete_minimal_spanning_set(self, single_quadrant_minimal_set):
        # Copy the 0 degree trajectories to 90 degerees
        end_points_for_90 = []

        for end_point, end_angle in single_quadrant_minimal_set[0.0]:
            x, y = end_point

            end_points_for_90.append((np.array([y, x]), 90 - end_angle))

        single_quadrant_minimal_set[90.0] = end_points_for_90

        # Generate the paths for all trajectories
        all_trajectories = defaultdict(list)
       
        for start_angle in single_quadrant_minimal_set.keys():

            for end_point, end_angle in single_quadrant_minimal_set[start_angle]:
                trajectory = self.trajectory_generator.generate_trajectory(end_point, start_angle, end_angle, step_distance=self.grid_separation)

                xs = trajectory.path.xs.round(5)
                ys = trajectory.path.ys.round(5)

                flipped_xs = [-x for x in xs]
                flipped_ys = [-y for y in ys]

                yaws_quad1 = [np.arctan2((yf - yi), (xf - xi)) for xi, yi, xf, yf in zip(xs[:-1], ys[:-1], xs[1:], ys[1:])] + [np.deg2rad(end_angle)]
                yaws_quad2 = [np.arctan2((yf - yi), (xf - xi)) for xi, yi, xf, yf in zip(flipped_xs[:-1], ys[:-1], flipped_xs[1:], ys[1:])] + [np.pi - np.deg2rad(end_angle)]
                yaws_quad3 = [np.arctan2((yf - yi), (xf - xi)) for xi, yi, xf, yf in zip(flipped_xs[:-1], flipped_ys[:-1], flipped_xs[1:], flipped_ys[1:])]  + [-np.pi + np.deg2rad(end_angle)]
                yaws_quad4 = [np.arctan2((yf - yi), (xf - xi)) for xi, yi, xf, yf in zip(xs[:-1], flipped_ys[:-1], xs[1:], flipped_ys[1:])] + [-np.deg2rad(end_angle)]

                arc_length = 2 * np.pi * trajectory.parameters.radius * abs(start_angle - end_angle) / 360.0
                straight_length = trajectory.parameters.start_to_arc_distance + trajectory.parameters.arc_to_end_distance
                trajectory_length = arc_length + trajectory.parameters.start_to_arc_distance + trajectory.parameters.arc_to_end_distance

                trajectory_info = (trajectory.parameters.radius, trajectory_length, arc_length, straight_length)

                '''
                Quadrant 1: +x, +y
                Quadrant 2: -x, +y
                Quadrant 3: -x, -y
                Quadrant 4: +x, -y
                '''
                
                # Special cases for trajectories that run straight across the axis
                if start_angle == 0 and end_angle == 0:
                    quadrant_1 = (0.0, 0.0, *trajectory_info, list(zip(xs, ys, yaws_quad1)))
                    quadrant_2 = (-180, -180, *trajectory_info, list(zip(flipped_xs, ys, yaws_quad2)))
                    
                    all_trajectories[quadrant_1[0]].append(quadrant_1)
                    all_trajectories[quadrant_2[0]].append(quadrant_2)
                
                elif (start_angle == 90 and end_angle == 90):
                    quadrant_1 = (start_angle, end_angle, *trajectory_info, list(zip(xs, ys, yaws_quad1)))
                    quadrant_4 = (-start_angle, -end_angle, *trajectory_info, list(zip(xs, flipped_ys, yaws_quad4)))

                    all_trajectories[quadrant_1[0]].append(quadrant_1)
                    all_trajectories[quadrant_4[0]].append(quadrant_4)

                else:
                    # Need to prevent 180 or -0 being added as a start or end angle
                    if start_angle == 0:
                        quadrant_1 = (start_angle, end_angle, *trajectory_info, list(zip(xs, ys, yaws_quad1)))
                        quadrant_2 = (-180, 180 - end_angle, *trajectory_info, list(zip(flipped_xs, ys, yaws_quad2)))
                        quadrant_3 = (-180, end_angle - 180, *trajectory_info, list(zip(flipped_xs, flipped_ys, yaws_quad3)))
                        quadrant_4 = (start_angle, -end_angle, *trajectory_info, list(zip(xs, flipped_ys, yaws_quad4)))

                    elif end_angle == 0:
                        quadrant_1 = (start_angle, end_angle, *trajectory_info, list(zip(xs, ys, yaws_quad1)))
                        quadrant_2 = (180 - start_angle, -180, *trajectory_info, list(zip(flipped_xs, ys, yaws_quad2)))
                        quadrant_3 = (start_angle - 180, -180, *trajectory_info, list(zip(flipped_xs, flipped_ys, yaws_quad3)))
                        quadrant_4 = (-start_angle, end_angle, *trajectory_info, list(zip(xs, flipped_ys, yaws_quad4)))

                    else:
                        quadrant_1 = (start_angle, end_angle, *trajectory_info, list(zip(xs, ys, yaws_quad1)))
                        quadrant_2 = (180 - start_angle, 180 - end_angle, *trajectory_info, list(zip(flipped_xs, ys, yaws_quad2)))
                        quadrant_3 = (start_angle - 180, end_angle - 180, *trajectory_info, list(zip(flipped_xs, flipped_ys, yaws_quad3)))
                        quadrant_4 = (-start_angle, -end_angle, *trajectory_info, list(zip(xs, flipped_ys, yaws_quad4)))

                    all_trajectories[quadrant_1[0]].append(quadrant_1)
                    all_trajectories[quadrant_2[0]].append(quadrant_2)
                    all_trajectories[quadrant_3[0]].append(quadrant_3)
                    all_trajectories[quadrant_4[0]].append(quadrant_4)

        return all_trajectories

    def run(self):
        return self.generate_minimal_spanning_set()


        