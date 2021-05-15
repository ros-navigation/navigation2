from trajectory_generator import TrajectoryGenerator
import numpy as np
from collections import defaultdict

import math
import time

class LatticeGenerator:

    def __init__(self, config):
        self.grid_separation = config["grid_separation"]
        self.trajectory_generator = TrajectoryGenerator(config)
        self.turning_radius = config["turning_radius"]
        self.max_level = round(config["max_length"] / self.grid_separation)
        self.number_of_headings = config["number_of_headings"]
        
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

        return [np.rad2deg(np.arctan2(i, j)) for i, j in zip(outer_edge_x, outer_edge_y)]

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

    def is_minimal_path(self, xs, ys, minimal_spanning_trajectories):

        yaws = [np.arctan2((yf - yi), (xf - xi)) for xi, yi, xf, yf in zip(xs[:-1], ys[:-1], xs[1:], ys[1:])]

        distance_threshold = 0.5 * self.grid_separation
        rotation_threshold = 0.5 * np.deg2rad(360 / self.number_of_headings)

        for x1, y1, x2, y2, yaw in zip(xs[:-1], ys[:-1], xs[1:], ys[1:], yaws[:-1]):

            p1 = np.array([x1, y1])
            p2 = np.array([x2, y2])

            for prior_end_point in minimal_spanning_trajectories:

                # TODO: point_to_line_distance gives direct distance which means the distance_threshold represents a circle 
                # around each point. Change so that we calculate manhattan distance? <- d_t will represent a box instead
                if self.point_to_line_distance(p1, p2, prior_end_point[:-1]) < distance_threshold \
                    and self.angle_difference(yaw, prior_end_point[-1]) < rotation_threshold:
                    return False
        
        return True
            

    def generate_minimal_spanning_set(self):
        s = time.time()
        result = defaultdict(list)

        all_headings = self.get_heading_discretization()

        initial_headings = sorted(list(filter(lambda x: 0 <= x and x < 90, all_headings)))

        start_level = int(round(self.turning_radius * np.cos(np.arctan(1/2) - np.pi / 2) / self.grid_separation))

        for start_heading in initial_headings:
            
            minimal_trajectory_set = []
            current_level = start_level

            # To get target headings: sort headings radially and remove those that are more than 90 degrees away
            target_headings = sorted(all_headings, key=lambda x: (abs(x - start_heading),-x))
            target_headings = list(filter(lambda x : abs(start_heading - x) <= 90, target_headings))
                
            while current_level <= self.max_level:

                # Generate x,y coordinates for current level
                positions = self.get_coords_at_level(current_level)

                for target_point in positions:
                    for target_heading in target_headings:
                        xs, ys = self.trajectory_generator.generate_trajectory(target_point, start_heading, target_heading)

                        if len(xs) != 0:

                            # Check if path overlaps something in minimal spanning set
                            if(self.is_minimal_path(xs, ys, minimal_trajectory_set)):
                                new_end_point = np.array([target_point[0], target_point[1], np.deg2rad(target_heading)])
                                minimal_trajectory_set.append(new_end_point)

                                result[start_heading].append((target_point, target_heading))

                current_level += 1

        print("Time elapsed = ", time.time() - s)    

        return result

if __name__ == "__main__":

    import matplotlib.pyplot as plt

    test = LatticeGenerator({"turning_radius":0.4,"step_distance":0.005, "grid_separation":0.05, "max_length":1, "number_of_headings": 16})

    result = test.generate_minimal_spanning_set()

    print("\n\n")

    fig = plt.figure()

    for i,start_angle in enumerate(result.keys(), 1):
        ax = fig.add_subplot(2,2,i)

        for end_point, end_angle in result[start_angle]:
            xs, ys = test.trajectory_generator.generate_trajectory(end_point, start_angle, end_angle)
            ax.plot(xs, ys, "b")
            plt.axis("square")
            ax.set_xlim([0,.8])
            ax.set_ylim([0,.8])
            plt.grid(True)
            plt.title(f'{round(start_angle,2)}')
            print(start_angle, end_angle, end_point)
    
    plt.show()